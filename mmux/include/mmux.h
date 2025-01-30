#include <stdio.h>
#include <stdlib.h>
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <rom/ets_sys.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "math.h"

#define NUM_CHANNELS 64
#define NUM_CHANNELS_BOX 8

#define MMUX_NUM_FREQ_COEFF 4
#define MMUX_NUM_TEMP_COEFF 3
#define MMUX_NUM_COMP_COEFF 2

#define DEFAULT_CONFIG_PCB_V_1_1(_num_sensors) ((mmux_config_t){ \
    .shift_reg_data = GPIO_NUM_13,                               \
    .shift_reg_srclk = GPIO_NUM_21,                              \
    .shift_reg_rclk = GPIO_NUM_14,                               \
    .spi_host = SPI2_HOST,                                       \
                                                                 \
    .rx_box = GPIO_NUM_4,                                        \
    .tx_box = GPIO_NUM_3,                                        \
    .de_box = GPIO_NUM_9,                                        \
    .box_uart = UART_NUM_1,                                      \
    .box_serial_cfg = {.baud_rate = 1200,                        \
                       .data_bits = UART_DATA_7_BITS,            \
                       .parity = UART_PARITY_EVEN,               \
                       .stop_bits = UART_STOP_BITS_1,            \
                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    \
                       .rx_flow_ctrl_thresh = 122,               \
                       .source_clk = UART_SCLK_DEFAULT},         \
    .box_addr = '1',                                             \
    .measurement_timeout = 30,                                   \
                                                                 \
    .rx_ext = GPIO_NUM_18,                                       \
    .tx_ext = GPIO_NUM_17,                                       \
    .de_ext = GPIO_NUM_8,                                        \
    .ext_uart = UART_NUM_2,                                      \
    .ext_serial_cfg = {.baud_rate = 19200,                       \
                       .data_bits = UART_DATA_8_BITS,            \
                       .parity = UART_PARITY_DISABLE,            \
                       .stop_bits = UART_STOP_BITS_1,            \
                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    \
                       .rx_flow_ctrl_thresh = 122,               \
                       .source_clk = UART_SCLK_DEFAULT},         \
                                                                 \
    .num_max_meas = 5,                                           \
    .num_close_vals = 3,                                         \
    .delta_close_vals = 10.0,                                    \
    .num_sensors = _num_sensors,                                 \
})

typedef struct
{
    // GPIO Shift Register
    gpio_num_t shift_reg_data;  // shift register data input
    gpio_num_t shift_reg_srclk; // shift register data clock
    gpio_num_t shift_reg_rclk;  // shift register output clock
    spi_host_device_t spi_host;

    // GPIO RS485
    gpio_num_t rx_box;
    gpio_num_t tx_box;
    gpio_num_t de_box;
    uart_port_t box_uart;
    uart_config_t box_serial_cfg;
    char box_addr;
    uint8_t measurement_timeout; // seconds; ca. 3 seconds per active (box) channel

    gpio_num_t rx_ext;
    gpio_num_t tx_ext;
    gpio_num_t de_ext;
    uart_port_t ext_uart;
    uart_config_t ext_serial_cfg;

    // meaurement settings
    // the box will try to find a certain number of measurement results which are close to one another.
    // This enables filtering out of nonsense values.
    // If the specified number of values within the specified range cannot be found within the max number of tries, nan will be set.
    uint8_t num_max_meas;    // how often will be tried to measure a channel?
    uint8_t num_close_vals;  // how many values close to one another are sufficient?
    double delta_close_vals; // how close must the values be together?
    uint16_t num_sensors;
} mmux_config_t;

typedef struct
{
    gpio_num_t shift_reg_data;  // shift register data input
    gpio_num_t shift_reg_srclk; // shift register data clock
    gpio_num_t shift_reg_rclk;  // shift register output clock
    spi_host_device_t spi_host;
    spi_device_handle_t shift_reg;

    // GPIO RS485
    gpio_num_t rx_box;
    gpio_num_t tx_box;
    gpio_num_t de_box;
    uart_port_t box_uart;
    uart_config_t box_serial_cfg;
    char box_addr;
    uint8_t measurement_timeout; // seconds

    gpio_num_t rx_ext;
    gpio_num_t tx_ext;
    gpio_num_t de_ext;
    uart_port_t ext_uart;
    uart_config_t ext_serial_cfg;

    // meaurement settings
    // the box will try to find a certain number of measurement results which are close to one another.
    // This enables filtering out of nonsense values.
    // If the specified number of values within the specified range cannot be found within the max number of tries, nan will be set.
    uint8_t num_max_meas;    // how often will be tried to measure a channel?
    uint8_t num_close_vals;  // how many values close to one another are sufficient?
    double delta_close_vals; // how close must the values be together?
    uint16_t num_sensors;
    uint8_t num_pcbs;

    // sensor scaling
    double (*freq_coeffs)[MMUX_NUM_FREQ_COEFF]; // [5]*x^5  [4]*x^4  [3]*x^3  [2]*x^2  [1]*x^1  [0]*x^0
    bool *use_digits;                           // tells the code to switch from frequency to digits before using the formula

    double (*temp_coeffs)[MMUX_NUM_TEMP_COEFF]; // R0/Ohm; T0/°C; Beta OR C1 C2 C3 (Steinhart-Hart))
    bool *use_steinhart;

    double (*comp_coeffs)[MMUX_NUM_COMP_COEFF];
} mmux;

/// @brief copies the data from the config; sets up mmux struct
/// @param config mmux config struct; may be set via macro e.g. DEFAULT_CONFIG_PCB_V_1_1(NUM_SENSORS)
/// @param device mmux device to setup
void mmux_init(mmux_config_t *config, mmux *device);
esp_err_t mmux_begin(mmux *device);
void mmux_end(mmux *device);
void mmux_version(char buf[9]);

void mmux_set_freq_coeffs(mmux *device, const uint16_t channel, const double _freq_coeffs[MMUX_NUM_FREQ_COEFF], const bool _use_digits);
void mmux_set_temp_coeffs(mmux *device, const uint16_t channel, const double _temp_coeffs[MMUX_NUM_TEMP_COEFF], const bool _use_steinhart);
void mmux_set_comp_coeffs(mmux *device, const uint16_t channel, const double t0, const double alpha);
void mmux_log_info_freq_coeffs(mmux *device);

void mmux_scaleFrequencies(mmux *device, double *freq_raw_vals, double *freq_vals);
void mmux_scaleTemperatures(mmux *device, double *temp_raw_vals, double *temp_vals);
void mmux_compensateTemperature(mmux *device, double *freq_vals, double *temp_vals, double *freq_vals_comp);

esp_err_t mmux_measure(mmux *device, double *freq_raw_vals, double *temp_raw_vals);
