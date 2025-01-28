#include <stdio.h>
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <rom/ets_sys.h>
#include <string.h>
#include "driver/uart.h"

#define MMUX_NUM_FREQ_COEFF 4
#define MMUX_NUM_TEMP_COEFF 3
#define MMUX_NUM_COMP_COEFF 2

typedef struct mmux_config_t
{
    // GPIO Shift Register
    gpio_num_t data;  // shift register data input
    gpio_num_t srclk; // shift register data clock
    gpio_num_t rclk;  // shift register output clock

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
};

typedef struct mmux
{
    gpio_num_t data;  // shift register data input
    gpio_num_t srclk; // shift register data clock
    gpio_num_t rclk;  // shift register output clock

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
    uint8_t num_pcbcs;


    double (*freq_coeffs)[MMUX_NUM_FREQ_COEFF];              // [5]*x^5  [4]*x^4  [3]*x^3  [2]*x^2  [1]*x^1  [0]*x^0
    bool *use_digits;                                         // tells the code to switch from frequency to digits before using the formula
   
    double (*temp_coeffs)[MMUX_NUM_TEMP_COEFF]; // R0/Ohm; T0/°C; Beta OR C1 C2 C3 (Steinhart-Hart))
    bool *use_steinhart;
  
    double (*comp_coeffs)[MMUX_NUM_COMP_COEFF];
};

void mmux_init(mmux_config_t &config, mmux &device);
esp_err_t mmux_begin(mmux &device);
void mmux_version(char buf[6]);
void mmux_end(mmux &device);
void set_freq_coeffs(mmux &device, uint16_t channel, double _freq_coeffs[MMUX_NUM_FREQ_COEFF]);
