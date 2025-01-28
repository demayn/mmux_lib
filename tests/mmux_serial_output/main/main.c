#include <stdio.h>
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rom/ets_sys.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

void app_main(void)
{
    ESP_LOGI("MMUX", "test 123 ¹⁰³⁶⁹⁷ⁿ⁸⁵⁵⁴");
    gpio_config_t out_cfg =
        {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = 1ULL << GPIO_NUM_14,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
    gpio_config(&out_cfg);
    gpio_set_level(14, 0);
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = 13,
        .sclk_io_num = 21,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 100 * 1000,
        .mode = 0,          // SPI mode 0
        .spics_io_num = -1, // CS pin
        .queue_size = 1,
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    spi_transaction_t t;
    uint8_t val = 2;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 8;             // Command is 8 bits
    t.tx_buffer = &val;       // The data is the cmd itself

    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // clock out data
    gpio_set_level(14, 1);
    ets_delay_us(5);
    gpio_set_level(14, 0);
    ets_delay_us(5);
}