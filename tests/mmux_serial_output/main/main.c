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
#include "mmux.h"

#include <math.h>

#define TAG "MMUX example"
#define NUM_SENSORS 39

void app_main(void)
{
    mmux mux;
    mmux_config_t mux_cfg = DEFAULT_CONFIG_PCB_V_1_1(NUM_SENSORS);
    mmux_init(&mux_cfg, &mux);
    mmux_begin(&mux);
    mmux_log_info_freq_coeffs(&mux);
    double freq_raw_vals[NUM_SENSORS];
    double temp_raw_vals[NUM_SENSORS];
    mmux_measure(&mux, freq_raw_vals, temp_raw_vals);
    mmux_scaleTemperatures(&mux, temp_raw_vals, temp_raw_vals);
    mmux_scaleFrequencies(&mux, freq_raw_vals, freq_raw_vals);

    for (int i = 0; i < NUM_SENSORS; i++)
        ESP_LOGI("MMUX Example", "channel %d: %lf, %lf", i, freq_raw_vals[i], temp_raw_vals[i]);

    char buf[32];
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sprintf(buf, "%d\t%.2lf\t%.1lf\r\n", i + 1, freq_raw_vals[i], temp_raw_vals[i]);
        mmux_write_bytes_ext(&mux, buf, strlen(buf));
    }
    vTaskDelay(portMAX_DELAY);
    /*
        const double poly_coeffs[4] = {
            2.0140458123e-09, -5.6733713088e-05, -4.0064608315e+00, 3.8168198146e+04};

        double raw_val = 8773.00;
        double val = 0;
        for (int i = 0; i < 4; i++)
        {
            val += (poly_coeffs[3 - i] * pow(raw_val, i));
        }
        ESP_LOGI("POLY", "%.2f", val);*/
}