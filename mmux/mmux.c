#include "mmux_priv.h"

void mmux_init(mmux_config_t *config, mmux *device);

esp_err_t mmux_begin(mmux *device)
{
    ESP_LOGI(TAG, "Version: %s", LIB_VERSION);

    return ESP_OK;
}

void end(mmux *device)
{
    free(device->temp_coeffs);
    free(device->freq_coeffs);
    free(device->use_digits);
    free(device->use_steinhart);
}