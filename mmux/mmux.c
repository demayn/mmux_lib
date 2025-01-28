#include "mmux_priv.h"

void end(mmux &device)
{
    free(device->temp_calib_coeffs);
    free(device->freq_calib_coeffs);
    free(device->use_digits);
    free(device->use_steinhart);
}