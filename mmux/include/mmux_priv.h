#include "mmux.h"

#ifndef LIB_VERSION
#define LIB_VERSION "x.x.x"
#endif

#define COMMAND_MEASURE(address) {'%', address, 'M', '!', '\0'}
#define COMMAND_DATA(address, dataset) { '%', address, 'D', (char)(dataset), '!', '\0' }

enum bluebox_datasets
{
    BLUEBOX_FREQ_0_to_3,
    BLUEBOX_FREQ_4_to_7,
    BLUEBOX_TEMP_0_TO_3,
    BLUEBOX_TEMP_4_TO_7,
};

#define DEFAULT_FREQ_COEFFS {0, 1, 0, 0} // [0]*x⁰ + [1]*x¹ + [2]*x² + [3]*x³
#define DEFAULT_USE_DIGITS false // default output is Hertz

#define DEFAULT_TEMP_COEFFS {3000, 25, 3891} // valid for YSI44005 (3K NTC, used by sisgeo)
#define DEFAULT_USE_STEINHART false // default calib is beta Formula
// example for Steinhart: double default_temp_calib[num_temp_coeff] = {1.4051e-3, 2.369e-4, 1.019e-7};

#define DEFAULT_COMP_COEFFS {16, 0}

void mmux_set_mux(mmux &device);
