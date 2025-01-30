#include "mmux_priv.h"

void mmux_init(mmux_config_t *config, mmux *device)
{
    device->shift_reg_data = config->shift_reg_data;
    device->shift_reg_srclk = config->shift_reg_srclk;
    device->shift_reg_rclk = config->shift_reg_rclk;
    device->spi_host = config->spi_host;

    device->rx_box = config->rx_box;
    device->tx_box = config->tx_box;
    device->de_box = config->de_box;
    device->box_uart = config->box_uart;
    device->box_serial_cfg = config->box_serial_cfg;
    device->box_addr = config->box_addr;
    device->measurement_timeout = config->measurement_timeout; // seconds; ca. 3 seconds per active (box) channel

    device->rx_ext = config->rx_ext;
    device->tx_ext = config->tx_ext;
    device->de_ext = config->de_ext;
    device->ext_uart = config->ext_uart;
    device->ext_serial_cfg = config->ext_serial_cfg;

    device->num_max_meas = config->num_max_meas;
    device->num_close_vals = config->num_close_vals;
    device->delta_close_vals = config->delta_close_vals;
    device->num_sensors = config->num_sensors;
}

esp_err_t mmux_begin(mmux *device)
{
    ESP_LOGI(TAG, "Version: %s", LIB_VERSION);

    // calculate number of pcbs used
    device->num_pcbs = device->num_sensors / NUM_CHANNELS;
    if (device->num_pcbs * NUM_CHANNELS < device->num_sensors) // this happens due to integer division
        device->num_pcbs++;

    // setup rclk pin
    gpio_config_t out_cfg =
        {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = 1ULL << device->shift_reg_rclk,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
    gpio_config(&out_cfg);
    gpio_set_level(device->shift_reg_rclk, 0);

    // Setup SPI
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = device->shift_reg_data,
        .sclk_io_num = device->shift_reg_srclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000 * 1000,
        .mode = 0,          // SPI mode 0
        .spics_io_num = -1, // CS pin unused
        .queue_size = 1,
    };

    ret = spi_bus_initialize(device->spi_host, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK;
    if (ret)
    {
        ESP_LOGE(TAG, "SPI bus init fail");
        return ret;
    }

    ret = spi_bus_add_device(device->spi_host, &devcfg, &device->shift_reg) != ESP_OK;
    if (ret)
    {
        ESP_LOGE(TAG, "SPI add device fail");
        return ret;
    }

    // setup UART
    uart_driver_install(device->box_uart, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0);
    uart_param_config(device->box_uart, &device->box_serial_cfg);
    uart_set_pin(device->box_uart, device->tx_box, device->rx_box, device->de_box, -1);
    uart_set_mode(device->box_uart, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_rx_timeout(device->box_uart, UART_TIMEOUT);

    uart_driver_install(device->ext_uart, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0);
    uart_param_config(device->ext_uart, &device->ext_serial_cfg);
    uart_set_pin(device->ext_uart, device->tx_ext, device->rx_ext, device->de_ext, -1);
    uart_set_mode(device->ext_uart, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_rx_timeout(device->ext_uart, UART_TIMEOUT);

    // Malloc Memory for Scaling Factors
    // maybe put this in PSRAM when many sensors are to be used
    device->temp_coeffs = (double(*)[MMUX_NUM_TEMP_COEFF])malloc(device->num_sensors * MMUX_NUM_TEMP_COEFF * __SIZEOF_DOUBLE__);
    if (device->temp_coeffs == NULL)
    {
        ESP_LOGE(TAG, "temp cal malloc failed");
        return ESP_ERR_NO_MEM;
    }
    device->use_steinhart = (bool *)malloc(device->num_sensors * MMUX_NUM_TEMP_COEFF);
    if (device->use_steinhart == NULL)
    {
        ESP_LOGE(TAG, "use st malloc failed");
        return ESP_ERR_NO_MEM;
    }
    device->freq_coeffs = (double(*)[MMUX_NUM_FREQ_COEFF])malloc(device->num_sensors * MMUX_NUM_FREQ_COEFF * __SIZEOF_DOUBLE__);
    if (device->freq_coeffs == NULL)
    {
        ESP_LOGE(TAG, "freq cal malloc failed");
        return ESP_ERR_NO_MEM;
    }
    device->use_digits = (bool *)malloc(device->num_sensors * MMUX_NUM_FREQ_COEFF);
    if (device->use_digits == NULL)
    {
        ESP_LOGE(TAG, "use dig malloc failed");
        return ESP_ERR_NO_MEM;
    }
    device->comp_coeffs = (double(*)[MMUX_NUM_COMP_COEFF])malloc(device->num_sensors * MMUX_NUM_COMP_COEFF * __SIZEOF_DOUBLE__);
    if (device->comp_coeffs == NULL)
    {
        ESP_LOGE(TAG, "comp coeff malloc failed");
        return ESP_ERR_NO_MEM;
    }
    // set calib coeffs to default
    for (int i = 0; i < device->num_sensors; i++)
    {
        mmux_set_temp_coeffs(device, i, DEFAULT_TEMP_COEFFS, DEFAULT_USE_STEINHART);
        mmux_set_freq_coeffs(device, i, DEFAULT_FREQ_COEFFS, DEFAULT_USE_DIGITS);
        mmux_set_comp_coeffs(device, i, DEFAULT_COMP_COEFFS[0], DEFAULT_COMP_COEFFS[1]);
    }
    /*
    uint8_t channel = 0;
    for (int i = 0; i < device->num_pcbs; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            set_mux(device, channel);
            channel++;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }*/
    set_mux(device, -1);

    return ESP_OK;
}

void mmux_end(mmux *device)
{
    free(device->temp_coeffs);
    free(device->freq_coeffs);
    free(device->use_digits);
    free(device->use_steinhart);

    spi_bus_remove_device(device->shift_reg);
    spi_bus_free(device->spi_host);
}

void mmux_version(char buf[9])
{
    strcpy(buf, LIB_VERSION);
}

void mmux_set_freq_coeffs(mmux *device, const uint16_t channel, const double _freq_coeffs[MMUX_NUM_FREQ_COEFF], const bool _use_digits)
{
    for (int i = 0; i < MMUX_NUM_FREQ_COEFF; i++)
        device->freq_coeffs[channel][i] = _freq_coeffs[i];

    device->use_digits[channel] = _use_digits;
    return;
}

void mmux_set_temp_coeffs(mmux *device, const uint16_t channel, const double _temp_coeffs[MMUX_NUM_TEMP_COEFF], const bool _use_steinhart)
{
    for (int i = 0; i < MMUX_NUM_TEMP_COEFF; i++)
        device->temp_coeffs[channel][i] = _temp_coeffs[i];
    device->use_steinhart[channel] = _use_steinhart;
    return;
}

void mmux_set_comp_coeffs(mmux *device, const uint16_t channel, const double t0, const double alpha)
{
    device->comp_coeffs[channel][0] = t0;
    device->comp_coeffs[channel][1] = alpha;
}

void mmux_log_info_freq_coeffs(mmux *device)
{

#if MMUX_NUM_FREQ_COEFF == 4
    ESP_LOGI(TAG, "Active Coeffs:");
    for (int i = 0; i < device->num_sensors; i++)
        printf("%.4f + %.4f x + %.4f x² + %.4f x³\r\n", device->freq_coeffs[i][0], device->freq_coeffs[i][1], device->freq_coeffs[i][2], device->freq_coeffs[i][3]);
#else
    ESP_LOGI(TAG, "Coefficient display not implemented");
#endif
}

/****** private Functions ******/
void set_mux(mmux *device, int16_t channel)
{
    uint8_t *msg = malloc(device->num_pcbs * sizeof(uint8_t));
    assert(msg != NULL);

    for (int i = 0; i < device->num_pcbs; i++)
        msg[i] = 0;

    if (channel < device->num_pcbs * 8)
    {
        // first choose which pcb (which byte) contains the row that must be activated
        uint8_t active_pcb = (device->num_pcbs - 1) - (channel / 8);
        msg[active_pcb] = 128;
        // the shift inside said byte
        msg[active_pcb] = msg[active_pcb] >> (channel % 8);
    }
    else
        ESP_LOGE(TAG, "channel too big for number of PCBs/Sensors");

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));        // Zero out the transaction
    t.length = 8 * device->num_pcbs; // Command is 8 bits
    t.tx_buffer = msg;               // The data is the cmd itself

    spi_device_transmit(device->shift_reg, &t);

    // clock out data
    free(msg);
    ets_delay_us(5);
    gpio_set_level(device->shift_reg_rclk, 1);
    ets_delay_us(5);
    gpio_set_level(device->shift_reg_rclk, 0);
    ets_delay_us(5);
}

void parse_box_string(char *box_string, double *vals)
{
    char val_buf[8];
    memset(val_buf, 0, sizeof(val_buf));
    uint8_t val_buf_idx = 0;
    for (int i = 2;; i++)
    {
        val_buf[val_buf_idx] = box_string[i];
        val_buf_idx++;
        if (box_string[i] == '+' || box_string[i] == '-' || box_string[i] < 31)
        {
            val_buf[val_buf_idx - 1] = '\0';
            *vals = atof(val_buf);
            vals++;
            memset(val_buf, 0, sizeof(val_buf));
            val_buf_idx = 0;
            if (box_string[i] < 31)
                break;
        }
    }
}

esp_err_t read_dataset(mmux *device, uint8_t dataset, double *raw_vals)
{
    if (dataset > 3)
    {
        ESP_LOGE(TAG, "invalid block requested");
        return ESP_ERR_INVALID_ARG;
    }
    char buf[32];
    uart_write_bytes(device->box_uart, COMMAND_DATA(device->box_addr, dataset), sizeof(COMMAND_DATA(device->box_addr, dataset)));

    memset(buf, 0, sizeof(buf));
    int n = 1;
    for (int i = 0; i < sizeof(buf) && n > 0; i++)
    {
        n = uart_read_bytes(device->box_uart, &buf[i], 1, 2000 / portTICK_PERIOD_MS);
        if (buf[i] == '\r')
        {
            buf[i] = '\0';
            break;
        }
    }
    uart_flush_input(device->box_uart);
    parse_box_string(buf, raw_vals);
    return ESP_OK;
}

int compd(const void *a, const void *b)
{
    if (*(double *)a > *(double *)b)
        return 1;
    else if (*(double *)a > *(double *)b)
        return -1;
    else
        return 0;
}

double find_mean(mmux *device, double *vals, size_t n)
{
    qsort(vals, n, sizeof(vals[0]), compd);
    /*ESP_LOGI(TAG, "sorted:");
    for (int i = 0; i < n; i++)
        ESP_LOGI(TAG, "%lf", vals[i]);
    printf("\n\n");*/

    double mean = 0;
    for (int i = 0; i + device->num_close_vals <= n; i++)
    {
        if (fabs(vals[i] - vals[i + device->num_close_vals - 1]) < device->delta_close_vals)
        {
            for (int j = 0; j < device->num_close_vals; j++)
                mean += vals[j];
            mean /= device->num_close_vals;
            break;
        }
        else
            mean = NAN;
    }
    return mean;
}

esp_err_t box_measure(mmux *device)
{
    uart_write_bytes(device->box_uart, COMMAND_MEASURE(device->box_addr), sizeof(COMMAND_MEASURE(device->box_addr)));
    char buf[32];
    memset(buf, 0, sizeof(buf));
    int num = uart_read_bytes(device->box_uart, buf, 7, 1000 / portTICK_PERIOD_MS);
    if (num == 0) //(device->measurement_timeout * 1000)
    {
        ESP_LOGE(TAG, "Box Pre-Timeout; no box connected?");
        return ESP_ERR_TIMEOUT;
    }
    memset(buf, 0, sizeof(buf));
    num = uart_read_bytes(device->box_uart, buf, 3, device->measurement_timeout * 1000 / portTICK_PERIOD_MS);
    if (num == 0)
    {
        ESP_LOGE(TAG, "Box Measurement Timeout");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t mmux_measure(mmux *device, double *freq_raw_vals, double *temp_raw_vals)
{
    ESP_LOGI(TAG, "VibWire Measurement");
    esp_err_t ret = ESP_OK; // esp error return val

    // clear uart buf
    uart_flush_input(device->box_uart);

    // get memory to buffer sensor vals
    double(*freq_val_buf)[8] = malloc(8 * device->num_max_meas * sizeof(double));
    double(*temp_val_buf)[8] = malloc(8 * device->num_max_meas * sizeof(double));
    if (freq_val_buf == NULL || temp_val_buf == NULL)
    {
        ESP_LOGE(TAG, "malloc fail for measurement buf");
        ret = ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < device->num_sensors; i++)
        freq_raw_vals[i] = NAN;

    // go through all sensors
    for (int sensor_num = 0; sensor_num < device->num_sensors && ret == ESP_OK; sensor_num += 8)
    {
        set_mux(device, sensor_num / NUM_CHANNELS_BOX);
        bool all_valid = false;
        uint8_t num_tries[NUM_CHANNELS_BOX];
        memset(num_tries, device->num_max_meas, sizeof(num_tries));
        for (int retry_num = 0; retry_num < device->num_max_meas && !all_valid && ret == ESP_OK; retry_num++)
        {
            ret |= box_measure(device);
            if (ret == ESP_OK)
            {
                ret |= read_dataset(device, BLUEBOX_FREQ_0_to_3, &freq_val_buf[retry_num][0]);
                ret |= read_dataset(device, BLUEBOX_FREQ_4_to_7, &freq_val_buf[retry_num][4]);
                ret |= read_dataset(device, BLUEBOX_TEMP_0_TO_3, &temp_val_buf[retry_num][0]);
                ret |= read_dataset(device, BLUEBOX_TEMP_4_TO_7, &temp_val_buf[retry_num][4]);

                // if there is a chance that the specified number of close vals was reached, try to determine it
                if (retry_num + 1 >= device->num_close_vals)
                {
                    double *tmp = malloc((retry_num + 1) * sizeof(double));
                    all_valid = true; // assume all sensors are fine (will be revoked if false later on)

                    // i<8 should be effective for all rows except the last one which might be not fully populated
                    for (int channel_idx = 0; channel_idx < 8 && (channel_idx + sensor_num) < device->num_sensors; channel_idx++)
                    {
                        for (int i = 0; i <= retry_num; i++)
                        {
                            tmp[i] = freq_val_buf[i][channel_idx];
                            // for temperatures: use all values we can get because they are always fine and mean decreses noise
                            temp_raw_vals[channel_idx + sensor_num] += temp_val_buf[i][channel_idx];
                        }
                        temp_raw_vals[channel_idx + sensor_num] /= (retry_num + 1);

                        if (isnan(freq_raw_vals[channel_idx + sensor_num])) // if value was alreday found leave it alone
                        {
                            freq_raw_vals[channel_idx + sensor_num] = find_mean(device, tmp, retry_num + 1);
                            if (isnan(freq_raw_vals[channel_idx + sensor_num])) // check again to see if all values are fine now
                                all_valid = false;
                            else
                                num_tries[channel_idx] = retry_num + 1;
                        }
                    }
                    free(tmp);
                }
            }
        }
        if (ret == ESP_OK)
        {
            for (int channel_idx = 0; channel_idx < 8 && (channel_idx + sensor_num) < device->num_sensors; channel_idx++)
            {
                ESP_LOGI(TAG, "CH %d: %.2lf after %d measurements", sensor_num + channel_idx + 1, freq_raw_vals[channel_idx + sensor_num], num_tries[channel_idx]);
                num_tries[channel_idx] = device->num_max_meas;
            }
        }
    }
    set_mux(device, -1);
    free(freq_val_buf);
    free(temp_val_buf);
    return ret;
}

void mmux_scaleFrequencies(mmux *device, double *freq_raw_vals, double *freq_vals)
{
    for (int i = 0; i < device->num_sensors; i++)
    {
        if (device->use_digits[i])
            freq_raw_vals[i] = (freq_raw_vals[i] * freq_raw_vals[i]) / 1000;
        double tmp = 0;
        for (int j = 0; j < MMUX_NUM_FREQ_COEFF; j++)
        {
            tmp += device->freq_coeffs[i][j] * pow(freq_raw_vals[i], j);
        }
        freq_vals[i] = tmp;
    }
}

void mmux_scaleTemperatures(mmux *device, double *temp_raw_vals, double *temp_vals)
{
    for (int i = 0; i < device->num_sensors; i++)
    {
        if (device->use_steinhart[i])
        {
            // steinhart formula
            // first get rtc resistance;
            // 3300 = Parallel resistance inside keynes box;
            // 2,4V is the "open" value (R_ntc infinite)
            // thus, the current output of the current source connected to the NTC and parallel Resistors is I_0 = 2,4V/3300 Ohm;
            // -273.15 is the offset from kelvin to °C
            // frac{1}{T} = A + B \ln(R) + C (\ln(R))^3
            double R_rtc = (temp_raw_vals[i] * 3300) / (2400 - temp_raw_vals[i]);
            double C_1 = device->temp_coeffs[i][0];
            double C_2 = device->temp_coeffs[i][1];
            double C_3 = device->temp_coeffs[i][2];
            temp_vals[i] = 1 / (C_1 + C_2 * log(R_rtc) + C_3 * pow(log(R_rtc), 3)) - 273.15;
        }
        else
        {
            // r_ratio = R_rtc/R_0; R_0 = 3300 (inside keynes Box)
            double r_ratio = (temp_raw_vals[i] / (2400 - temp_raw_vals[i]));
            // double R_t0 = device->temp_coeffs[i][0]; // idk
            double t_0 = device->temp_coeffs[i][1] + 273.15;
            double beta = device->temp_coeffs[i][2];
            temp_vals[i] = (beta * t_0) / (beta + t_0 * (log(r_ratio))) - 273.15;
        }
    }
}

void mmux_compensateTemperature(mmux *device, double *freq_vals, double *temp_vals, double *freq_vals_comp)
{
    for (int i = 0; i < device->num_sensors; i++)
    {
        if (device->comp_coeffs[i][0] != 0)
        {
            if (temp_vals[i] > 80 || temp_vals[i] < -30)
                ESP_LOGE(TAG, "invalid temperature, no compoensation");
            else
            {
                double t0 = device->comp_coeffs[i][0];
                double alpha = device->comp_coeffs[i][1];
                double tmp = freq_vals[i] + (temp_vals[i] - t0) * alpha;
                freq_vals_comp[i] = tmp;
            }
        }
    }
}

int mmux_write_bytes_ext(mmux *device, const void *src, size_t size)
{
    return uart_write_bytes(device->ext_uart, src, size);
}

int mmux_read_bytes_ext(mmux *device, void *buf, uint32_t length, TickType_t ticks_to_wait)
{
    return uart_read_bytes(device->ext_uart, buf, length, ticks_to_wait);
}
