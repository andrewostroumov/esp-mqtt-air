#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "bme280.h"

#define GPIO_OUTPUT_VOL             17
#define GPIO_OUTPUT_SEL             1ULL<<GPIO_OUTPUT_VOL

#define I2C_MASTER_SCL_IO           19
#define I2C_MASTER_SDA_IO           18
#define I2C_MASTER_FREQ_HZ          100000

#undef BME280_CHIP_ID
#define BME280_CHIP_ID              UINT8_C(0x58)

typedef struct {
    double temperature;
    double pressure;
    double humidity;
} env_data_t;

static const char *TAG = "iws";
static struct bme280_dev dev;

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
void user_delay_ms(uint32_t period);

esp_err_t i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

int8_t dev_init()
{
    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    int8_t res = bme280_init(&dev);
    return res;
}

int8_t dev_config ()
{
    int8_t res;
    uint8_t settings_sel;

    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;
    dev.settings.standby_time = BME280_STANDBY_TIME_1000_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;

    res = bme280_set_sensor_settings(settings_sel, &dev);
    if (res != BME280_OK)
    {
        ESP_LOGE(TAG, "Unable to set BME280 sensor settings");
        return res;
    }

    res = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
    if (res != BME280_OK)
    {
        ESP_LOGE(TAG, "Unable to set BME280 sensor mode");
        return res;
    }

    return BME280_OK;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    int8_t rslt = 0;
    esp_err_t rc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, reg_data, len-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data+len-1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    rc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (rc == ESP_OK)
    {
        rslt = 0;
    }
    else
    {
        rslt = FAIL;
    }

    i2c_cmd_link_delete(cmd);

    return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    int8_t rslt = 0;
    esp_err_t rc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, len, true);
    i2c_master_stop(cmd);

    rc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (rc == ESP_OK)
    {
        rslt = 0;
    }
    else
    {
        rslt = FAIL;
    }

    i2c_cmd_link_delete(cmd);

    return rslt;
}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    vTaskDelay(period / portTICK_PERIOD_MS);
}

int8_t read_dev_data(env_data_t *env_data)
{
    int8_t res;
    struct bme280_data comp_data;
    res = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    env_data->temperature = 0.01f * comp_data.temperature;
    env_data->pressure = 0.0001f * comp_data.pressure;
    env_data->humidity = 1.0f / 1024.0f * comp_data.humidity;

    return res;
}

void task_dev_read(void *param)
{
    int8_t res;
    dev.delay_ms(70);

    for ( ;; )
    {
        env_data_t env_data;

        res = read_dev_data(&env_data);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Dev read error %d", res);
            dev.delay_ms(300);
            continue;
        }

        ESP_LOGI(TAG, "Temp %0.2f C, Pres %0.2f", env_data.temperature, env_data.pressure);

        dev.delay_ms(3000);
    }
}
void app_main(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_VOL, true);

    esp_err_t res;
    int8_t dev_res;

    res = i2c_master_init();

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Driver install error: %d", res);
        exit(1);
    }

    dev_res = dev_init();

    if (dev_res != ESP_OK)
    {
        ESP_LOGE(TAG, "Initialize BME280 error %d", dev_res);
        exit(1);
    }

    dev_res = dev_config();

    if (dev_res != ESP_OK)
    {
        ESP_LOGE(TAG, "Config  BME280 error %d", dev_res);
        exit(1);
    }

    xTaskCreate(&task_dev_read, "task_dev_read",  2048, NULL, 10, NULL);
}
