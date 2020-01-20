static double hum_reference = 40;
static double gas_lower_limit = 10000;
static double gas_upper_limit = 300000;

enum AIR_QUALITY_INDEX {
    AIR_QUALITY_NULL,
    AIR_QUALITY_GOOD,
    AIR_QUALITY_MODERATE,
    AIR_QUALITY_UNHEALTHY_FOR_SENSITIVE_GROUPS,
    AIR_QUALITY_UNHEALTHY,
    AIR_QUALITY_VERY_UNHEALTHY,
    AIR_QUALITY_HAZARDOUS,
};

int8_t dev_init()
{
    dev.dev_id = BME680_I2C_ADDR_SECONDARY;
    dev.intf = BME680_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;
    dev.amb_temp = 25;
    return bme680_init(&dev);
}

int8_t dev_config()
{
    int8_t res;
    uint8_t settings_sel;

    dev.tph_sett.os_hum = BME680_OS_2X;
    dev.tph_sett.os_pres = BME680_OS_4X;
    dev.tph_sett.os_temp = BME680_OS_8X;
    dev.tph_sett.filter = BME680_FILTER_SIZE_3;


    dev.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    dev.gas_sett.heatr_temp = 320; /* degree Celsius */
    dev.gas_sett.heatr_dur = 150; /* milliseconds */
    dev.power_mode = BME680_FORCED_MODE;

    settings_sel = BME680_OST_SEL;
    settings_sel |= BME680_OSP_SEL;
    settings_sel |= BME680_OSH_SEL;
    settings_sel |= BME680_FILTER_SEL;
    settings_sel |= BME680_GAS_SENSOR_SEL;

    res = bme680_set_sensor_settings(settings_sel, &dev);
    if (res != BME680_OK)
    {
        ESP_LOGE(TAG, "Unable to set BME680 sensor settings");
        return res;
    }

    res = bme680_set_sensor_mode(&dev);

    if (res != BME680_OK)
    {
        ESP_LOGE(TAG, "Unable to set BME680 sensor mode");
        return res;
    }

    return BME680_OK;
}

int8_t read_air_data(esp_air_t *esp_air)
{
    int8_t res;
    struct bme680_field_data data;
    res = bme680_get_sensor_data(&data, &dev);

    esp_air->temperature = 0.01f * data.temperature;
    esp_air->pressure = 0.01f * data.pressure;
    esp_air->humidity = 0.001f * data.humidity;

    if(data.status & BME680_GASM_VALID_MSK) {
        esp_air->resistance = data.gas_resistance;
    } else {
        esp_air->resistance = 0;
    }

    if (dev.power_mode == BME680_FORCED_MODE) {
        res = bme680_set_sensor_mode(&dev);
    }

    return res;
}

double build_air_hum_score(double current) {
    double hum_score;

    if (current >= 38 && current <= 42) {
        hum_score = 0.25 * 100;
    } else {
        if (current < 38)
            hum_score = 0.25 / hum_reference * current * 100;
        else
        {
            hum_score = ((-0.25 / (100 - hum_reference) * current) + 0.416666) * 100;
        }
    }

    return hum_score;
}


double build_air_gas_score(double current) {
    double gas_reference;
    double gas_score;

    if (current > gas_upper_limit) {
        gas_reference = gas_upper_limit;
    } else if (current < gas_lower_limit) {
        gas_reference = gas_lower_limit;
    } else {
        gas_reference = current;
    }

    gas_score = (0.75 / (gas_upper_limit - gas_lower_limit) * gas_reference - (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) * 100;
    return gas_score;
}

void build_air_quality(esp_air_t *esp_air) {
    double hum_score, gas_score;

    if (!esp_air->resistance) {
        esp_air->quality = 0;
        return;
    }

    hum_score = build_air_hum_score(esp_air->humidity);
    gas_score = build_air_gas_score(esp_air->resistance);

    ESP_LOGI(TAG, "[IAQ] Air Quality: %.2f%% derived from 25%% of Humidity reading and 75%% of Gas reading - 100%% is good quality air", esp_air->quality);
    ESP_LOGI(TAG, "[IAQ] Humidity element was: %.2f of 0.25", hum_score / 100);
    ESP_LOGI(TAG, "[IAQ] Gas element was: %.2f of 0.75", gas_score / 100);

    if (esp_air->resistance < 120000) {
        ESP_LOGI(TAG, "***** Poor air quality *****");
    }

    esp_air->quality = hum_score + gas_score;
}


void task_air_retrieve(void *param)
{
    int8_t res;
    esp_air_t esp_air;

    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &dev);

    while (true)
    {
        dev.delay_ms(meas_period);
        res = read_air_data(&esp_air);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Dev read error %d", res);
            dev.delay_ms(300);
            continue;
        }

        build_air_quality(&esp_air);
        build_air_quality_index(&esp_air);
        xQueueSend(esp_air_queue, &esp_air, portMAX_DELAY);
        xEventGroupSetBits(esp_event_group, CONNECTED_BIT);
        dev.delay_ms(3000);
    }
}
