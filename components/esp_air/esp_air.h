#include "stdint.h"

enum AIR_QUALITY_INDEX {
    AIR_QUALITY_NULL,
    AIR_QUALITY_GOOD,
    AIR_QUALITY_MODERATE,
    AIR_QUALITY_UNHEALTHY_FOR_SENSITIVE_GROUPS,
    AIR_QUALITY_UNHEALTHY,
    AIR_QUALITY_VERY_UNHEALTHY,
    AIR_QUALITY_HAZARDOUS,
};

enum CO2_INDEX {
    CO2_NULL,
    CO2_GOOD,
    CO2_MODERATE,
    CO2_UNHEALTHY,
    CO2_HAZARDOUS,
};

typedef struct {
    double temperature;
    double humidity;
    double pressure;
    uint8_t iaq_accuracy;
    enum AIR_QUALITY_INDEX iaq_index;
    double iaq;
    double gas;
    double co2_equivalent;
    double breath_voc_equivalent;
} esp_air_t;

typedef struct {
    double co;
} esp_air_co_t;

typedef struct {
    enum CO2_INDEX co2_index;
    double co2;
} esp_air_co2_t;

void build_air_quality_index(esp_air_t *esp_air);
char* serialize_esp_air(esp_air_t *esp_air);

char* serialize_esp_air_co(esp_air_co_t *esp_air);

void build_co2_index(esp_air_co2_t *esp_air);
char* serialize_esp_air_co2(esp_air_co2_t *esp_air);
