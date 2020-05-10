#include <cJSON.h>
#include "math.h"
#include "esp_air.h"

void build_air_quality_index(esp_air_t *esp_air) {
    enum AIR_QUALITY_INDEX index = AIR_QUALITY_NULL;
    double score = esp_air->iaq;

    if (esp_air->iaq) {
        if      (score >= 301)                  index = AIR_QUALITY_HAZARDOUS;
        else if (score >= 201 && score <= 300 ) index = AIR_QUALITY_VERY_UNHEALTHY;
        else if (score >= 176 && score <= 200 ) index = AIR_QUALITY_UNHEALTHY;
        else if (score >= 151 && score <= 175 ) index = AIR_QUALITY_UNHEALTHY_FOR_SENSITIVE_GROUPS;
        else if (score >=  51 && score <= 150 ) index = AIR_QUALITY_MODERATE;
        else if (score >=  00 && score <=  50 ) index = AIR_QUALITY_GOOD;
    }

    esp_air->iaq_index = index;
}

char* serialize_esp_air(esp_air_t *esp_air) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();

    if (esp_air->temperature) {
        cJSON_AddNumberToObject(root, "temperature", floor(esp_air->temperature * 100) / 100);
    }

    if (esp_air->humidity) {
        cJSON_AddNumberToObject(root, "humidity", floor(esp_air->humidity * 100) / 100);
    }

    if (esp_air->pressure) {
        cJSON_AddNumberToObject(root, "pressure", floor(esp_air->pressure * 100) / 100);
    }

    if (esp_air->gas) {
        cJSON_AddNumberToObject(root, "gas", floor(esp_air->gas * 100) / 100);
    }

    cJSON_AddNumberToObject(root, "iaq_accuracy", esp_air->iaq_accuracy);

    if (esp_air->iaq) {
        cJSON_AddNumberToObject(root, "iaq", floor(esp_air->iaq * 100) / 100);
    }

    if (esp_air->iaq_index) {
        cJSON_AddNumberToObject(root, "iaq_index", esp_air->iaq_index);
    }

    if (esp_air->co2_equivalent) {
        cJSON_AddNumberToObject(root, "co2_equivalent", floor(esp_air->co2_equivalent * 100) / 100);
    }

    if (esp_air->breath_voc_equivalent) {
        cJSON_AddNumberToObject(root, "breath_voc_equivalent", floor(esp_air->breath_voc_equivalent * 100) / 100);
    }

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

void build_co2_index(esp_air_co2_t *esp_air) {
    enum CO2_INDEX index = CO2_NULL;
    double score = esp_air->co2;

    if (score) {
        if      (score >= 1400)                 index = CO2_HAZARDOUS;
        else if (score >= 1000 && score < 1400) index = CO2_UNHEALTHY;
        else if (score >= 800 && score < 1000)  index = CO2_MODERATE;
        else if (score < 800)                   index = CO2_GOOD;
    }

    esp_air->co2_index = index;
}

char* serialize_esp_air_co2(esp_air_co2_t *esp_air) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();

    if (esp_air->co2) {
        cJSON_AddNumberToObject(root, "co2", floor(esp_air->co2 * 100) / 100);
    }

    if (esp_air->co2_index) {
        cJSON_AddNumberToObject(root, "co2_index", esp_air->co2_index);
    }

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}
