#include <cJSON.h>
#include "esp_system.h"
#include "esp_state.h"

void update_esp_state(esp_state_t* esp_state) {
    esp_state->free_heap_size = esp_get_free_heap_size();
}

char * serialize_esp_state(esp_state_t* esp_state) {
    char *json = NULL;
    cJSON *root = cJSON_CreateObject();

    if (esp_state->free_heap_size) {
        cJSON_AddNumberToObject(root, "free_heap_size", esp_state->free_heap_size);
    }

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}
