typedef struct {
    uint32_t free_heap_size;
} esp_state_t;

void update_esp_state(esp_state_t* esp_state);
char* serialize_esp_state(esp_state_t* esp_state);