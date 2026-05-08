#include <Arduino.h>
#include <esp_system.h>
void setup() {
    esp_reset_reason_t reason = esp_reset_reason();
    if(reason == ESP_RST_PANIC) {
        // Crash happened
    }
}
void loop() {}
