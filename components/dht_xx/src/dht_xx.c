#include "dht_xx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_check.h"

static const char *TAG = "DHT22";
static int Dht_xx_wait_for_level(gpio_num_t pin, int level, int timeout);

void Dht_xx_init(void)
{
    gpio_set_direction(DHT_xx_DATA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_xx_DATA_PIN, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait for sensor to stabilize
}

int Dht_11_read(int *temperature, int *humidity)
{
    uint8_t data[5] = {0};
    int checksum;

    // Send start signal
    gpio_set_direction(DHT_xx_DATA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_xx_DATA_PIN, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);  // 20ms low
    gpio_set_level(DHT_xx_DATA_PIN, 1);
    esp_rom_delay_us(40);  // 40us high
    gpio_set_direction(DHT_xx_DATA_PIN, GPIO_MODE_INPUT);

    // Wait for DHT11 response
    if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 0, 80) < 0) return -1;
    if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 1, 80) < 0) return -1;
    if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 0, 80) < 0) return -1;

    // Read data
    for (int i = 0; i < 40; i++) {
        if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 1, 50) < 0) return -1;
        esp_rom_delay_us(28);
        data[i / 8] <<= 1;
        if (gpio_get_level(DHT_xx_DATA_PIN)) {
            data[i / 8] |= 1;
        }
        if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 0, 50) < 0) return -1;
    }

    // Checksum validation
    checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        ESP_LOGE(TAG, "Checksum error");
        return -1;
    }

    *humidity = data[0];
    *temperature = data[2];

    return 0;
}

int Dht_22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};
    int checksum;

    // Send start signal
    gpio_set_direction(DHT_xx_DATA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_xx_DATA_PIN, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);  // 20ms low
    gpio_set_level(DHT_xx_DATA_PIN, 1);
    gpio_set_direction(DHT_xx_DATA_PIN, GPIO_MODE_INPUT);
    esp_rom_delay_us(40);  // 40us high
    
    // Wait for DHT22 response
    if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 0, 80) < 0) {
        ESP_LOGE(TAG, "Timeout waiting for response from DHT22 (low)");
        return -1;
    }
    if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 1, 80) < 0) {
        ESP_LOGE(TAG, "Timeout waiting for response from DHT22 (high)");
        return -1;
    }
    if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 0, 80) < 0) {
        ESP_LOGE(TAG, "Timeout waiting for response from DHT22 (low)");
        return -1;
    }

    // Read data
    for (int i = 0; i < 40; i++) {
        if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 1, 50) < 0) {
            ESP_LOGE(TAG, "Timeout waiting for data bit %d (high)", i);
            return -1;
        }
        esp_rom_delay_us(28);
        data[i / 8] <<= 1;
        if (gpio_get_level(DHT_xx_DATA_PIN)) {
            data[i / 8] |= 1;
        }
        if (Dht_xx_wait_for_level(DHT_xx_DATA_PIN, 0, 50) < 0) {
            ESP_LOGE(TAG, "Timeout waiting for data bit %d (low)", i);
            return -1;
        }
    }

    // Checksum validation
    checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        ESP_LOGE(TAG, "Checksum error: expected %d, got %d", checksum, data[4]);
        return -1;
    }

    *humidity = ((data[0] << 8) + data[1]) * 0.1;
    *temperature = (((data[2] & 0x7F) << 8) + data[3]) * 0.1;
    if (data[2] & 0x80) {
        *temperature = -*temperature;
    }

    return 0;
}

static int Dht_xx_wait_for_level(gpio_num_t pin, int level, int timeout) 
{
    int us = 0;
    while (gpio_get_level(pin) != level) {
        if (us++ > timeout) {
            return -1;
        }
        esp_rom_delay_us(1);
    }
    return us;
}