#ifndef DHT11_H
#define DHT11_H

#include "driver/gpio.h"

#define DHT_xx_DATA_PIN GPIO_NUM_0

/**
 * @brief Initialize the DHT11 sensor.
 *
 * This function initializes the DHT11 sensor by setting the specified GPIO pin
 * to output mode and setting it high. It also waits for the sensor to stabilize.
 */
void Dht_xx_init(void);

/**
 * @brief Read data from the DHT11 sensor.
 *
 * This function reads the temperature and humidity data from the DHT11 sensor.
 * It sends a start signal to the sensor, waits for the sensor's response, and
 * then reads the data. The function returns the temperature and humidity values
 * through the provided pointers.
 *
 * @param temperature Pointer to store the read temperature value
 * @param humidity Pointer to store the read humidity value.
 * @return 0 on success, -1 on failure.
 */
int Dht_11_read(int *temperature, int *humidity);

/**
 * @brief Read data from the DHT22 sensor.
 *
 * This function reads the temperature and humidity data from the DHT22 sensor.
 * It sends a start signal to the sensor, waits for the sensor's response, and
 * then reads the data. The function returns the temperature and humidity values
 * through the provided pointers.
 *
 * @param temperature Pointer to store the read temperature value.
 * @param humidity Pointer to store the read humidity value.
 * @return 0 on success, -1 on failure.
 */
int Dht_22_read(float *temperature, float *humidity);

#endif // DHT11_H