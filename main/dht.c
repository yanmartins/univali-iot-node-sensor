#include "dht.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <driver/gpio.h>
#include <rom/ets_sys.h> // os_delay_us

#define DHT_TIMER_INTERVAL 2
#define DHT_DATA_BITS 40

#ifdef DEBUG_DHT
#define debug(fmt, ...) printf("%s" fmt "\n", "dht: ", ## __VA_ARGS__);
#else
#define debug(fmt, ...) /* (do nothing) */
#endif

static bool dht_await_pin_state(uint8_t pin, uint32_t timeout,
        bool expected_pin_state, uint32_t *duration)
{
    for (uint32_t i = 0; i < timeout; i += DHT_TIMER_INTERVAL) {
        os_delay_us(DHT_TIMER_INTERVAL);
        if (gpio_get_level(pin) == expected_pin_state) {
            if (duration) {
                *duration = i;
            }
            return true;
        }
    }
    return false;
}

static inline bool dht_fetch_data(dht_sensor_type_t sensor_type, uint8_t pin, bool bits[DHT_DATA_BITS])
{
    uint32_t low_duration;
    uint32_t high_duration;

    gpio_set_level(pin, 0);
    os_delay_us(sensor_type == DHT_TYPE_SI7021 ? 500 : 20000);
    gpio_set_level(pin, 1);

    if (!dht_await_pin_state(pin, 40, false, NULL)) {
        debug("Initialization error, problem in phase 'B'\n");
        return false;
    }

    if (!dht_await_pin_state(pin, 88, true, NULL)) {
        debug("Initialization error, problem in phase 'C'\n");
        return false;
    }

    if (!dht_await_pin_state(pin, 88, false, NULL)) {
        debug("Initialization error, problem in phase 'D'\n");
        return false;
    }

    for (int i = 0; i < DHT_DATA_BITS; i++) {
        if (!dht_await_pin_state(pin, 65, true, &low_duration)) {
            debug("LOW bit timeout\n");
            return false;
        }
        if (!dht_await_pin_state(pin, 75, false, &high_duration)) {
            debug("HIGH bit timeout\n");
            return false;
        }
        bits[i] = high_duration > low_duration;
    }
    return true;
}

static inline float dht_convert_data(dht_sensor_type_t sensor_type, uint8_t msb, uint8_t lsb)
{
    float data;

    if (sensor_type == DHT_TYPE_DHT22) {
        data = ((msb & 0x7F) << 8) | lsb; // Combine MSB and LSB
        data /= 10.0; // Convert to float with decimal precision
        if (msb & 0x80) { // Check if the sign bit is set (for negative values)
            data = -data;
        }
    } else {
        data = msb; // DHT11 data only in integer
    }

    return data;
}

esp_err_t dht_read_data(dht_sensor_type_t sensor_type, gpio_num_t pin, float *humidity, float *temperature)
{
    bool bits[DHT_DATA_BITS];
    uint8_t data[DHT_DATA_BITS / 8] = {0};
    bool result;

    taskENTER_CRITICAL();
    result = dht_fetch_data(sensor_type, pin, bits);
    taskEXIT_CRITICAL();

    if (!result) {
        return ESP_FAIL;
    }

    for (uint8_t i = 0; i < DHT_DATA_BITS; i++) {
        data[i / 8] <<= 1;
        data[i / 8] |= bits[i];
    }

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        debug("Checksum failed, invalid data received from sensor\n");
        return ESP_FAIL;
    }

    if (sensor_type == DHT_TYPE_DHT11) {
        *humidity = data[0] + data[1] * 0.1;
        *temperature = data[2] + data[3] * 0.1;
    } else {
        *humidity = ((data[0] << 8) | data[1]) / 10.0;
        *temperature = ((data[2] << 8) | data[3]) / 10.0;
    }

    debug("Sensor data: humidity=%.1f, temp=%.1f\n", *humidity, *temperature);

    return ESP_OK;
}


esp_err_t dht_init(gpio_num_t pin, bool pull_up) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pin_bit_mask = 1ULL << pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE
    };
    esp_err_t result = gpio_config(&io_conf);
    if (result == ESP_OK) {
        gpio_set_level(pin, 1);
        return ESP_OK;
    }
    return result;
}
