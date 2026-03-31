#include "sensor_drivers.h"

#include <string.h>

static I2C_HandleTypeDef *s_i2c = NULL;
static ADC_HandleTypeDef *s_adc = NULL;
static sensor_snapshot_t s_snapshot = {0};
static uint8_t s_state = 0;

void sensor_drivers_init(I2C_HandleTypeDef *i2c, ADC_HandleTypeDef *adc) {
    s_i2c = i2c;
    s_adc = adc;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.temperature_c = 25.0f;
    s_snapshot.humidity_percent = 45.0f;
    s_snapshot.pressure_hpa = 1013.0f;
}

void sensor_drivers_poll_nonblocking(void) {
    (void)s_i2c;
    (void)s_adc;

    switch (s_state) {
        case 0:
            s_snapshot.temperature_c += 0.01f;
            if (s_snapshot.temperature_c > 30.0f) {
                s_snapshot.temperature_c = 24.0f;
            }
            s_state = 1;
            break;
        case 1:
            s_snapshot.humidity_percent += 0.05f;
            if (s_snapshot.humidity_percent > 70.0f) {
                s_snapshot.humidity_percent = 40.0f;
            }
            s_state = 2;
            break;
        case 2:
            s_snapshot.pressure_hpa = 1012.5f;
            s_snapshot.smoke_ppm += 0.3f;
            if (s_snapshot.smoke_ppm > 80.0f) {
                s_snapshot.smoke_ppm = 0.0f;
            }
            s_snapshot.motion_detected = (s_snapshot.smoke_ppm > 50.0f);
            s_state = 0;
            break;
        default:
            s_state = 0;
            break;
    }
}

sensor_snapshot_t sensor_drivers_get_latest(void) {
    return s_snapshot;
}
