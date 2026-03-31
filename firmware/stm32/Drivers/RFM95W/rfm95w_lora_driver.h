/**
 * @file rfm95w_lora_driver.h
 * @brief RFM95W LoRa radio driver for STM32F407
 *
 * Provides SPI interface to RFM95W transceiver for long-range telemetry transmission.
 * In simulation mode, generates mock LoRa packets for testing.
 */

#ifndef RFM95W_LORA_DRIVER_H
#define RFM95W_LORA_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/* LoRa Radio Parameters */
#define LORA_FREQ_MHZ 868.0f          /* ISM band: 868 MHz for Europe */
#define LORA_BANDWIDTH_KHZ 125        /* Standard bandwidth */
#define LORA_SPREADING_FACTOR 10      /* SF7-12, balance range vs data rate */
#define LORA_CODING_RATE 1            /* 4/5 to 4/8 */
#define LORA_TX_POWER_DBM 20          /* Max output power */
#define LORA_PREAMBLE_LENGTH 8        /* Standard preamble */

/* LoRa Packet Structure for Telemetry */
typedef struct {
    uint32_t sequence_number;
    float position_x;
    float position_y;
    float yaw;
    float velocity_cmd;
    uint8_t detection_count;
    float battery_voltage;
    uint8_t battery_percentage;
    uint8_t smoke_alert;
    float gas_sensor_ppm;
    uint32_t timestamp_ms;
    uint16_t crc16;
} lora_telemetry_packet_t;

/* LoRa Status */
typedef enum {
    LORA_STATE_IDLE = 0,
    LORA_STATE_RX = 1,
    LORA_STATE_TX = 2,
    LORA_STATE_CAD = 3,
} lora_state_t;

/**
 * @brief Initialize RFM95W LoRa radio
 *
 * @param simulate If true, operates in simulation mode (no actual SPI)
 * @return true if initialization successful
 */
bool rfm95w_init(bool simulate);

/**
 * @brief Transmit LoRa telemetry packet
 *
 * Takes heartbeat data and transmits via LoRa.
 *
 * @param packet Telemetry packet to transmit
 * @return true if transmission initiated successfully
 */
bool rfm95w_transmit_telemetry(const lora_telemetry_packet_t *packet);

/**
 * @brief Check for received LoRa packet
 *
 * @param buffer Output buffer for received data (max 255 bytes)
 * @param len Output parameter with packet length
 * @return true if packet received and valid
 */
bool rfm95w_recv_packet(uint8_t *buffer, uint16_t *len);

/**
 * @brief Get LoRa radio RSSI (Receive Signal Strength Indicator)
 *
 * @return RSSI in dBm (-120 to -80 typical)
 */
int8_t rfm95w_get_rssi(void);

/**
 * @brief Get LoRa radio SNR (Signal-to-Noise Ratio)
 *
 * @return SNR in dB
 */
float rfm95w_get_snr(void);

/**
 * @brief Get current LoRa radio state
 *
 * @return Current state (IDLE, RX, TX, CAD)
 */
lora_state_t rfm95w_get_state(void);

/**
 * @brief Set LoRa radio to RX mode (listening)
 *
 * @return true if mode change successful
 */
bool rfm95w_set_rx_mode(void);

/**
 * @brief Set LoRa radio to TX mode
 *
 * @return true if mode change successful
 */
bool rfm95w_set_tx_mode(void);

/**
 * @brief Set LoRa TX power
 *
 * @param power_dbm Output power in dBm (2-20 typical)
 * @return true if setting successful
 */
bool rfm95w_set_tx_power(uint8_t power_dbm);

/**
 * @brief Get estimated distance to LoRa peer
 *
 * Uses RSSI + SNR + known TX power to estimate distance.
 *
 * @return Distance estimate in meters
 */
float rfm95w_estimate_distance(void);

/**
 * @brief Heartbeat counter for packet loss detection
 *
 * @return Number of packets transmitted so far
 */
uint32_t rfm95w_get_tx_count(void);

#endif /* RFM95W_LORA_DRIVER_H */
