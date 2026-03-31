#ifndef RFM95W_LORA_DRIVER_H
#define RFM95W_LORA_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/* LoRa Configuration Constants */
#define RFM95W_MAX_PAYLOAD_SIZE 64
#define RFM95W_BUFFER_SIZE 255
#define RFM95W_FREQUENCY_868_MHZ 868000000UL
#define RFM95W_PREAMBLE_LENGTH 8
#define RFM95W_SPREADING_FACTOR 10
#define RFM95W_BANDWIDTH_125_KHZ 7
#define RFM95W_TRANSMISSION_POWER 20

/* RFM95W Register Addresses */
#define RFM95W_REG_FIFO 0x00
#define RFM95W_REG_OPMODE 0x01
#define RFM95W_REG_FREQUENCY_MSB 0x06
#define RFM95W_REG_FREQUENCY_MID 0x07
#define RFM95W_REG_FREQUENCY_LSB 0x08
#define RFM95W_REG_MODEM_CONFIG1 0x1D
#define RFM95W_REG_MODEM_CONFIG2 0x1E
#define RFM95W_REG_PAYLOAD_LENGTH 0x22
#define RFM95W_REG_SIGNAL_STRENGTH 0x1D
#define RFM95W_REG_LAST_SNR 0x19
#define RFM95W_REG_IRQ_FLAGS 0x12

/* Operation Modes */
#define RFM95W_MODE_SLEEP 0x00
#define RFM95W_MODE_STANDBY 0x01
#define RFM95W_MODE_TX 0x03
#define RFM95W_MODE_RX 0x05

/* Status Codes */
typedef enum {
    RFM95W_OK = 0,
    RFM95W_ERROR_SPI = -1,
    RFM95W_ERROR_PAYLOAD_TOO_LARGE = -2,
    RFM95W_ERROR_CRC_MISMATCH = -3,
    RFM95W_ERROR_TIMEOUT = -4,
    RFM95W_ERROR_INVALID_BUFFER = -5
} RFM95W_Status;

/* LoRa Packet Structure with CRC */
typedef struct {
    uint16_t crc16;              /* CRC-16 CCITT checksum */
    uint8_t payload_length;      /* Actual payload size */
    uint8_t payload[RFM95W_MAX_PAYLOAD_SIZE];
} RFM95W_Packet;

/* Status Information */
typedef struct {
    int16_t rssi;              /* Received Signal Strength Indicator */
    int8_t snr;                /* Signal-to-Noise Ratio */
    uint16_t packets_tx;       /* Transmitted packet count */
    uint16_t packets_rx;       /* Received packet count */
    uint16_t packets_lost;     /* Lost packet count */
} RFM95W_Status_Info;

/* Function Declarations */

/**
 * Initialize RFM95W LoRa module
 * @param simulate: If true, run in simulation mode (no actual SPI)
 * @return Status code
 */
RFM95W_Status rfm95w_init(bool simulate);

/**
 * Transmit telemetry data via LoRa
 * @param buffer: Pointer to payload buffer
 * @param length: Payload length in bytes
 * @return Status code
 */
RFM95W_Status rfm95w_transmit_telemetry(const uint8_t *buffer, uint8_t length);

/**
 * Receive telemetry data via LoRa with CRC validation
 * @param buffer: Pointer to output buffer
 * @param max_length: Maximum buffer size
 * @param length: Pointer to store actual received length
 * @return Status code (negative if CRC fails)
 */
RFM95W_Status rfm95w_receive_telemetry(uint8_t *buffer, uint8_t max_length, uint8_t *length);

/**
 * Estimate distance based on RSSI using Friis path loss model
 * @param rssi: Received Signal Strength Indicator
 * @param tx_power: Transmission power in dBm
 * @return Estimated distance in meters
 */
float rfm95w_estimate_distance(int16_t rssi, int8_t tx_power);

/**
 * Get current LoRa status
 * @param status: Pointer to status structure
 * @return Status code
 */
RFM95W_Status rfm95w_get_status(RFM95W_Status_Info *status);

/**
 * Calculate CRC-16 CCITT checksum
 * @param data: Pointer to data buffer
 * @param length: Data length
 * @return 16-bit CRC checksum
 */
uint16_t rfm95w_crc16(const uint8_t *data, uint8_t length);

/**
 * Set LoRa operation mode
 * @param mode: Operating mode (SLEEP, STANDBY, TX, RX)
 * @return Status code
 */
RFM95W_Status rfm95w_set_mode(uint8_t mode);

/**
 * Read register value from RFM95W
 * @param reg: Register address
 * @param value: Pointer to store register value
 * @return Status code
 */
RFM95W_Status rfm95w_read_register(uint8_t reg, uint8_t *value);

/**
 * Write register value to RFM95W
 * @param reg: Register address
 * @param value: Value to write
 * @return Status code
 */
RFM95W_Status rfm95w_write_register(uint8_t reg, uint8_t value);

#endif /* RFM95W_LORA_DRIVER_H */
