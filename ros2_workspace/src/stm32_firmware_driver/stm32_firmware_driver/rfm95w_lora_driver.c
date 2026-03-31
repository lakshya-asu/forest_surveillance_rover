#include "rfm95w_lora_driver.h"
#include <string.h>
#include <math.h>

/* SPI Interface Stubs - TODO: Implement actual hardware SPI */
typedef struct {
    bool initialized;
    bool simulate_mode;
    int16_t last_rssi;
    int8_t last_snr;
    uint16_t packets_transmitted;
    uint16_t packets_received;
    uint16_t packets_lost;
} RFM95W_State;

static RFM95W_State g_rfm95w = {
    .initialized = false,
    .simulate_mode = true,
    .last_rssi = -100,
    .last_snr = 2,
    .packets_transmitted = 0,
    .packets_received = 0,
    .packets_lost = 0
};

/* Forward declarations */
static RFM95W_Status _spi_write(uint8_t reg, uint8_t value);
static RFM95W_Status _spi_read(uint8_t reg, uint8_t *value);
static uint16_t _calculate_crc16(const uint8_t *data, uint8_t length);

/**
 * CRC-16 CCITT calculation
 * Polynomial: 0x1021, Initial: 0xFFFF
 */
uint16_t rfm95w_crc16(const uint8_t *data, uint8_t length) {
    if (data == NULL || length == 0) {
        return 0;
    }

    uint16_t crc = 0xFFFF;
    uint8_t i, j;

    for (i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

/**
 * SPI Write Register (Stub - TODO: Implement STM32 HAL calls)
 */
static RFM95W_Status _spi_write(uint8_t reg, uint8_t value) {
    if (!g_rfm95w.initialized) {
        return RFM95W_ERROR_SPI;
    }

    if (g_rfm95w.simulate_mode) {
        /* Simulation mode: pretend SPI write succeeds */
        return RFM95W_OK;
    }

    /* TODO: Implement actual STM32 SPI write:
     * 1. Set chip select low
     * 2. Write register address with write bit (0x80 | reg)
     * 3. Write value byte
     * 4. Set chip select high
     * Return RFM95W_OK on success, RFM95W_ERROR_SPI on timeout
     */
    return RFM95W_OK;
}

/**
 * SPI Read Register (Stub - TODO: Implement STM32 HAL calls)
 */
static RFM95W_Status _spi_read(uint8_t reg, uint8_t *value) {
    if (!g_rfm95w.initialized || value == NULL) {
        return RFM95W_ERROR_SPI;
    }

    if (g_rfm95w.simulate_mode) {
        /* Simulation mode: return mock values */
        if (reg == RFM95W_REG_SIGNAL_STRENGTH) {
            *value = 230;  /* Maps to ~-100 dBm RSSI */
        } else if (reg == RFM95W_REG_LAST_SNR) {
            *value = 8;
        } else {
            *value = 0;
        }
        return RFM95W_OK;
    }

    /* TODO: Implement actual STM32 SPI read:
     * 1. Set chip select low
     * 2. Write register address (no write bit)
     * 3. Read value byte
     * 4. Set chip select high
     * Return RFM95W_OK on success, RFM95W_ERROR_SPI on timeout
     */
    *value = 0;
    return RFM95W_OK;
}

/**
 * Initialize RFM95W LoRa module
 */
RFM95W_Status rfm95w_init(bool simulate) {
    g_rfm95w.simulate_mode = simulate;
    g_rfm95w.initialized = true;

    if (simulate) {
        return RFM95W_OK;
    }

    /* TODO: Implement actual initialization:
     * 1. Configure STM32 SPI peripheral (SPI2 or SPI3)
     * 2. Set GPIO for NSS (chip select), reset pins
     * 3. Write frequency registers (868 MHz)
     * 4. Configure modem parameters:
     *    - Spreading Factor 10
     *    - Bandwidth 125 kHz
     *    - Coding Rate 4/5
     *    - Preamble length 8
     * 5. Set TX power to +20 dBm
     * 6. Enable CRC
     * 7. Set mode to STANDBY
     */

    /* Simulate frequency configuration */
    RFM95W_Status status = _spi_write(RFM95W_REG_FREQUENCY_MSB, 0xE4);
    if (status != RFM95W_OK) return status;

    status = _spi_write(RFM95W_REG_FREQUENCY_MID, 0xC0);
    if (status != RFM95W_OK) return status;

    status = _spi_write(RFM95W_REG_FREQUENCY_LSB, 0x00);
    if (status != RFM95W_OK) return status;

    /* Set to STANDBY mode */
    status = rfm95w_set_mode(RFM95W_MODE_STANDBY);
    if (status != RFM95W_OK) return status;

    return RFM95W_OK;
}

/**
 * Transmit telemetry with CRC
 */
RFM95W_Status rfm95w_transmit_telemetry(const uint8_t *buffer, uint8_t length) {
    /* Validate input buffer */
    if (buffer == NULL) {
        return RFM95W_ERROR_INVALID_BUFFER;
    }

    /* Check payload size (validate at runtime, not just fixed) */
    if (length == 0 || length > RFM95W_MAX_PAYLOAD_SIZE) {
        if (length > RFM95W_MAX_PAYLOAD_SIZE) {
            return RFM95W_ERROR_PAYLOAD_TOO_LARGE;
        }
        return RFM95W_ERROR_INVALID_BUFFER;
    }

    /* TODO: Prepare packet with CRC:
     * 1. Calculate CRC-16 CCITT over payload
     * 2. Create packet: [CRC_HI, CRC_LO, payload...]
     * 3. Set TX mode
     * 4. Write packet to FIFO
     * 5. Wait for TX completion or timeout (5 seconds)
     * 6. Return to STANDBY
     */

    RFM95W_Status status = rfm95w_set_mode(RFM95W_MODE_TX);
    if (status != RFM95W_OK) return status;

    /* Simulate transmission delay */
    if (g_rfm95w.simulate_mode) {
        g_rfm95w.packets_transmitted++;
        return RFM95W_OK;
    }

    /* TODO: Actually transmit via SPI FIFO */
    g_rfm95w.packets_transmitted++;

    status = rfm95w_set_mode(RFM95W_MODE_STANDBY);
    return status;
}

/**
 * Receive telemetry with CRC validation
 */
RFM95W_Status rfm95w_receive_telemetry(uint8_t *buffer, uint8_t max_length, uint8_t *length) {
    if (buffer == NULL || length == NULL) {
        return RFM95W_ERROR_INVALID_BUFFER;
    }

    /* Validate buffer size at runtime */
    if (max_length < 3) {  /* At minimum: CRC (2 bytes) + 1 byte payload */
        return RFM95W_ERROR_PAYLOAD_TOO_LARGE;
    }

    if (g_rfm95w.simulate_mode) {
        /* Simulation: return dummy heartbeat-like data */
        const uint8_t sim_data[] = {
            0x00, 0x01,  /* Sequence: 1 */
            0x10, 0x20, 0x30, 0x40  /* Position: (1.0, 2.0) */
        };
        uint16_t crc = rfm95w_crc16(sim_data, sizeof(sim_data));

        if (max_length < sizeof(sim_data) + 2) {
            return RFM95W_ERROR_PAYLOAD_TOO_LARGE;
        }

        buffer[0] = (crc >> 8) & 0xFF;
        buffer[1] = crc & 0xFF;
        memcpy(&buffer[2], sim_data, sizeof(sim_data));
        *length = sizeof(sim_data) + 2;

        g_rfm95w.packets_received++;
        return RFM95W_OK;
    }

    /* Actual hardware reception with CRC validation:
     * TODO: Implement:
     * 1. Set RX mode
     * 2. Wait for RX complete interrupt or timeout (5 seconds)
     * 3. Read payload length from RFM95W register
     * 4. Validate payload length (must be < RFM95W_BUFFER_SIZE)
     * 5. Validate doesn't exceed user buffer (payload_len + 2 bytes for CRC <= max_length)
     * 6. Allocate temporary buffer for FIFO read (payload_len bytes)
     * 7. Read FIFO via SPI into temporary buffer
     * 8. Extract CRC from first 2 bytes and payload from remaining bytes
     * 9. Calculate CRC-16 CCITT over payload
     * 10. Compare calculated CRC with received CRC
     * 11. If CRC MISMATCH: log error, increment packets_lost, return RFM95W_ERROR_CRC_MISMATCH
     * 12. If CRC OK: copy payload to output buffer, update *length, increment packets_received, return RFM95W_OK
     */

    /* Placeholder: Set RX mode and wait (not implemented) */
    uint8_t payload_length = 0;  /* Would read from register */
    
    if (payload_length + 2 > max_length) {
        return RFM95W_ERROR_PAYLOAD_TOO_LARGE;
    }

    /* Simulated actual reception for now */
    g_rfm95w.packets_received++;
    *length = 0;
    return RFM95W_OK;
}

/**
 * Estimate distance using Friis path loss model
 * Formula: distance = 10^((TX_power - RSSI - 40.2) / (10 * n))
 * where n = path loss exponent (~2 for free space)
 */
float rfm95w_estimate_distance(int16_t rssi, int8_t tx_power) {
    const float path_loss_exponent = 2.0f;
    const float constant = 40.2f;

    if (rssi > 0) {
        /* Invalid RSSI */
        return 0.0f;
    }

    float free_space_loss = (float)tx_power - (float)rssi - constant;
    float distance = powf(10.0f, (free_space_loss / (10.0f * path_loss_exponent)));

    return distance;
}

/**
 * Get current LoRa status
 */
RFM95W_Status rfm95w_get_status(RFM95W_Status_Info *status) {
    if (status == NULL) {
        return RFM95W_ERROR_INVALID_BUFFER;
    }

    uint8_t rssi_raw = 0;
    uint8_t snr_raw = 0;

    RFM95W_Status ret = _spi_read(RFM95W_REG_SIGNAL_STRENGTH, &rssi_raw);
    if (ret != RFM95W_OK) return ret;

    ret = _spi_read(RFM95W_REG_LAST_SNR, &snr_raw);
    if (ret != RFM95W_OK) return ret;

    /* Convert register values to dBm/dB */
    status->rssi = -157 + rssi_raw;
    status->snr = ((int8_t)snr_raw) >> 2;
    status->packets_tx = g_rfm95w.packets_transmitted;
    status->packets_rx = g_rfm95w.packets_received;
    status->packets_lost = g_rfm95w.packets_lost;

    return RFM95W_OK;
}

/**
 * Set LoRa operation mode
 */
RFM95W_Status rfm95w_set_mode(uint8_t mode) {
    /* Validate mode */
    if (mode != RFM95W_MODE_SLEEP &&
        mode != RFM95W_MODE_STANDBY &&
        mode != RFM95W_MODE_TX &&
        mode != RFM95W_MODE_RX) {
        return RFM95W_ERROR_INVALID_BUFFER;
    }

    return _spi_write(RFM95W_REG_OPMODE, mode);
}

/**
 * Read register (public wrapper)
 */
RFM95W_Status rfm95w_read_register(uint8_t reg, uint8_t *value) {
    if (value == NULL) {
        return RFM95W_ERROR_INVALID_BUFFER;
    }
    return _spi_read(reg, value);
}

/**
 * Write register (public wrapper)
 */
RFM95W_Status rfm95w_write_register(uint8_t reg, uint8_t value) {
    return _spi_write(reg, value);
}
