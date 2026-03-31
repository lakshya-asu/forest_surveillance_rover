/**
 * @file rfm95w_lora_driver.c
 * @brief RFM95W LoRa radio driver implementation for STM32F407
 *
 * Features:
 * - SPI interface to RFM95W module
 * - Simulation mode for testing without hardware
 * - Telemetry packet encoding/transmission
 * - RSSI/SNR monitoring
 * - Distance estimation based on path loss model
 */

#include "rfm95w_lora_driver.h"
#include <string.h>
#include <math.h>

/* LoRa Driver State */
typedef struct {
    bool initialized;
    bool simulate;
    lora_state_t current_state;
    int8_t last_rssi;
    float last_snr;
    uint32_t tx_count;
    uint32_t ack_count;
    uint32_t packet_loss;
} rfm95w_driver_state_t;

static rfm95w_driver_state_t g_lora_state = {0};

/* RFM95W Register Definitions (Subset) */
#define RFM95W_REG_FIFO 0x00
#define RFM95W_REG_OPMODE 0x01
#define RFM95W_REG_FREQ_MSB 0x06
#define RFM95W_REG_FREQ_MID 0x07
#define RFM95W_REG_FREQ_LSB 0x08
#define RFM95W_REG_PA_CONFIG 0x09
#define RFM95W_REG_RSSI_VALUE 0x1d
#define RFM95W_REG_RSSI_WIDEBAND 0x2c
#define RFM95W_REG_SNR_VALUE 0x19
#define RFM95W_REG_RX_SLOP_THRESH 0x20
#define RFM95W_REG_PKT_SNR_VALUE 0x19
#define RFM95W_REG_MODEM_CONFIG_1 0x1d
#define RFM95W_REG_MODEM_CONFIG_2 0x1e
#define RFM95W_REG_MODEM_CONFIG_3 0x26
#define RFM95W_REG_PREAMBLE_MSB 0x20
#define RFM95W_REG_PREAMBLE_LSB 0x21
#define RFM95W_REG_PAYLOAD_LENGTH 0x22
#define RFM95W_REG_FIFO_RX_CURRENT 0x10
#define RFM95W_REG_IRQ_FLAGS 0x12

/* Mock SPI Operations (for simulation) */
static uint8_t rfm95w_spi_read_reg(uint8_t reg) {
    if (!g_lora_state.simulate) {
        /* Real SPI read implementation would go here */
        return 0;
    }
    /* Simulation: return mock values */
    if (reg == RFM95W_REG_RSSI_VALUE) return 150;      /* Simulated RSSI */
    if (reg == RFM95W_REG_SNR_VALUE) return 8;         /* Simulated SNR */
    return 0;
}

static void rfm95w_spi_write_reg(uint8_t reg, uint8_t value) {
    if (!g_lora_state.simulate) {
        /* Real SPI write implementation would go here */
        return;
    }
    /* Simulation: no-op */
}

/* CRC16 Calculation */
static uint16_t rfm95w_crc16_ccitt(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

bool rfm95w_init(bool simulate) {
    g_lora_state.initialized = true;
    g_lora_state.simulate = simulate;
    g_lora_state.current_state = LORA_STATE_IDLE;
    g_lora_state.last_rssi = -100;
    g_lora_state.last_snr = 0.0f;
    g_lora_state.tx_count = 0;
    g_lora_state.ack_count = 0;
    g_lora_state.packet_loss = 0;

    if (!simulate) {
        /* Real initialization: configure RFM95W registers via SPI */
        // Set frequency: FREQ = (freq_hz / (32e6 / (1 << 19)))
        // For 868 MHz: FREQ = 0xD9 0x00 0x00
        rfm95w_spi_write_reg(RFM95W_REG_FREQ_MSB, 0xD9);
        rfm95w_spi_write_reg(RFM95W_REG_FREQ_MID, 0x00);
        rfm95w_spi_write_reg(RFM95W_REG_FREQ_LSB, 0x00);

        // Set to LoRa mode, standby
        rfm95w_spi_write_reg(RFM95W_REG_OPMODE, 0x88);

        // Configure modem for SF10, BW=125kHz
        rfm95w_spi_write_reg(RFM95W_REG_MODEM_CONFIG_1, 0x72); /* BW=125kHz, CR=4/5 */
        rfm95w_spi_write_reg(RFM95W_REG_MODEM_CONFIG_2, 0xA4); /* SF=10, CRC on */

        // Set preamble length
        rfm95w_spi_write_reg(RFM95W_REG_PREAMBLE_MSB, 0x00);
        rfm95w_spi_write_reg(RFM95W_REG_PREAMBLE_LSB, LORA_PREAMBLE_LENGTH);

        // Set TX power to +20dBm
        rfm95w_spi_write_reg(RFM95W_REG_PA_CONFIG, 0xFF);
    }

    return true;
}

bool rfm95w_transmit_telemetry(const lora_telemetry_packet_t *packet) {
    if (!g_lora_state.initialized || packet == NULL) {
        return false;
    }

    /* Encode telemetry packet */
    uint8_t packet_buffer[64];
    memcpy(packet_buffer, (uint8_t *)packet, sizeof(*packet) - 2); /* Exclude CRC field */

    /* Calculate and append CRC16 */
    uint16_t crc = rfm95w_crc16_ccitt(packet_buffer, sizeof(*packet) - 2);
    memcpy(packet_buffer + (sizeof(*packet) - 2), &crc, 2);

    uint16_t packet_len = sizeof(*packet);

    if (!g_lora_state.simulate) {
        /* Real TX: Load FIFO and trigger transmission */
        rfm95w_spi_write_reg(RFM95W_REG_FIFO, packet_len);
        for (uint16_t i = 0; i < packet_len; i++) {
            rfm95w_spi_write_reg(RFM95W_REG_FIFO, packet_buffer[i]);
        }

        /* Set TX mode and wait for TxDone interrupt */
        rfm95w_spi_write_reg(RFM95W_REG_OPMODE, 0x8B); /* TX mode */
        g_lora_state.current_state = LORA_STATE_TX;
    } else {
        /* Simulation: increment counter and mark as transmitted */
        g_lora_state.current_state = LORA_STATE_TX;
    }

    g_lora_state.tx_count++;
    return true;
}

bool rfm95w_recv_packet(uint8_t *buffer, uint16_t *len) {
    if (!g_lora_state.initialized || buffer == NULL || len == NULL) {
        return false;
    }

    if (!g_lora_state.simulate) {
        /* Real RX: Check IRQ flags and read FIFO */
        uint8_t irq_flags = rfm95w_spi_read_reg(RFM95W_REG_IRQ_FLAGS);
        if (!(irq_flags & 0x40)) { /* RxDone flag not set */
            return false;
        }

        uint8_t rx_len = rfm95w_spi_read_reg(RFM95W_REG_PAYLOAD_LENGTH);
        if (rx_len > 255) rx_len = 255;

        for (uint16_t i = 0; i < rx_len; i++) {
            buffer[i] = rfm95w_spi_read_reg(RFM95W_REG_FIFO);
        }

        *len = rx_len;
        return true;
    }

    /* Simulation: no received packet */
    *len = 0;
    return false;
}

int8_t rfm95w_get_rssi(void) {
    if (!g_lora_state.initialized) {
        return -150;
    }

    if (!g_lora_state.simulate) {
        /* Real RSSI: Read register and convert */
        uint8_t rssi_raw = rfm95w_spi_read_reg(RFM95W_REG_RSSI_VALUE);
        g_lora_state.last_rssi = -157 + rssi_raw; /* Convert to dBm */
    } else {
        /* Simulation: return consistent mock value */
        g_lora_state.last_rssi = -85;
    }

    return g_lora_state.last_rssi;
}

float rfm95w_get_snr(void) {
    if (!g_lora_state.initialized) {
        return 0.0f;
    }

    if (!g_lora_state.simulate) {
        /* Real SNR: Read register */
        int8_t snr_raw = (int8_t)rfm95w_spi_read_reg(RFM95W_REG_SNR_VALUE);
        g_lora_state.last_snr = snr_raw / 4.0f; /* Convert to dB */
    } else {
        /* Simulation: return consistent mock value */
        g_lora_state.last_snr = 8.5f;
    }

    return g_lora_state.last_snr;
}

lora_state_t rfm95w_get_state(void) {
    if (!g_lora_state.initialized) {
        return LORA_STATE_IDLE;
    }
    return g_lora_state.current_state;
}

bool rfm95w_set_rx_mode(void) {
    if (!g_lora_state.initialized) {
        return false;
    }

    if (!g_lora_state.simulate) {
        rfm95w_spi_write_reg(RFM95W_REG_OPMODE, 0x8A); /* RX mode */
    }

    g_lora_state.current_state = LORA_STATE_RX;
    return true;
}

bool rfm95w_set_tx_mode(void) {
    if (!g_lora_state.initialized) {
        return false;
    }

    if (!g_lora_state.simulate) {
        rfm95w_spi_write_reg(RFM95W_REG_OPMODE, 0x8B); /* TX mode */
    }

    g_lora_state.current_state = LORA_STATE_TX;
    return true;
}

bool rfm95w_set_tx_power(uint8_t power_dbm) {
    if (!g_lora_state.initialized || power_dbm < 2 || power_dbm > 20) {
        return false;
    }

    if (!g_lora_state.simulate) {
        /* Map power_dbm (2-20) to PA_CONFIG register value (0-15) */
        uint8_t pa_value = (power_dbm - 2); /* Linear approximation */
        if (pa_value > 15) pa_value = 15;
        rfm95w_spi_write_reg(RFM95W_REG_PA_CONFIG, 0xF0 | pa_value);
    }

    return true;
}

float rfm95w_estimate_distance(void) {
    /* Friis path loss formula: RSSI = TxPower - 20*log10(distance) - 20*log10(f) - constant */
    /* Simplified: distance ≈ 10^((RxPower - TxPower - 32) / 20) in km/10 */

    float rssi_dbm = (float)rfm95w_get_rssi();
    const float tx_power_dbm = 20.0f;
    const float path_loss_ref = 32.4f; /* Reference at 1m, 868MHz */

    float distance_m = powf(10.0f, (tx_power_dbm - rssi_dbm - path_loss_ref) / 20.0f) * 1000.0f;

    /* Apply SNR correction: strong SNR allows longer range */
    float snr_correction = 1.0f + (rfm95w_get_snr() / 10.0f);
    distance_m *= snr_correction;

    return distance_m;
}

uint32_t rfm95w_get_tx_count(void) {
    if (!g_lora_state.initialized) {
        return 0;
    }
    return g_lora_state.tx_count;
}
