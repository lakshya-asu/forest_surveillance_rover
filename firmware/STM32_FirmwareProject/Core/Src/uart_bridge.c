#include "uart_bridge.h"

#include <string.h>

static UART_HandleTypeDef *s_uart = NULL;
static uint8_t s_rx_byte = 0;
static uint8_t s_rx_frame[UART_RX_BUFFER_SIZE];
static uint16_t s_rx_index = 0;
static uart_frame_t s_latest_frame;
static bool s_has_frame = false;

static inline uint32_t uart_lock_irq(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void uart_unlock_irq(uint32_t primask) {
    if (primask == 0U) {
        __enable_irq();
    }
}

void uart_bridge_init(UART_HandleTypeDef *huart) {
    s_uart = huart;
    s_rx_index = 0;
    s_has_frame = false;
}

void uart_bridge_start_rx(void) {
    if (s_uart != NULL) {
        HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);
    }
}

void uart_bridge_on_rx_byte(uint8_t byte) {
    if (byte == 0x00U) {
        uint8_t decoded[UART_RX_BUFFER_SIZE] = {0};
        uint16_t decoded_len = cobs_decode(s_rx_frame, s_rx_index, decoded);
        s_rx_index = 0;

        if (decoded_len >= 3U) {
            uint16_t payload_len = (uint16_t)(decoded[1] | (decoded[2] << 8));
            if ((payload_len + 5U) <= decoded_len && payload_len <= sizeof(s_latest_frame.payload)) {
                uint16_t frame_crc = (uint16_t)(decoded[3U + payload_len] | (decoded[4U + payload_len] << 8));
                uint16_t calc_crc = uart_crc16(decoded, (uint16_t)(3U + payload_len));
                if (frame_crc == calc_crc) {
                    uint32_t primask = uart_lock_irq();
                    s_latest_frame.msg_id = decoded[0];
                    s_latest_frame.payload_len = payload_len;
                    memcpy(s_latest_frame.payload, &decoded[3], payload_len);
                    s_has_frame = true;
                    uart_unlock_irq(primask);
                }
            }
        }
        return;
    }

    if (s_rx_index < UART_RX_BUFFER_SIZE) {
        s_rx_frame[s_rx_index++] = byte;
    } else {
        s_rx_index = 0;
    }
}

bool uart_bridge_pop_frame(uart_frame_t *frame) {
    if (frame == NULL) {
        return false;
    }

    uint32_t primask = uart_lock_irq();
    if (!s_has_frame) {
        uart_unlock_irq(primask);
        return false;
    }

    *frame = s_latest_frame;
    s_has_frame = false;
    uart_unlock_irq(primask);
    return true;
}

bool uart_bridge_send_frame(uint8_t msg_id, const uint8_t *payload, uint16_t payload_len) {
    if (s_uart == NULL || payload == NULL || payload_len > 128U) {
        return false;
    }

    uint8_t raw[UART_TX_BUFFER_SIZE] = {0};
    uint8_t encoded[UART_TX_BUFFER_SIZE] = {0};

    raw[0] = msg_id;
    raw[1] = (uint8_t)(payload_len & 0xFFU);
    raw[2] = (uint8_t)((payload_len >> 8U) & 0xFFU);
    memcpy(&raw[3], payload, payload_len);

    uint16_t crc = uart_crc16(raw, (uint16_t)(payload_len + 3U));
    raw[3U + payload_len] = (uint8_t)(crc & 0xFFU);
    raw[4U + payload_len] = (uint8_t)((crc >> 8U) & 0xFFU);

    uint16_t encoded_len = cobs_encode(raw, (uint16_t)(payload_len + 5U), encoded);
    encoded[encoded_len++] = 0x00U;

    return HAL_UART_Transmit(s_uart, encoded, encoded_len, 50U) == HAL_OK;
}

uint16_t uart_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

uint16_t cobs_encode(const uint8_t *input, uint16_t length, uint8_t *output) {
    uint16_t read_index = 0;
    uint16_t write_index = 1;
    uint16_t code_index = 0;
    uint8_t code = 1;

    while (read_index < length) {
        if (input[read_index] == 0U) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if (code == 0xFFU) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }

    output[code_index] = code;
    return write_index;
}

uint16_t cobs_decode(const uint8_t *input, uint16_t length, uint8_t *output) {
    uint16_t read_index = 0;
    uint16_t write_index = 0;

    while (read_index < length) {
        uint8_t code = input[read_index++];
        if (code == 0U) {
            break;
        }

        for (uint8_t i = 1; i < code; ++i) {
            if (read_index >= length) {
                return 0U;
            }
            output[write_index++] = input[read_index++];
        }

        if (code != 0xFFU && read_index < length) {
            output[write_index++] = 0U;
        }
    }

    return write_index;
}
