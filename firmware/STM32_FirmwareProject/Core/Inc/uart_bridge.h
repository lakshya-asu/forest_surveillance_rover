#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define UART_RX_BUFFER_SIZE 256U
#define UART_TX_BUFFER_SIZE 256U

typedef struct {
    uint8_t msg_id;
    uint8_t payload[128];
    uint16_t payload_len;
} uart_frame_t;

void uart_bridge_init(UART_HandleTypeDef *huart);
void uart_bridge_start_rx(void);
void uart_bridge_on_rx_byte(uint8_t byte);
bool uart_bridge_pop_frame(uart_frame_t *frame);
bool uart_bridge_send_frame(uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);

uint16_t uart_crc16(const uint8_t *data, uint16_t len);
uint16_t cobs_encode(const uint8_t *input, uint16_t length, uint8_t *output);
uint16_t cobs_decode(const uint8_t *input, uint16_t length, uint8_t *output);

#endif
