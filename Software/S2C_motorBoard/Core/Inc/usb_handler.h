#ifndef USB_HANDLER_H
#define USB_HANDLER_H

#include "stm32g4xx_hal.h"
#include "usart.h"

#define BUFFER_SIZE 256 //  (multiples of 2 recommanded for DMA)
#define HEADER_BYTE 0x24
#define SEPARATOR_BYTE 0x40
#define MAX_PAYLOAD_SIZE 4
#define FRAME_SIZE 5


typedef struct {
    uint8_t payload[MAX_PAYLOAD_SIZE]; ///< Payload data
    uint8_t length;                    ///< Payload length
    uint8_t checksum;                  ///< Received checksum
    uint8_t valid;                     ///< 1 if frame is valid, 0 otherwise
} Frame_t;


void process_received_data(uint8_t *UserRxBufferFS,uint32_t Len);


#endif /* USB_HANDLER_H */
