#include "usb_handler.h"
#include "usbd_cdc_if.h"
#include "stm32g4xx_hal_tim.h"
#include <stdint.h>
#include "motor.h"



extern CommandBuffer command_buffer;


static uint8_t calculate_checksum(uint8_t * data,uint32_t len) {
	uint8_t checksum = data[0];
	for (uint32_t i = 1; i < len-1; i++) {
		checksum ^= data[i];
	}
	return checksum;
}

void process_received_data(uint8_t* data, uint32_t len) {
	if (len==0) return;

	if (len == 4) {
		// Traiter directement
		// Exemple : data[0] = commande, data[1-4] = paramètres
		uint8_t response[1];
		switch (data[0]) {
		case 0x24:
			command_buffer.direction = data[1];
			command_buffer.speed = data[2];
			command_buffer.valid = 1;
			response[0] = calculate_checksum(data,len);
			break;
		case 0x40: // Commande 2
			command_buffer.direction = 0;
			command_buffer.speed = 0;
			command_buffer.valid = 0;

			response[0] = calculate_checksum(data,len);
			break;
		default:
			response[0] = ((uint8_t)0x33);
			break;

		}
		CDC_Transmit_FS(response,1);
	}
}

