/**
  ******************************************************************************
  * @file           : SerialCommunication.c
  * @brief          : Send and receive data over serial
  ******************************************************************************
  ** This module uses the libserialport library to send and receive data over
  * UART.
  * 
  * Created by Galen Savidge. Edited 4/13/2019.
  ******************************************************************************
  */

#include <SerialCommunication.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <PacketProtocol.h>

#define BAUD 115200
#define UART_TIMEOUT 250
#define HANDSHAKE_TIMEOUT 500

#define STM_DESCRIPTION "STM32 STLink"

// Private functions
void receivePacket(port_t port, uint8_t* packet, unsigned int bytes) {
	int bytes_received;
	do{
		bytes_received = sp_input_waiting(port);
	} while(bytes_received < bytes);
	sp_blocking_read(port, packet, bytes, UART_TIMEOUT);
}

// Public functions
port_t serialInit(void) {
	// Get list of serial devices
	port_t* ports;
	enum sp_return err;
	err = sp_list_ports(&ports);
	
	port_t port;
	int i = 0;
	char* description;
	do {
		port = ports[i];
		if(port == NULL) {
			printf("STM32 board not found!\n");
			return NULL;
		}
		i++;
		description = sp_get_port_description(port);
		description[strlen(STM_DESCRIPTION)] = '\0'; // Truncate description
	} while(strcmp(description, STM_DESCRIPTION) != 0 /*&& strcmp(description, "ttyS0") != 0*/);
	
	printf("Opening port:\n");
	printf("%s | %s\n", sp_get_port_name(port), sp_get_port_description(port));
	err = sp_open(port, SP_MODE_READ_WRITE);
	if(err != 0) {
		printf("Error %d opening port!\n", err);
		return NULL;
	}

	sp_set_baudrate(port, BAUD);

	sp_flush(port, SP_BUF_BOTH);
	//sp_drain(port);

	return port;
}


void serialHandshake(port_t port) {
	sp_flush(port, SP_BUF_BOTH); // Flush UART registers

	uint8_t send[1] = {HANDSHAKE_BYTE};
	uint8_t receive[1];

	// Wait to receive handshake byte
	while(1) {
		if(sp_blocking_read(port, receive, 1, UART_TIMEOUT) > 0) {
			if(receive[0] == HANDSHAKE_BYTE) {

				// Send handshake byte
				sp_blocking_write(port, send, 1, UART_TIMEOUT);

				// Flush UART registers and return
				sp_flush(port, SP_BUF_BOTH);
				return;
			}
		}
	}
}


int serialSendFloats(port_t port, float* f, unsigned int n) {
	// Create data packet
	uint8_t data_packet[BYTES_PER_FLOAT*n];
	floatsToPacket(f, data_packet, n);

	// Send packet
	enum sp_return err = sp_blocking_write(port, data_packet, BYTES_PER_FLOAT*n, UART_TIMEOUT);
	sp_drain(port);
	return err;
}

int serialReceiveFloats(port_t port, float* f, unsigned int n) {
	// Receive data packet
	uint8_t data_packet[BYTES_PER_FLOAT*n];
	receivePacket(port, data_packet, BYTES_PER_FLOAT*n);
//	enum sp_return err = sp_blocking_read(port, data_packet, BYTES_PER_FLOAT*n, UART_TIMEOUT);
//
//	if(err == SP_OK) {
//		// Read packet
//		packetToFloats(f, data_packet, n);
//	}

	//return err;
	return 0;
}

int serialReceiveString(port_t port, char* string) {
	int i = -1;
	do {
		while(sp_input_waiting(port) < 1);
		i++;
		if(sp_blocking_read(port, &string[i], 1, UART_TIMEOUT) <= 0) return 1;
	} while(string[i] != '\0' && i < 300); // Wait for null character
	return 0;
}
