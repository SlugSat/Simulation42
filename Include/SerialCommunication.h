/**
  ******************************************************************************
  * @file           : SerialCommunication.h
  * @brief          : Send and receive data over serial
  ******************************************************************************
  ** This module uses the libserialport library to send and receive data over
  * UART.
  * 
  * Created by Galen Savidge. Edited 5/6/2019.
  ******************************************************************************
  */

#include <libserialport.h>

// Pointer to a serial port
typedef struct sp_port* port_t;

/**
 * @brief	Initializes a serial port and attempts to connect to an STM32 board
 * @param	None
 * @return	A port_t, or NULL if no STM32 board was found
 */
port_t serialInit(void);

/**
 * @brief	Blocks until a handshake byte is received and then sends back a handshake byte
 * @param	port: the port returned by serialInit()
 * @return	None
 */
void serialHandshake(port_t port);

/**
 * @brief	Sends n floating point numbers over serial
 * @param	port: the port returned by serialInit()
 * @param	f: array of floats of size >= n to be sent
 * @param	n: number of floats to be sent
 * @return	The number of bytes written on success, or a negative error code
 */
int serialSendFloats(port_t port, float* f, unsigned int n);

/**
 * @brief	Blocking function to receive n floating point numbers from serial (with a timeout)
 * @param	port: the port returned by serialInit()
 * @param	f: array of size >= n to hold received floats
 * @param	n: number of floats to be received
 * @return	Number of floats received
 */
int serialReceiveFloats(port_t port, float* f, unsigned int n);

/**
 * @brief	Blocking function to receive a string from serial; exits on timeout or after 300 characters
 * @param	port: the port returned by serialInit()
 * @param	string: char array of size >= strlen + 1 to hold the received string
 * @return	Always 0 for now
 */
int serialReceiveString(port_t port, char* string);
