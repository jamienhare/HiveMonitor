/*
 * LoRa.h
 *
 * Created: 10/26/2020 2:55:18 PM
 *  Author: Justin
 */ 
#include <Arduino.h>
#include <string.h>

#define NETWORKID 6
#define TXNODE    120
#define RXNODE    100

// AT Message Opcodes
#define AT_NONE       0x00
#define AT_NETWORKID  0x01
#define AT_ADDRESS    0x02
#define AT_SEND       0x03

int8_t sendAT();
int8_t setNetworkID(uint8_t id);
int8_t setNodeID(uint8_t id);
int8_t sendData(uint8_t id, char *data, uint8_t size);

int8_t sendATCommand(uint8_t opcode, uint8_t id, char *data, uint8_t size);
int8_t checkResponse();