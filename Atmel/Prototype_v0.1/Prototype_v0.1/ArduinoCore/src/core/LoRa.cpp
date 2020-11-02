/*
 * LoRa.c
 *
 * Created: 10/26/2020 2:59:34 PM
 *  Author: Justin
 */ 

#include <LoRa.h>

int8_t sendAT() {
	return sendATCommand(AT_NONE, 0, NULL, 0);
}

int8_t setNetworkID(uint8_t id) {
	return sendATCommand(AT_NETWORKID, id, NULL, 0);
}

int8_t setNodeID(uint8_t id) {
	return sendATCommand(AT_ADDRESS, id, NULL, 0);
}

int8_t sendData(uint8_t id, char *data, uint8_t size) {
	return sendATCommand(AT_SEND, id, data, size);
}

int8_t sendATCommand(uint8_t opcode, uint8_t id, char *data, uint8_t size) {
	char temp[4];
	
	// message header
	Serial.write("AT");

	// message body
	switch(opcode) {
	case AT_NONE:
		break;
	
	case AT_NETWORKID:
		itoa(id, temp, 10);
		Serial.write("+NETWORKID=");
		Serial.write(temp, strlen(temp));
		break;
	
	case AT_ADDRESS:
		itoa(id, temp, 10);
		Serial.write("+ADDRESS=");
		Serial.write(temp, strlen(temp));
		break;
	
	case AT_SEND:
		itoa(id, temp, 10);
		Serial.write("+SEND=");
		Serial.write(temp, strlen(temp));
		Serial.write(",");
		itoa(size, temp, 10);
		Serial.write(temp, strlen(temp));
		Serial.write(",");
		Serial.write(data, size);
		break;
	
	default:
		break;
	}
	
	// message tail
	Serial.write("\r\n");
	
	return checkResponse();
}

int8_t checkResponse() {
	delay(1000);
	String rx = Serial.readString();
	if(rx.indexOf("OK") >= 0) {
		return 1;
	}
	else {
		return -1;
	}
}