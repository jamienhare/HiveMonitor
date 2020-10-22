/*
  main.cpp - Main loop for Arduino sketches
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
// Device Libraries
#include <Adafruit_CCS811.h>
#include <DHT.h>
#include <SD.h>

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }
	
// Helper functions
bool readCCS(uint16_t*, uint16_t*);
bool readDHT(float*, float*);
bool initSD();
void logData(uint16_t*, uint16_t*, float*, float*);

// CCS811 Air Quality/CO2 Sensor
Adafruit_CCS811 ccs;

// DHT22 Temperature/Relative Humidity Sensor
#define DHT_PIN  2
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// SD Card
#define SD_CS  8  // chip select
#define SD_CD  9  // card detect
File fd;
char fileName[] = "log.txt";
bool sdBegan = false; // library complains if begin() called twice

int main(void)
{
	init();

	initVariant();

	// Auto-generated from Arduino sketch. Allows for use of USB libraries. We
	// probably won't need this.
#if defined(USBCON)
	USBDevice.attach();
#endif

	/********** begin setup **********/
	Serial.begin(9600);
	Serial.println("--- EECS 473 : Wireless Beehive Monitor Prototype ---");
	
	// CCS811
	if(!ccs.begin()) {
		Serial.println("Failed to initialize CCS811 sensor.");
		while(1); // trap error
	}
	while(!ccs.available());
	
	// DHT22
	dht.begin();
	
	// SD Card
	pinMode(SD_CD, INPUT);
	if(!initSD()) {
		// if initialization fails, trap it
		Serial.println("Failed to initialize SD card.");
		while(1);
	}
	
	/********** end setup **********/
	
    
	/********** begin main program loop **********/
	bool ledOn = false;
	uint16_t eco2, tvoc;
	float h, t;
	bool status1, status2;
	
	for (;;) {
		// 1 second delay
		delay(1000);
		
		// CCS811
		status1 = readCCS(&eco2, &tvoc);
		if(status1) {
			Serial.print("CO2: ");
			Serial.print(eco2);
			Serial.print("ppm, TVOC: ");
			Serial.println(tvoc);
		}
		else {
			Serial.println("No CCS data read.");
		}
		
		// DHT22
		status2 = readDHT(&h, &t);
		if(status2) {
			Serial.print("RH: ");
			Serial.print(h);
			Serial.print("%, Temp (C): ");
			Serial.print(t);
			Serial.print(" deg, Temp (F): ");
			Serial.print(dht.convertCtoF(t));
			Serial.println(" deg");
		}
		
		// Write collected data to SD file
		uint16_t *arg1 = NULL, *arg2 = NULL;
		float *arg3 = NULL, *arg4 = NULL;
		if(status1) {
			arg1 = &eco2;
			arg2 = &tvoc;
		}
		if(status2) {
			arg3 = &h;
			arg4 = &t;
		}
		
		logData(arg1, arg2, arg3, arg4);
		
		// Arduino function, we likely won't need this. Allows for implementation of
		// custom USB event handler.
		if (serialEventRun) serialEventRun();
	}
	/********** end main program loop **********/
        
	return 0;
}

/*
 * Reads eCO2 and TVOC data from CCS811 and stores in provided pointers. Returns
 * true if reading was successful, false otherwise. In the event reading was 
 * unsuccessful, data at pointers remains unchanged.
 *
 * eco2 - pointer to store read eCO2 (ppm)
 * tvoc - pointer to store read TVOC
 *
 * REQUIRES: - begin() previously called for CCS811
 *           - eco2 != NULL
 *           - tvoc != NULL
*/
bool readCCS(uint16_t *eco2, uint16_t *tvoc) {
	if(eco2 == NULL || tvoc == NULL) {
		// bad inputs
		return false;
	}
	
	if(ccs.available()) {
		if(!ccs.readData()) {
			// successful reading
			*eco2 = ccs.geteCO2();
			*tvoc = ccs.getTVOC();
			return true;
		}
		else {
			// sensor error
			Serial.println("CCS811 ERROR");
			return false;
		}
	}
	
	// no data to read
	return false;
}

/*
 * Reads temperature and relative humidity from DHT22 and stores in provided
 * pointers. Returns true if reading was successful, false otherwise. In the
 * event reading was unsuccessful, validity of data at pointers is not
 * guaranteed.
 *
 * h  - pointer to store read relative humidity (%)
 * t  - pointer to store read temperature (deg Celsius)
 *
 * REQUIRES: - begin() previously called for DHT22
 *           - rh != NULL
 *           - tc != NULL
*/
bool readDHT(float *h, float *t) {
	if(h == NULL || t == NULL) {
		// bad inputs
		return false;
	}
	
	*h = dht.readHumidity();
	*t = dht.readTemperature();
	if(isnan(*h) || isnan(*t)) {
		// bad data read
		Serial.println("DHT22 ERROR");
		return false;
	}
	
	return true;
}

/*
 * Initializes an SD card. Waits indefinitely if no card is detected.
 *
 * REQUIRES: Card detect pin has been set to mode INPUT.
 * RETURNS:  true if card initialization succeeded, false otherwise.
*/
bool initSD() {
	
	// check for a card
	if(!digitalRead(SD_CD)) {
		Serial.println("No card detected. Waiting...");
		while(!digitalRead(SD_CD));
		delay(250);
	}
	
	// begin() returns false no matter what if not the first call
	if(!SD.begin(SD_CS) && !sdBegan) {
		Serial.println("Initialization failed.");
		return false;
	}
	else {
		sdBegan = true;
	}
	
	return true;
}

/*
 * Writes a set of data readings to log.txt on the SD card. Each data point to
 * be logged should be passed in via the appropriate pointer. Passing NULL for
 * a given pointer will cause that data to not be logged.
 *
 * eco2 - pointer to eco2 reading to be logged
 * tvoc - pointer to tvoc reading to be logged
 * rh   - pointer to relative humidity reading to be logged
 * tc   - pointer to temperature reading to be logged (deg C)
*/
void logData(uint16_t *eco2, uint16_t *tvoc, float *rh, float *tc) {
	char buf[64];
	
	fd = SD.open(fileName, FILE_WRITE);
	
	if(eco2 != NULL) {
		// log eCO2 reading
		sprintf(buf, "eCO2: %d (ppm)\n", *eco2);
		fd.write(buf, strlen(buf));
		fd.flush();
	}
	
	if(tvoc != NULL) {
		// log TVOC reading
		sprintf(buf, "TVOC: %d\n", *tvoc);
		fd.write(buf, strlen(buf));
		fd.flush();
	}
	
	if(rh != NULL) {
		Serial.println(*rh);
		// log relative humidity reading
		sprintf(buf, "RH: %d %%\n", (int)*rh);
		fd.write(buf, strlen(buf));
		fd.flush();
	}
	
	if(tc != NULL) {
		Serial.println(*tc);
		// log temperature reading
		sprintf(buf, "Temp: %d (deg C)\n", (int)*tc);
		fd.write(buf, strlen(buf));
		fd.flush();
	}
	
	sprintf(buf, "==========\n");
	fd.write(buf);
	fd.flush();
	fd.close();
}