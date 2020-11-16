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
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <string.h>
#include <arduinoFFT.h>

// Device Libraries
#include <Adafruit_CCS811.h>
#include <DHT.h>
#include <SD.h>
#include <LoRa.h>

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }
	
/* PINS */
// Debug Signals
#define DB1            6  // PD6
#define DB2            7  // PD7
#define DB3            8  // PB0
#define DB4           A0  // PC0
#define ERROR_LED      5  // PD5
// Sensors
#define SD_CS         10  // PB2
#define SD_CD          9  // PB1
// #define MIC_RAW        X  // ADC7 (TODO: Confirm ADC7 functionality with Arduino)
#define MIC_FILTERED  A1  // PC1
#define DHT_DATA       2  // PD2
#define CCS_RESET     A3  // PC3
// Power
#define DEV_PWR        3  // PD3

/* Debug Codes */
#define ERROR_NO_LORA        0x00
#define ERROR_CCS_INIT_FAIL  0x01
#define ERROR_SD_INIT_FAIL   0x02
#define ERROR_SD_OPEN_FAIL   0x03
#define GENERAL_DEBUG        0x04
#define ERROR_NO_ACK         0x05
#define ERROR_NO_CSS         0x06
#define ERROR_NO_DHT         0x07

/* FFT */
#define SAMPLES         64
#define SAMPLING_FREQ   6100
#define SAMPLING_PERIOD 1000000 / SAMPLING_FREQ

/* HELPER FUNCTIONS */
bool readCCS(uint16_t*, uint16_t*);
bool readDHT(float*, float*);
bool initSD();
double readAudio();

void debug_output(uint8_t opcode);
void trap_error();
void flash_led(uint8_t pin);
void gotosleep(uint8_t cycles);

/* DEVICES */
Adafruit_CCS811 ccs;
DHT dht(DHT_DATA, DHT22);
arduinoFFT FFT = arduinoFFT();

/* GLOBALS */
unsigned long t_start;
File fd;
bool sdBegan = false;       // library complains if begin() called twice
const int numReadings = 5;  // number of readings to take per wake-up cycle

// Watchdog Timer ISR
ISR (WDT_vect) {	
	// disable watchdog on expiry
	wdt_disable();
}

int main(void)
{
	uint8_t ret;
	
	init();

	/********** begin setup **********/
	
	Serial.begin(57600, SERIAL_8N1);
	Serial.setTimeout(2000);
	Serial.flush();

	// Pin Setup
	/* Debug Signals */
	pinMode(DB1, OUTPUT);
	pinMode(DB2, OUTPUT);
	pinMode(DB3, OUTPUT);
	pinMode(DB4, OUTPUT);
	pinMode(ERROR_LED, OUTPUT);
	/* Sensors */
	pinMode(SD_CS, OUTPUT);
	pinMode(SD_CD, INPUT);
	// pinMode(MIC_RAW, INPUT);
	pinMode(MIC_FILTERED, INPUT);
	pinMode(CCS_RESET, OUTPUT);
	/* Power */
	pinMode(DEV_PWR, OUTPUT);
	
	digitalWrite(CCS_RESET, HIGH);
	digitalWrite(DEV_PWR, HIGH);
	delay(1000);
	analogReference(EXTERNAL); // use 1.8V on AREF pin for analog reference
	if(!ccs.begin()) {
		debug_output(ERROR_CCS_INIT_FAIL);
		trap_error();
	}
	while(!ccs.available());
	
	dht.begin();
	
	/********** end setup **********/
	
    
	/********** begin main program loop **********/
	uint16_t eco2, tvoc, totalEco2, totalTvoc, numEco2Tvoc;
	float h, t, totalH, totalT, numHT;
	double fpeak, totalFpeak;
	
	size_t buf_size = 2*sizeof(uint16_t) + 2*sizeof(float);
	char *buf = (char*)malloc(buf_size);
	
	for (;;) {
		// power devices and give time for power up
		digitalWrite(DEV_PWR, HIGH);
		delay(1000);
		
		// set reset high for CCS on each wake-up
		digitalWrite(CCS_RESET, HIGH);
		if(!ccs.begin()) {
			debug_output(ERROR_CCS_INIT_FAIL);
			trap_error();
		}
		
		// delay 10 seconds for CSS warm-up
		delay(10000);
		
		// take measurements
		eco2 = tvoc = h = t = fpeak = 0;
		totalEco2 = totalTvoc = totalH = totalT = totalFpeak = 0;
		numEco2Tvoc = numHT = numReadings;
		for(int i = 0; i < numReadings; ++i) {
			ret = readCCS(&eco2, &tvoc);
			if(!ret) {
				debug_output(ERROR_NO_CSS);
				Serial.println("No CSS");
				numEco2Tvoc -= 1; // discard
			}
			else {
				Serial.println("CCS Readings");
				Serial.println(eco2);
				Serial.println(tvoc);
				totalEco2 += eco2;
				totalTvoc += tvoc;
			}
			ret = readDHT(&h, &t);
			delay(1000); // TODO: check if this is needed
			if(!ret) {
				debug_output(ERROR_NO_DHT);
				Serial.println("No DHT");
				numHT -= 1; // discard
			}
			else {
				Serial.println("DHT readings");
				Serial.println(h);
				Serial.println(t);
				totalH += h;
				totalT += t;
			}
			delay(1000);
		}
		if(!numHT) {
			++numHT;
		}
		if(!numEco2Tvoc) {
			++numEco2Tvoc;
		}
		h = totalH/numHT;
		t = totalT/numHT;
		eco2 = totalEco2/numEco2Tvoc;
		tvoc = totalTvoc/numEco2Tvoc;
		Serial.println("Averages");
		Serial.println(h);
		Serial.println(t);
		Serial.println(eco2);
		Serial.println(tvoc);
		
		// let serial print statements finish
		delay(1000);
		
		// power off devices
		digitalWrite(DEV_PWR, LOW);

		// sleep until next measurement
		// TODO: modify for true measurement frequency
		gotosleep(1);
	}
	
	free(buf);
	
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
			Serial.println("Error in reading"); // sensor error
			// add trap
			// look for error code in the library
			return false;
		}
	}
	
	Serial.println("No data to read"); // no data to read
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
	
	*h = dht.readHumidity(true); // force a new reading
	*t = dht.readTemperature(true); // force a new reading
	if(isnan(*h) || isnan(*t)) {
		// bad data read
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
		// TODO: What do we do if there is no SD card?
		while(!digitalRead(SD_CD));
		delay(250);
	}
	
	// begin() returns false no matter what if not the first call
	if(!SD.begin(SD_CS) && !sdBegan) {
		return false;
	}
	else {
		sdBegan = true;
	}
	
	return true;
}

/*
 * Takes SAMPLES audio samples from the ADC at frequency 1 / SAMPLING_PERIOD,
 * computes the FFT of the sample, and returns the peak of the spectrum.
*/
double readAudio() {
	// TODO: Averaging/curve smoothing?
	
	double *vReal = (double*)malloc(sizeof(double)*SAMPLES);
	double *vImag = (double*)malloc(sizeof(double)*SAMPLES);
	
	for(int i=0; i<SAMPLES; i++)
	{
		t_start = micros();
		vReal[i] = analogRead(MIC_FILTERED);
		vImag[i] = 0;
		while((unsigned long)(micros() - t_start) < SAMPLING_PERIOD);
	}
	
	FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
	FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD); /* Compute FFT */
	FFT.ComplexToMagnitude(vReal, vImag, SAMPLES); /* Compute magnitudes */
	double x = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQ);
	
	free(vReal);
	free(vImag);

	return x;
}

/*
 * Outputs a four-bit number on the debugging pins DB1:DB4
*/
void debug_output(uint8_t opcode) {
	digitalWrite(DB1, opcode & 0x01 ? HIGH : LOW);
	digitalWrite(DB2, opcode & 0x02 ? HIGH : LOW);
	digitalWrite(DB3, opcode & 0x04 ? HIGH : LOW);
	digitalWrite(DB4, opcode & 0x08 ? HIGH : LOW);
}

/*
 * Sets the ERROR_LED high and loops forever
*/
void trap_error() {
	digitalWrite(ERROR_LED, HIGH);
	while(1);
}

/*
 * Flashes an LED for 1 second
*/
void flash_led(uint8_t pin) {
	for(int i = 0; i < 5; ++i) {
		digitalWrite(pin, HIGH);
		delay(100);
		digitalWrite(pin, LOW);
		delay(100);
	}
}

/*
 * Causes the processor to sleep for a specified number of 8 second cycles
*/
void gotosleep(uint8_t cycles) {
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	
	for(int i = 0; i < cycles; ++i) {
		// clear reset flags
		MCUSR = 0;
		// allow changes to watchdog, disable reset mode
		WDTCSR = bit (WDCE) | bit (WDE);
		// enable interrupt mode and set time interval
		WDTCSR = bit (WDIE) | bit(WDP3) | bit (WDP0); // set WDIE, 8 sec delay
		// pat the dog
		wdt_reset();
		// go to sleep!
		sleep_cpu();
		// WDT ISR will return here
	}
	
	sleep_disable();	
}