# PCB Test Suite

This folder contains a test suite of .elf files for verifying functionality of an assembled PCB. To run each test, simply apply power to the board and upload the .elf file using one of our programming tools.

## 01_blinky.elf

Blinks the error LED at 1 Hz, 50% duty cycle. 

**IMPORTANT:** Check that the processor fuse bits are set correctly in this test. The ATMega328P defaults to using the internal 8 MHz oscillator with a clock divider of 8, so if the clock select fuses haven't been updated the blink rate will be wrong. If necessary, use the programmer to update the clock select fuses. The low fuse byte should be programmed to the value 0xFD (no clock divider, 8 MHz oscillator, longest startup time).

## 02_devpwr.elf

Waits 3 seconds before turning on power to the peripherals. The device power LED should go on at this point. Waits another 2 seconds before blinking the error LED at 1 Hz, 50% duty cycle.

## 03_audio.elf

Turns on power to the peripherals and delays for 2 seconds. Takes audio samples and prints the FFT peak to Serial in an infinite loop. Use a USB-to-UART converter to view the values. Connect GND of the converter to pin 6 of the LoRa header (GND) and RX for the converter to LoRa Rx (pin 3 of the LoRa header).

## 04_dht.elf

Turns on power to the peripherals and delays for 2 seconds. Takes temperature/humidity samples from the DHT22 every second and prints them to Serial. View the output values using the same method as the audio test.

## 05_lora.elf

Turns on power to the peripherals and delays for 2 seconds. Configures the ADDRESS and NETWORKID of the LoRa, waiting for a +OK each time. Periodically sends a message containing the string "HELLO WORLD!" and verifies it gets back both the +OK and an acknowledgement containing the string "GK". This test requires a receiver LoRa module (could be on an Arduino) sending the appropriate acknowledgement messages. Ask Deepen for an Arduino sketch containing this code.

## 06_sd.elf

Turns on power to the peripherals and delays for 2 seconds. Initializes the SD card, opens a file called PCB_TEST.txt and writes "Hello World!" to it.

**IMPORTANT:** You cannot program the board while an SD card is inserted. To run this test, power on and program the board with the SD card **not** inserted. Then, power off the board, insert the SD card, and power the board on again.

## 07_ccs.elf

Coming soon.