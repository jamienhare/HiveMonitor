#include "KickFFT.h"
#include <SD.h>

int microphonePin = A0;
int buttonPin = 2;
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int SD_CS = 8;
int SD_CD = 9; 
File f;

const uint16_t samples = 128;
const float samplingFrequency = 9500; //not over 10000

unsigned int sampling_period_us;
unsigned long microseconds;

int16_t input[samples];

//TMRpcm audio;

void setup() {
  pinMode(microphonePin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(SD_CD, INPUT);
                       
  Serial.begin(9600);
  analogReference(EXTERNAL);
  //audio.CSPin = SD_CS;

  //Serial.print("Initializing SD card...");

  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  if(!digitalRead(SD_CD)) {
    //Serial.println("No card detected. Waiting...");
    while(!digitalRead(SD_CD));
    delay(250);
  }
  
  if (!SD.begin(SD_CS)) {
      //Serial.println("initialization failed!");
      while (1);
  }
  //Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  f = SD.open("test1.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (f) {
      //Serial.print("File opened");
      f.close();
  } 
  else {
      //Serial.println("error opening file");
  }
}

void loop() {
   // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      //Serial.println("on");
      readAudio();
    } else {
      // if the current state is LOW then the button went from on to off:
      //Serial.println("off");
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
}



void readAudio() {
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      input[i] = analogRead(microphonePin);
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  
  //audio.startRecording("1.wav", 16000, A0);
  //audio.stopRecording("1.wav");

  
  uint32_t mag[samples] = {0};
  KickFFT<int16_t>::fft(samplingFrequency, 100, 500, samples, input, mag);

  uint16_t startIndex = 100/(samplingFrequency/samples);
  uint16_t endIndex = 500/(samplingFrequency/samples);
  uint32_t maxM = 0;
  double maxF = 0;

  for(uint16_t i = startIndex; i < endIndex; i++)
  {
    //Peak should be around 1.3 Hz or so
    if(maxM < mag[i]){
      maxM = mag[i];
      maxF = i*samplingFrequency/samples;
    }
      
  }
  
  f = SD.open("test1.txt", FILE_WRITE);
  f.println(maxF);
  f.close();

}
