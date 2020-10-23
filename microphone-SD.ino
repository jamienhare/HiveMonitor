//#include <TMRpcm.h>
#include "arduinoFFT.h"
#include <SD.h>

int microphonePin = A0;
int buttonPin = 2;
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int SD_CS = 8;
int SD_CD = 9; 
File f;

const uint16_t samples = 128;
const double samplingFrequency = 9500; //not over 10000

unsigned int sampling_period_us;
unsigned long microseconds;

arduinoFFT FFT = arduinoFFT();

double vReal[samples];
double vImag[samples];

//TMRpcm audio;

void setup() {
  pinMode(microphonePin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(SD_CD, INPUT);
                       
  Serial.begin(9600);
  analogReference(EXTERNAL);
  //audio.CSPin = SD_CS;

  //Serial.print("Initializing SD card...");

  if(!digitalRead(SD_CD)) {
    Serial.println("No card detected. Waiting...");
    while(!digitalRead(SD_CD));
    delay(250);
  }
  
  if (!SD.begin(SD_CS)) {
      Serial.println("initialization failed!");
      while (1);
  }
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  f = SD.open("test1.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (f) {
      Serial.print("File opened");
      f.close();
  } 
  else {
      Serial.println("error opening file");
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
      vReal[i] = analogRead(microphonePin);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  
  //audio.startRecording("1.wav", 16000, A0);
  //audio.stopRecording("1.wav");
  
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  
  f = SD.open("test1.txt", FILE_WRITE);
  f.println(x);
  f.close();

}
