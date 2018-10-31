#include <EEPROM.h>
#include "FastLED.h"
#include <NXPMotionSense.h>
#include <Wire.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

//Audio Sample coverted file
// to change your file name here: 
#include "AudioSampleAteststring1.h"
#include "AudioSampleS16bitmonow.h"
#include "AudioSampleS50p16bit2w.h"
#include "AudioSampleS11025wa.h"



//prop shield Amplifier
#define PROP_AMP_ENABLE    5
//#define FLASH_CHIP_SELECT  6


unsigned int address = 0;
byte mode;
String TIME;
boolean iniStatus = false;
int frame = 0;
//================= LIGHT ========================
// How many leds in your strip?
#define NUM_LEDS 4

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
//#define CLOCK_PIN 13
//elapsedMillis Timer;
// Define the array of leds
boolean LightSwitchMode = false;
CRGB leds[NUM_LEDS];
float lightPow;
//boolean red = false;

//================= Audio ========================
// GUItool: begin automatically generated code
AudioSynthWaveformSine   sine2;          //xy=439,268
AudioSynthWaveformSine   sine1;          //xy=445,225
AudioSynthNoiseWhite     noise1;         //xy=447,361
//AudioSynthWaveformSine   sine3;      //xy=451,312
AudioPlayMemory          playMem1; 
AudioMixer4              mixer1;         //xy=679,299
AudioOutputAnalog        dac1;           //xy=992,286
AudioConnection          patchCord2(sine1, 0, mixer1, 0);
AudioConnection          patchCord1(sine2, 0, mixer1, 1);
AudioConnection          patchCord3(noise1, 0, mixer1, 2);
AudioConnection          patchCord4(playMem1, 0, mixer1, 3);
AudioConnection          patchCord5(mixer1, dac1);

// GUItool: end automatically generated code
//synth part
float baseVol = 0.2;
boolean noiseEnable = false;
float noiseAmp = 0.1;
boolean blipEnable = false;
boolean toneEnable = false;
boolean samplePlayerMode = true;
int tone1 = 622;
int tone2 = 466;
int tone3 = 0;
float tone1Amp = 0.15;
float tone2Amp = 0.3;
float tone3Amp = 0.2;

//sampling playing
bool SamplePlaying = false;

int VolumeKnob;
/*

  int tone1 = 931;
  int tone2 = 932;
  int tone3 = 933;

  float tone1Amp = 0.9;
  float tone2Amp = 0.9;
  float tone3Amp = 0.9;
*/

//369.99 466.16 622.25 554.37 739.99 830.61 932.33



//================ Motion ========================
NXPMotionSense imu;
NXPSensorFusion filter;

int intensityThreshold = 30;
boolean smoothEnable = true;
const int numSmooth = 10;
float smooth[numSmooth];
int datIndex = 0;
int total = 0;
float average = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.begin(9600);

  
    //---- light swtich mode ------  (comment if always ON) 
    if (LightSwitchMode){
        
        mode = EEPROM.read(address);
        if (mode == 1) {
          TIME = "lightOn";
          EEPROM.write(address, 0);
        } else if (mode != 1) {
          TIME = "lightOff";
          EEPROM.write(address, 1);
        }
      
    } else {
      TIME = "lightOn" ;
    }
   /*---end of light set up--*/

    
  

  //intitialize smooth
  for (int thisDat = 0; thisDat < numSmooth; thisDat++) {
    smooth[thisDat] = 0;
  }

  // Initialize IMU and filter
  imu.begin();
  filter.begin(100);

  //initialize LED
  FastLED.addLeds<APA102, BGR>(leds, NUM_LEDS);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);  // enable access to LEDs

  // Turn on amp
  AudioMemory(20);
  dac1.analogReference(INTERNAL); // much louder! if external
  delay(50);             // time for DAC voltage stable
  pinMode(PROP_AMP_ENABLE, OUTPUT);
  digitalWrite(PROP_AMP_ENABLE, HIGH); // turn on the amplifier
  delay(10);             // allow time to wake up

  delay(2000);      // sanity check delay


}

void loop() {
  // put your main code here, to run repeatedly:
  if (iniStatus == false) {
   // Serial.println(TIME);
    //========light check===========
    if (TIME == "lightOff") {
      for (int n = 0; n < NUM_LEDS; n++) {

        leds[n].red = 0;
        leds[n].green = 0;
        leds[n].blue = 0;

        FastLED.show();
        delay(100);
      }
    }
    //=============end=============
    iniStatus = true;
  }

  //================= Volume knob ==================
  //  VolumeKnob = analogRead(11);
  // Serial.print("volume knob is: ");
  //Serial.println(VolumeKnob);

  //================= MOTION =======================
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch;
  uint8_t hue, val;
  long freq;
  float vol;
  float intensity;
  // Read motion sensors and filter the results

  imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  roll = filter.getRoll();
  pitch = filter.getPitch();
  //Serial.println("pingping");
  /*

    Serial.print("roll:");
    Serial.println(roll);

    Serial.print("pitch:");
    Serial.println(pitch);
  */
  intensity = sqrt(sq(sqrt(sq(gx) + sq(gy))) + sq(gz));

  if (smoothEnable == true) {
    total = total - smooth[datIndex];
    smooth[datIndex] = intensity;
    total = total + smooth[datIndex];
    datIndex = datIndex + 1;

    if (datIndex >= numSmooth) {
      datIndex = 0;
    }

    average = total / numSmooth;
    intensity = average;
  }
  if (intensity < 0) {
    intensity = 0;
  }
  //================= LIGHT ========================
   if (TIME == "lightOn") {
    lightPow = floatMap(intensity, 0, intensityThreshold, 0, 255);
    for (int n = 0; n < NUM_LEDS; n++) {
  
      leds[n].red = lightPow;
      leds[n].green = random(50, 170) + lightPow;
      leds[n].blue = 15 + lightPow;
      //Serial.println(intensity);
      FastLED.show();
      //delay(10);
    }

  }
  //================= Audio ========================


  freq = floatMap(intensity, 0, intensityThreshold, 932, 700);

  vol = baseVol + floatMap(intensity, 0, intensityThreshold, 0.0, 1 - baseVol);

  if (vol > 1) {
    vol = 1;
  }
  // Set frequency and volume for sound
  vol = sin(vol * PI / 2);
  //vol = log(vol)/4 + 1;
  if (vol <= 0) {
    vol = 0;
  }

  //Serial.println(intensity);
 // Serial.print(frame);
  Serial.print("intensity : ");
  Serial.print(intensity);
  Serial.print(" , Vol : ");
  Serial.println(vol);
 // frame++;
  if (toneEnable == true) {
    mixer1.gain(0, vol);
    mixer1.gain(1, vol);
    //mixer1.gain(2, vol);
    sine1.frequency(tone1);
    sine1.amplitude(tone1Amp);
    sine2.frequency(tone2);
    sine2.amplitude(tone2Amp);
    //sine3.frequency(tone3);
    //sine3.amplitude(tone3Amp);
  }
  if (blipEnable == true) {
    mixer1.gain(0, vol * 0.4);
    mixer1.gain(1, vol * 0.4);
    //mixer1.gain(2, vol * 0.3);
    sine1.frequency(freq);
    sine1.amplitude(1);
    sine2.frequency(freq + intensity * 0.3);
    sine2.amplitude(1);
    //  sine3.frequency(freq - intensity * 0.3);
    //sine3.amplitude(1);
  }

  if (noiseEnable == true) {
    mixer1.gain(2, vol);
    noise1.amplitude(noiseAmp);
  }

  // Sample playing 
  if (samplePlayerMode == true) {
    mixer1.gain(3, vol);
  
    if (playMem1.isPlaying() == false) {
      //playMem1.play(AudioSampleAteststring1);
      //playMem1.play(AudioSampleS16bitmonow);
      //playMem1.play(AudioSampleS50p16bit2w);
      playMem1.play(AudioSampleS11025wa);
      
      
      
    }
  }
    
  delay(20);

}

// Function to map a flaot number
float floatMap(float x,
               float inMin,
               float inMax,
               float outMin,
               float outMax) {

  // Set bounds
  if ( x < inMin ) x = inMin;
  if ( x > inMax ) x = inMax;

  return (x - inMin) * (outMax - outMin) /
         (inMax - inMin) + outMin;
}


