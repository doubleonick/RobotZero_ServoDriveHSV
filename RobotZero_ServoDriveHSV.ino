/**********************************************************************
 * RobotZero Peripheral Basics
 * This program illustrates the abilities of the RobotZero processor 
 * board by spinning any attached motors or servos, while reading the
 * accelerometer data of the onboard 9-Axis sensor and Color
 * Sensor data for a Color Sensor Wireling attached to port 0. 
 * 
 * NOTES: 
 *   - Serial Monitor must be open for program to run.
 *   - Battery must be plugged in to power motors.
 * 
 * Hardware by: TinyCircuits
 * Written by: Ben Rose & Laveréna Wienclaw for TinyCircuits
 * 
 * Initialized: June 2019
 * Last modified: Jan 2020
 **********************************************************************/

#include <Wire.h>
#include <ServoDriver.h> // Download latest here: https://github.com/TinyCircuits/TinyCircuits-TinyShield_Motor_Library/archive/master.zip
#include <Wireling.h>
#include "Adafruit_TCS34725.h"  // The library used for the Color Sensor Wireling

ServoDriver servo(15);// Value passed is the address- RobotZero is always address 15
#define BASE_SPEED  80
#define HALT_SPEED 1500
#define SerialMonitorInterface SerialUSB // for SAMD21 processors
#define NUM_COLOR_CH 3
#define NUM_COLORS 6
#define NUM_COLOR_SENSORS 3
/* Initialise with specific int time and gain values.  50ms is fast enough for near-instantaneous reaction.*/
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
int tcsPorts[NUM_COLOR_SENSORS] = {0, 1, 2};

// Variables to hold the values the sensor reads
uint16_t r, g, b, c, colorTemp, lux;//, h, s, v;
uint16_t h[NUM_COLOR_SENSORS];
uint16_t s[NUM_COLOR_SENSORS];
uint16_t v[NUM_COLOR_SENSORS];

float rPrime, gPrime, bPrime;
float cMax, cMin, cDelta;
float RToG, RToB, GToR, GToB, BToR, BToG;
uint16_t hue, saturation, value;

void setup() {
  SerialMonitorInterface.begin(9600);
  Wire.begin();
  Wireling.begin();

  //The port is the number on the Adapter board where the sensor is attached
//  Wireling.selectPort(0);
  for (byte port = 0; port < NUM_COLOR_SENSORS; port++) {
    Wireling.selectPort(tcsPorts[port]);
    SerialMonitorInterface.print("Port ");
    SerialMonitorInterface.print(port);

    if (tcs.begin()) {
      SerialMonitorInterface.println("Found sensor");
    } else {
      SerialMonitorInterface.println("No TCS34725 found ... check your connections");
      while (1);
    }
  
    // Turn Wireling LEDs on 
    LEDon();

    cMax = 0.0;
    cMin = 255.0;
    
    h[port] = 0;
    s[port] = 0;
    v[port] = 0;
  }

  // Initialize servo driver
  if(servo.begin(20000)){
    while(1);
    SerialMonitorInterface.println("Servo driver not detected!");
  }

  Wireling.selectPort(0);
}

void loop() {
  ReadColorSensor(0);
  //delay(500);
//  ReadColorSensor(1);
  //delay(500);
//  ReadColorSensor(2);
  //delay(500);

  if((h[0] >= 0 && h[0] <= 10) || (h[0] >= 340 && h[0] <= 360))
  {
    drive(1500, 1500);
  }else
  {
    drive(HALT_SPEED + BASE_SPEED, HALT_SPEED - BASE_SPEED);
  }
  
//  delay(500);

}

// Turn Wireling LEDs on
void LEDon() {
  tcs.setInterrupt(true);
}

// Turn Wireling LEDs off
void LEDoff() {
  tcs.setInterrupt(false);
}

void ReadColorSensor(byte port)
{
//  SerialMonitorInterface.print("Color sensor on port, "); SerialMonitorInterface.println(port);
//  LEDon();
//  Wireling.selectPort(port);
  tcs.getRawData(&r, &g, &b, &c);
//  colorTemp = tcs.calculateColorTemperature(r, g, b);
//  lux = tcs.calculateLux(r, g, b);

  RGBToHSV(port);
//  LEDoff();
}

void RGBToHSV(byte port)
{
  //Constrain between 0 and 255, but keep proportions
  if(r > 255 || g > 255 || b > 255)
  {
    if(r > g && r > b)
    {
//       GToR = (float) g/r;
//       BToR = (float) b/r;
       
       r = constrain(r, 0, 255.0);
       g = constrain(g, 0, 255.0 * (g/r));
       b = constrain(b, 0, 255.0 * (b/r));
    }else if(g > r && g > b)
    {
//       RToG = (float) r/g;
//       BToG = (float) b/g;
       
       r = constrain(r, 0, 255.0 * (r/g));
       g = constrain(g, 0, 255.0);
       b = constrain(b, 0, 255.0 * (b/g));
    }else if(b > r && b > g)
    {
//       RToB = (float) r/b;
//       GToB = (float) g/b;
       
       r = constrain(r, 0, 255.0 * (r/b));
       g = constrain(g, 0, 255.0 * (g/b));
       b = constrain(b, 0, 255.0);
    }
    
  }

//  SerialMonitorInterface.print("R: "); SerialMonitorInterface.print(r, DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("G: "); SerialMonitorInterface.print(g, DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("B: "); SerialMonitorInterface.println(b); 

  rPrime = (float) r / 255.0;
  gPrime = (float) g / 255.0;
  bPrime = (float) b / 255.0;

//  SerialMonitorInterface.print("R': "); SerialMonitorInterface.print(rPrime, DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("G': "); SerialMonitorInterface.print(gPrime, DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("B': "); SerialMonitorInterface.println(bPrime);

  cMax = max(rPrime, gPrime);
  cMax = max(cMax, bPrime);
  cMin = min(rPrime, gPrime);
  cMin = min(cMin, bPrime);
  cDelta = cMax - cMin;

//  SerialMonitorInterface.print("cMax: "); SerialMonitorInterface.print(cMax, DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("cMin: "); SerialMonitorInterface.print(cMin, DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("cDelta: "); SerialMonitorInterface.println(cDelta);
//  SerialMonitorInterface.println(" ");

  //Hue:
  //RED..... 0
  //GREEN... 120
  //BLUE.... 240
  //CYAN.... 180
  //MAGENTA. 300
  //YELLOW.. 60
  if(cDelta == 0)
  {
    hue = 0;
  }else if(cMax == rPrime)
  {
//    SerialMonitorInterface.print("gPrime - bPrime = "); SerialMonitorInterface.println(gPrime - bPrime); 
//    SerialMonitorInterface.print("(gPrime - bPrime)/cDelta = "); SerialMonitorInterface.println((gPrime - bPrime)/cDelta);
//    SerialMonitorInterface.print("fmod(((gPrime - bPrime)/cDelta), 6.0) = "); SerialMonitorInterface.print(fmod(((gPrime - bPrime)/cDelta), 6.0)); 
//    SerialMonitorInterface.print("h = 60 * fmod(((gPrime - bPrime)/cDelta), 6.0) = "); SerialMonitorInterface.println(60 * fmod(((gPrime - bPrime)/cDelta), 6.0));
    if(cDelta == 0)
    {
      hue = 0;
    }else
    {
      hue = 60 * fmod(fabs((gPrime - bPrime)/cDelta), 6.0);
    }
  }else if(cMax == gPrime)
  {
    SerialMonitorInterface.println("h = 60 * (((bPrime - rPrime)/cDelta) + 2.0);");
    hue = 60 * (((bPrime - rPrime)/cDelta) + 2.0);
  }else if(cMax == bPrime)
  {
    SerialMonitorInterface.println("h = 60 * (((rPrime - gPrime)/cDelta) + 4.0);");
    hue = 60 * (((rPrime - gPrime)/cDelta) + 4.0);
  }

  //Saturation
  if(cMax == 0)
  {
    saturation = 0.0;
  }else{
    saturation = fabs(cDelta/cMax);
  }

  //Value
  value = cMax;

//  SerialMonitorInterface.print("H: "); SerialMonitorInterface.print(hue, DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("S': "); SerialMonitorInterface.print(saturation, DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("V': "); SerialMonitorInterface.println(value);

  h[port] = hue;
  s[port] = saturation;
  v[port] = value;
}

void drive(int servo_left_power, int servo_right_power)
{
  //Forward is left > 1500, right < 1500.  1500 is full stop.
  //Servo 1 is the Left
  //Servo 4 is the Right
  int servo_left_port  = 1;
  int servo_right_port = 2;

  servo.setServo(servo_left_port, servo_left_power);
  servo.setServo(servo_right_port, servo_right_power);
  delay(10);

}
