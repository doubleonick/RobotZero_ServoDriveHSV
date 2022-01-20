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
#include <FastLED.h>
#include <Wire.h>
#include <ServoDriver.h> // Download latest here: https://github.com/TinyCircuits/TinyCircuits-TinyShield_Motor_Library/archive/master.zip
#include <Wireling.h>
#include "Adafruit_TCS34725.h"  // The library used for the Color Sensor Wireling

ServoDriver servo(15);// Value passed is the address- RobotZero is always address 15
#define BASE_SPEED  75
#define HALT_SPEED 1500
#define SerialMonitorInterface SerialUSB // for SAMD21 processors
#define NUM_COLOR_CH 3
#define NUM_COLORS 6
#define NUM_COLOR_SENSORS 3
#define CENTER_COLOR 0
#define LEFT_COLOR 1
#define RIGHT_COLOR 2

//Hue:
#define RED      0
#define GREEN    120
#define BLUE     240
#define CYAN     180
#define MAGENTA  300
#define YELLOW   60
#define COLOR_MARGIN 30

/* Initialise with specific int time and gain values.  50ms is fast enough for near-instantaneous reaction.*/
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
int tcsPorts[NUM_COLOR_SENSORS] = {0, 1, 2};

// Variables to hold the values the sensor reads
uint16_t c, colorTemp, lux;//, h, s, v;
uint16_t r[NUM_COLOR_SENSORS]; 
uint16_t g[NUM_COLOR_SENSORS]; 
uint16_t b[NUM_COLOR_SENSORS];
//float r[NUM_COLOR_SENSORS]; 
//float g[NUM_COLOR_SENSORS]; 
//float b[NUM_COLOR_SENSORS];
uint16_t h[NUM_COLOR_SENSORS];
float    s[NUM_COLOR_SENSORS];
uint16_t v[NUM_COLOR_SENSORS];


float rPrime, gPrime, bPrime;
float cMax, cMin, cDelta;
float RToG, RToB, GToR, GToB, BToR, BToG;
uint16_t hue; 
float saturation, value;

#define LED_PIN A3 // Corresponds to Wireling Port 0 (A1 = Port 1, A2 = Port 2, A3 = Port 3)

#define NUM_LEDS 3 // This is the number of RGB LEDs connected to the pin
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
CRGB RED_LED     = CRGB( 128, 0, 0);
CRGB GREEN_LED   = CRGB( 0, 128, 0);
CRGB BLUE_LED    = CRGB( 0, 0, 128);
CRGB CYAN_LED    = CRGB( 0, 128, 128);
CRGB MAGENTA_LED = CRGB( 128, 0, 128);
CRGB YELLOW_LED  = CRGB( 128, 128, 0);
CRGB WHITE_LED   = CRGB( 255, 255, 255);
CRGB BLACK_LED   = CRGB( 0, 0, 0);
int brightness = 64; // Brightness is on a scale of 0-255, 128 is 50% brightness
int colorDelay = 3000;

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

    r[port] = 0;
    g[port] = 0;
    b[port] = 0;
    
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

  // Initialize RGB LEDs attached to LED_PIN
  FastLED.addLeds<WS2812, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(brightness);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  int left_speed, right_speed, duration;
  ReadColorSensor(0);
  //delay(500);
  ReadColorSensor(1);
  //delay(500);
  ReadColorSensor(2);
  //delay(500);
  //drive(HALT_SPEED + BASE_SPEED, HALT_SPEED - BASE_SPEED);
  LineFollow(&left_speed, &right_speed, &duration);
  drive(left_speed, right_speed, duration);
  FastLED.show();   

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
  SerialMonitorInterface.print("Port: "); SerialMonitorInterface.println(port);
  Wireling.selectPort(port);
//  LEDon();
  tcs.getRawData(&r[port], &g[port], &b[port], &c);
//  colorTemp = tcs.calculateColorTemperature(r, g, b);
//  lux = tcs.calculateLux(r, g, b);

  RGBToHSV(port);
//  String colorNow = IdentifyColor(port);
//  SerialMonitorInterface.print("COLOR: "); SerialMonitorInterface.println(colorNow);
//  LEDoff();
}

void RGBToHSV(byte port)
{
  float r_ch, g_ch, b_ch;
  SerialMonitorInterface.print("R0: "); SerialMonitorInterface.print(r[port], DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("G0: "); SerialMonitorInterface.print(g[port], DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("B0: "); SerialMonitorInterface.println(b[port]);
  //Constrain between 0 and 255, but keep proportions
  if(r[port] > 255 || g[port] > 255 || b[port] > 255)
  {
    r_ch = (float)r[port];
    g_ch = (float)g[port];
    b_ch = (float)b[port];
    if(r[port] >= g[port] && r[port] >= b[port])
    {  
       r[port] = constrain(r_ch, 0, 255.0);
       g[port] = constrain(g_ch, 0, 255.0 * (g_ch/r_ch));
       b[port] = constrain(b_ch, 0, 255.0 * (b_ch/r_ch));
    }else if(g[port] >= r[port] && g[port] >= b[port])
    {       
       r[port] = constrain(r_ch, 0, 255.0 * (r_ch/g_ch));
       g[port] = constrain(g_ch, 0, 255.0);
       b[port] = constrain(b_ch, 0, 255.0 * (b_ch/g_ch));
    }else if(b[port] >= r[port] && b[port] >= g[port])
    {
       r[port] = constrain(r_ch, 0, 255.0 * (r_ch/b_ch));
       g[port] = constrain(g_ch, 0, 255.0 * (g_ch/b_ch));
       b[port] = constrain(b_ch, 0, 255.0);
    }else
    {
       r[port] = constrain(r_ch, 0, 255.0);
       g[port] = constrain(g_ch, 0, 255.0);
       b[port] = constrain(b_ch, 0, 255.0);
    }
    
  }

  SerialMonitorInterface.print("R: "); SerialMonitorInterface.print(r[port], DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("G: "); SerialMonitorInterface.print(g[port], DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("B: "); SerialMonitorInterface.println(b[port]); 

  rPrime = (float) r[port] / 255.0;
  gPrime = (float) g[port] / 255.0;
  bPrime = (float) b[port] / 255.0;

  cMax = max(rPrime, gPrime);
  cMax = max(cMax, bPrime);
  cMin = min(rPrime, gPrime);
  cMin = min(cMin, bPrime);
  cDelta = cMax - cMin;

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
    if(cDelta == 0)
    {
      hue = 0;
    }else
    {
      hue = 60 * fmod(fabs((gPrime - bPrime)/cDelta), 6.0);
    }
  }else if(cMax == gPrime)
  {
    hue = 60 * (((bPrime - rPrime)/cDelta) + 2.0);
  }else if(cMax == bPrime)
  {
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

//  SerialMonitorInterface.print("H: "); SerialMonitorInterface.print(h[port], DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("S': "); SerialMonitorInterface.print(s[port], DEC); SerialMonitorInterface.print(", ");
//  SerialMonitorInterface.print("V': "); SerialMonitorInterface.println(v[port]);
}

String IdentifyColor(byte port)
{
  /*
   * RED      0
   * GREEN    120
   * BLUE     240
   * CYAN     180
   * MAGENTA  300
   * YELLOW   60
   */
  String color = "UNKNOWN";
  hue = h[port];

  //Red: hue <= 0 + color margin OR hue >= 360 - color margin
  //GREEN: 90 <= hue <= 150
  //BLUE:  210 <= hue <= 270
  //CYAN:  150 <= hue <= 210
  //MAGENtA: 270 <= hue <= 330
  //YELLOW:  30 <= hue <= 90
  
  if(hue <= RED + COLOR_MARGIN || hue >= 360 - COLOR_MARGIN)
  {
    color = "RED";
    leds[port] = CRGB(128, 0, 0);
  }else if(hue >= GREEN - COLOR_MARGIN && hue <= GREEN + COLOR_MARGIN)
  {
    color = "GREEN";
    leds[port] = CRGB(0, 128, 0);
  }else if(hue >= BLUE - COLOR_MARGIN && hue <= BLUE + COLOR_MARGIN)
  {
    color = "BLUE";
    leds[port] = CRGB(0, 0, 128);
  }else if(hue >= CYAN - COLOR_MARGIN && hue <= CYAN + COLOR_MARGIN)
  {
    color = "CYAN";
    leds[port] = CRGB(0, 128, 128);
  }else if(hue >= MAGENTA - COLOR_MARGIN && hue <= MAGENTA + COLOR_MARGIN)
  {
    color = "MAGENTA";
    leds[port] = CRGB(128, 0, 128);
  }else if(hue >= YELLOW - COLOR_MARGIN && hue <= YELLOW + COLOR_MARGIN)
  {
    color = "YELLOW";
    leds[port] = CRGB(128, 128, 0);
  }

  return color;
}
void LineFollow(int * servo_left_power, int * servo_right_power, int * duration)
{
  uint16_t white[NUM_COLOR_CH] = {68, 68, 68};//{180, 180, 180};
  uint16_t black[NUM_COLOR_CH] = {60, 60, 60};//{75, 75, 75};
  //C = Black....
  //L = Black: pivot R
  //R = Black: pivot L
  //just C = Black, FWD
  //C = White....
  //L = Black, R = Black: pivot L
  //L = White, R = White: move FWD
  //L = Black, R = White: pivot L
  //L = White, R = Black: pivot R
  if((r[CENTER_COLOR] <= black[0] && g[CENTER_COLOR] <= black[1] && b[CENTER_COLOR] <= black[2]))
  {
    if((r[LEFT_COLOR] <= black[0] && g[LEFT_COLOR] <= black[1] && b[LEFT_COLOR] <= black[2]) && 
       (r[RIGHT_COLOR] >= white[0] && g[RIGHT_COLOR] >= white[1] && b[RIGHT_COLOR] >= white[2]) )
    {
      SerialMonitorInterface.println("Black : Black : White... Pivot R");
      *servo_left_power  = HALT_SPEED + BASE_SPEED;
      *servo_right_power = HALT_SPEED + BASE_SPEED;
      leds[1] = WHITE_LED;
      leds[2] = BLACK_LED;
      *duration = 500;
    }else if((r[LEFT_COLOR] >= white[0] && g[LEFT_COLOR] >= white[1] && b[LEFT_COLOR] >= white[2]) &&
             (r[RIGHT_COLOR] <= black[0] && g[RIGHT_COLOR] <= black[1] && b[RIGHT_COLOR] <= black[2]) )
    {
      SerialMonitorInterface.println("White  : Black : Black... Pivot L");
      *servo_left_power  = HALT_SPEED - BASE_SPEED;
      *servo_right_power = HALT_SPEED - BASE_SPEED;
      leds[1] = BLACK_LED;
      leds[2] = WHITE_LED;
      *duration = 500;
    }else
    {
      SerialMonitorInterface.println("Center Black, move FWD");
      *servo_left_power  = HALT_SPEED + BASE_SPEED;
      *servo_right_power = HALT_SPEED - BASE_SPEED;
      leds[1] = WHITE_LED;
      leds[2] = WHITE_LED;
      *duration = 5;
    }
  }else if((r[CENTER_COLOR] >= white[0] && g[CENTER_COLOR] >= white[1] && b[CENTER_COLOR] >= white[2]))
  {
    if((r[LEFT_COLOR] <= black[0] && g[LEFT_COLOR] <= black[1] && b[LEFT_COLOR] <= black[2]) && 
       (r[RIGHT_COLOR] <= black[0] && g[RIGHT_COLOR] <= black[1] && b[RIGHT_COLOR] <= black[2]) ) 
    {
      SerialMonitorInterface.println("Black  : White :  Black... Pivot R");
      *servo_left_power  = HALT_SPEED + BASE_SPEED;
      *servo_right_power = HALT_SPEED + BASE_SPEED;
      leds[1] = BLACK_LED;
      leds[2] = BLACK_LED;
      *duration = 500;
    }else if((r[LEFT_COLOR] >= white[0] && g[LEFT_COLOR] >= white[1] && b[LEFT_COLOR] >= white[2]) && 
             (r[RIGHT_COLOR] >= white[0] && g[RIGHT_COLOR] >= white[1] && b[RIGHT_COLOR] >= white[2]) )
    {
      SerialMonitorInterface.println("White  :  White  :  White... FWD");
      *servo_left_power  = HALT_SPEED + BASE_SPEED;
      *servo_right_power = HALT_SPEED - BASE_SPEED;
      leds[1] = WHITE_LED;
      leds[2] = WHITE_LED;
      *duration = 5;
    }else if((r[LEFT_COLOR] <= black[0] && g[LEFT_COLOR] <= black[1] && b[LEFT_COLOR] <= black[2]) && 
             (r[RIGHT_COLOR] >= white[0] && g[RIGHT_COLOR] >= white[1] && b[RIGHT_COLOR] >= white[2]) )
    {
      SerialMonitorInterface.println("Black  :  White  :  White... Pivot L");
      *servo_left_power  = HALT_SPEED - BASE_SPEED;
      *servo_right_power = HALT_SPEED - BASE_SPEED;
      leds[1] = BLACK_LED;
      leds[2] = WHITE_LED;
      *duration = 500;
    }else if((r[LEFT_COLOR] >= white[0] && g[LEFT_COLOR] >= white[1] && b[LEFT_COLOR] >= white[2]) &&
             (r[RIGHT_COLOR] <= black[0] && g[RIGHT_COLOR] <= black[1] && b[RIGHT_COLOR] <= black[2]) )
    {
      SerialMonitorInterface.println("White  :  White  :  Black... Pivot R");
      *servo_left_power  = HALT_SPEED + BASE_SPEED;
      *servo_right_power = HALT_SPEED + BASE_SPEED;
      leds[1] = WHITE_LED;
      leds[2] = BLACK_LED;
      *duration = 500;
    }else
    {
      SerialMonitorInterface.println("Center White... FWD");
      *servo_left_power  = HALT_SPEED + BASE_SPEED;
      *servo_right_power = HALT_SPEED - BASE_SPEED;
      leds[1] = BLUE_LED;
      leds[2] = BLUE_LED;
      *duration = 5;
    }
  }else
  {
    SerialMonitorInterface.print("Center: ");SerialMonitorInterface.print(r[CENTER_COLOR]);SerialMonitorInterface.print(", ");SerialMonitorInterface.print(g[CENTER_COLOR]);SerialMonitorInterface.print(", ");SerialMonitorInterface.println(b[CENTER_COLOR]);
  }
  SerialMonitorInterface.println("LineFollow...");
  SerialMonitorInterface.print("left_speed = ");SerialMonitorInterface.print(*servo_left_power);SerialMonitorInterface.print(", right_speed = ");SerialMonitorInterface.print(*servo_right_power);SerialMonitorInterface.print(", duration = ");SerialMonitorInterface.println(*duration);
}

void drive(int servo_left_power, int servo_right_power, int duration)
{
  //Forward is left > 1500, right < 1500.  1500 is full stop.
  //Servo 1 is the Left
  //Servo 4 is the Right
  int servo_left_port  = 1;
  int servo_right_port = 2;

  servo_left_power = servo_left_power * 0.97;

  servo.setServo(servo_left_port, servo_left_power);
  servo.setServo(servo_right_port, servo_right_power);
  delay(duration);

}
