#include <Arduino.h>
#include "Adafruit_NeoPixel.h"

#define PIN 8
//Connect NeoPixel D_in to output pin 6 on Arduino
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);

void lightctl_setup() {
  // put your setup code here, to run once:
  strip.begin();
  strip.show();

}

void light_ctl(int distance) {
  // put your main code here, to run repeatedly:
//int distance = 74; //for testing purposes

//while(distance >= 0) {
  //delay(100);
  //distance--;
strip.clear();
if(distance > 100){
  for(int i = 0; i < distance/100; i += 1){
  strip.setPixelColor(i, 0, 0, 10);
  strip.setPixelColor(distance/100, 0, 0, distance%100 * 0.1);
  strip.show();
  }
}

if ((100 >= distance) && (distance >= 80)){
  strip.setPixelColor(7, 0, 0, 10);
  strip.setPixelColor(6, 0, 0, 10);
  strip.setPixelColor(1, 0, 10, 0);
  strip.setPixelColor(0, 0, 10, 0);
  strip.show();
}

if ((80 > distance) && (distance > 10 )){
for(int i = 0; i < distance/10; i += 1){
  strip.setPixelColor(i, 0, 10, 0);
  strip.setPixelColor(distance/10, 0, distance%10 , 0);
  strip.show();
  }
}

if ((10 >= distance) && (distance >= 8)){
  strip.setPixelColor(7, 0, 10, 0);
  strip.setPixelColor(6, 0, 10, 0);
  strip.setPixelColor(1, 10, 0, 0);
  strip.setPixelColor(0, 10, 0, 0);
  strip.show();
}

if (8 > distance){
for(int i = 0; i <= distance; i += 1){
  strip.setPixelColor(i, 10, 0, 0);
  strip.show();
  }
//}
}
}
