#include <Arduino.h>

#include <Servo.h>
#include <math.h>
#include "TFmini.h"
#include "LightCtl.h"

#define MOVE_LIDAR
#define LIDAR_THETA_OFFSET 60
#define LASER_THETA_OFFSET 180
#define LIDAR_OFFSET 10*2.54 //cm
#define MAX_LIDAR_DISTANCE 120 //cm
#define LIDAR_PIN 5 //pin for lidar servo
#define LASER_PIN 6 //pin for laser servo
#define LASER_LIGHT_PIN 7

#ifdef MOVE_LIDAR
#define LIDAR_MIN 60
#define NREADINGS 5
#else
#define LIDAR_MIN 90
#define NREADINGS 10
#endif /* MOVE_LIDAR */
#define LIDAR_MAX 120


Servo laser_servo;
Servo lidar_servo;

int rad2deg(float radians){
  return (int)(radians * (180.0/ M_PI));
}

float deg2rad(int deg){
  return (deg * M_PI/180.0);
}

//use law of cosines to calculate laser angle
int getLaserAngle(float lidardist, float lidaroffset, float lidarangle){
  int lidar_dist_sq = lidardist*lidardist;
  int lidar_off_sq = lidaroffset*lidaroffset;
  float cos_lidar_angle = cos(deg2rad(lidarangle));
  float sin_lidar_angle = sin(deg2rad(lidarangle));
  //law of cosines: get the distance from the laser to the target
  int laser_dist_sq = lidar_dist_sq+lidar_off_sq -
      (2*lidardist*lidaroffset*cos_lidar_angle);
  //law of sines: get the angle from the
  float sin_laser_angle = lidardist*sin_lidar_angle/sqrt(laser_dist_sq);
  return rad2deg(asin(sin_laser_angle));
}

//use inverse tangent to calculate laser angle
int getLaserRightAngle(float lidardist, float lidaroffset){
  return rad2deg(atan2(lidardist,lidaroffset));
}

int lidar_theta;
int lidar_step;
int sweep_done;
int IN_OBJECT = 0;
int IN_THETA, OUT_THETA;
int IN_DIST;
int ctr = 0;
int angl_avg = 0;

void setup() {
  Serial.begin(115200);
  laser_servo.attach(LASER_PIN);
  lidar_servo.attach(LIDAR_PIN);
  pinMode(LASER_LIGHT_PIN,OUTPUT);
  lightctl_setup();
  //move the lidar to its initial position
  lidar_theta = LIDAR_MIN;
  lidar_step = 1;
  sweep_done = 0;
  lidar_servo.write(lidar_theta);
}

int getAnglAvg(){
    int tmp = angl_avg/NREADINGS;
    angl_avg = 0;
    return tmp;
}

void pointLaser(){
  //point the laser at the median angle of the object detected
  int lidar_mean_theta = (IN_THETA + OUT_THETA)/2;
  int laser_theta = getLaserAngle(IN_DIST, LIDAR_OFFSET, lidar_mean_theta);
  Serial.print("FIRING LASER AT ANGLE ");
  Serial.println(laser_theta);
  Serial.println();
  laser_servo.write(laser_theta);
  digitalWrite(LASER_LIGHT_PIN,HIGH);
  delay(1000);
  digitalWrite(LASER_LIGHT_PIN,LOW);
}

void loop() {
  int distance = 0;
  int strength = 0;
  ctr++;
  getLidarDistance(&distance,&strength);
#ifdef MOVE_LIDAR
/* 2D case: move up and down looking for the boundaries of an object
 * Point at it after a sweep is done
 */

  lidar_theta += lidar_step;
  lidar_servo.write(lidar_theta - LIDAR_THETA_OFFSET);
  sweep_done = (lidar_step > 0 && lidar_theta >= LIDAR_MAX) ||
               (lidar_step < 0 && lidar_theta <= LIDAR_MIN);

  if(sweep_done){
    lidar_theta = LIDAR_MIN;
    IN_OBJECT = 0;
    return;
  }

  if (distance <= 0){
    return;
  }

  light_ctl(distance);
  delay(20);
  if(!IN_OBJECT && distance < MAX_LIDAR_DISTANCE){
    IN_OBJECT = 1;
    Serial.print("ENTERING OBJECT at ");
    IN_THETA = lidar_theta;
    IN_DIST = distance;
    Serial.print(IN_DIST);
    Serial.print(", ");
    Serial.println(IN_THETA);
  } else if (IN_OBJECT && (sweep_done || distance >= MAX_LIDAR_DISTANCE)) {
    Serial.print("LEAVING OBJECT at ");
    IN_OBJECT = 0;
    OUT_THETA = lidar_theta;
    Serial.print(IN_DIST);
    Serial.print(", ");
    Serial.println(OUT_THETA);
    pointLaser();
  }

#else
  int laser_theta = 0;
/* 1D case: find the distance of an object and point at it */
  if(distance > MAX_LIDAR_DISTANCE || distance <= 0){
    return;
  }
  laser_theta = getLaserRightAngle(distance,LIDAR_OFFSET);
  angl_avg += laser_theta;
  if (ctr%NREADINGS == 0){
    laser_theta = getAnglAvg();
    Serial.print(distance);
    Serial.print("cm\t");
    Serial.print("strength: ");
    Serial.print(strength);
    Serial.print("AVERAGE ANGLE: ");
    Serial.println(laser_theta);
    laser_servo.write(laser_theta);
  }
#endif /*MOVE_LIDAR*/
}
