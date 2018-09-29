#include <Arduino.h>

/*
 * getTFminiData: https://github.com/TFmini/TFmini-Arduino
 */

#include <Servo.h>
#include <math.h>

#define LIDAR_OFFSET 10*2.54 //cm
#define MAX_LIDAR_DISTANCE 1200 //cm
#define LIDAR_PIN 2 //pin for lidar servo
#define LASER_PIN 3 //pin for laser servo

#ifdef MOVE_LIDAR
#define LIDAR_MIN 45
#define NREADINGS 20
#else
#define LIDAR_MIN 90
#define NREADINGS 100
#endif /* MOVE_LIDAR */
#define LIDAR_MAX 90


Servo laser_servo;

void getTFminiData(int* distance, int* strength) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if(Serial.available()) {
    rx[i] = Serial.read();
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    } else {
      i++;
    }
  }
}


int rad2deg(float radians){
  return (int)(radians * (180.0/ M_PI));
}

//use law of cosines to calculate laser angle
int getLaserAngle(float lidardist, float lidaroffset, float lidarangle){
  return 0;
}

//use inverse tangent to calculate laser angle
int getLaserRightAngle(float lidardist, float lidaroffset){
  return rad2deg(atan2(lidardist,lidaroffset));
}

int lidar_theta;
int lidar_step;
int sweep_done;

int ctr = 0;
int angl_avg = 0;

void setup() {
  Serial.begin(115200);
  laser_servo.attach(3);
  //move the lidar to its initial position
  lidar_theta = LIDAR_MIN;
  lidar_step = 1;
  sweep_done = 0;
}


void getLidarDistance(int * distance, int * strength){
  getTFminiData(distance, strength);
  while(!*distance) {
    getTFminiData(distance, strength);
    if(*distance) {
      Serial.print(*distance);
      Serial.print("cm\t");
      Serial.print("strength: ");
      Serial.print(*strength);
    }
  }
}

int getAnglAvg(){
    int tmp;
    Serial.print("AVERAGE ANGLE: ");
    Serial.println(angl_avg/NREADINGS);
    tmp = angl_avg;
    angl_avg = 0;
    delay(100);
    return tmp;
}

void loop() {
  int distance = 0;
  int strength = 0;
  int laser_theta = 0;
  ctr++;
  getLidarDistance(&distance,&strength);
#ifdef MOVE_LIDAR
/* 2D case: move up and down looking for the boundaries of an object
 * Point at it after a sweep is done
 */
  angl_avg += laser_theta;
  lidar_theta+=lidar_step;
  moveServo(lidar_theta,LASER_PIN);
  sweep_done = (lidar_step > 0 && lidar_theta >= LIDAR_MAX) ||
               (lidar_step < 0 && lidar_theta <= LIDAR_MIN);

  if(sweep_done){
    lidar_step *= -1;
  }
  if(distance > MAX_LIDAR_DISTANCE || distance < 0){
    Serial.println();
    return;
  }
  if(ctr%NREADINGS == 0){

  }
#else
/* 1D case: find the distance of an object and point at it */
  if(distance > MAX_LIDAR_DISTANCE || distance < 0){
    Serial.println();
    return;
  }
  laser_theta = getLaserRightAngle(distance,LIDAR_OFFSET);
  Serial.print(" Angle: ");
  Serial.println(laser_theta);
  angl_avg += laser_theta;
  if (ctr%NREADINGS == 0){
    laser_theta = getAnglAvg();
    laser_servo.write(laser_theta);
  }
#endif /*MOVE_LIDAR*/
}
