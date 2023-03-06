#include <Arduino.h>
#include <M5Atom.h>
#include "rc.hpp"
#include "sensor.hpp"

#define FR_MOTOR 19
#define FL_MOTOR 22
#define RR_MOTOR 33
#define RL_MOTOR 23
#define RESOLUTION_BITS 8
#define FR_CH 0
#define FL_CH 1
#define RR_CH 2
#define RL_CH 3
#define PWM_FREQ 300000

#define WHITE 0xffffff
#define BLUE 0x0000ff
#define RED 0xff0000
#define GREEN 0x00ff00
#define YELLOW 0xffff00
#define PERPLE 0xff00ff

#define CTRL_LIMIT (0.7f)
#define UGAIN (250.0f)

void setup() {
  // put your setup code here, to run once:
  M5.begin(true, false, true);
  ledcSetup(FR_CH, PWM_FREQ, RESOLUTION_BITS);
  ledcSetup(FL_CH, PWM_FREQ, RESOLUTION_BITS);
  ledcSetup(RR_CH, PWM_FREQ, RESOLUTION_BITS);
  ledcSetup(RL_CH, PWM_FREQ, RESOLUTION_BITS);
  ledcAttachPin(FR_MOTOR, FR_CH);
  ledcAttachPin(FL_MOTOR, FL_CH);
  ledcAttachPin(RR_MOTOR, RR_CH);
  ledcAttachPin(RL_MOTOR, RL_CH);
  ledcWrite(FR_CH, 0);
  ledcWrite(FL_CH, 0);
  ledcWrite(RR_CH, 0);
  ledcWrite(RL_CH, 0);
  rc_init();
  sensor_init();
  Serial.begin(115300);
  delay(500);
  M5.dis.drawpix(0, YELLOW);
}

float Thrust0,Thrust,Roll,Pitch,Yaw;
float T1,T2,T3,T4;
float Time=0.0;

void loop() {
  // put your main code here, to run repeatedly:
  Thrust0 = Stick[THROTTLE];
  Thrust = 0.0f;
  Roll = Stick[AILERON];
  Pitch = Stick[ELEVATOR];
  Yaw = Stick[RUDDER];
  if(Thrust0 > 0.8f)Thrust0 = 0.8f;
  if(Thrust0 < 0.0f)Thrust0 = 0.0f;
  if(Thrust > CTRL_LIMIT)Thrust = CTRL_LIMIT;
  if(Thrust < 0.0f)Thrust = 0.0f;
  if(Roll >  CTRL_LIMIT)Roll =  CTRL_LIMIT;
  if(Roll < -CTRL_LIMIT)Roll = -CTRL_LIMIT;
  if(Pitch >  CTRL_LIMIT)Pitch =  CTRL_LIMIT;
  if(Pitch < -CTRL_LIMIT)Pitch = -CTRL_LIMIT;
  if(Yaw >  CTRL_LIMIT)Yaw =  CTRL_LIMIT;
  if(Yaw < -CTRL_LIMIT)Yaw = -CTRL_LIMIT;

  
  if(Thrust0 > 0.02f)
  {
    M5.dis.drawpix(0, RED);
    T1 = UGAIN*(Thrust0 + (Thrust - Roll + Pitch - Yaw)/4.0);
    T2 = UGAIN*(Thrust0 + (Thrust - Roll - Pitch + Yaw)/4.0);
    T3 = UGAIN*(Thrust0 + (Thrust + Roll - Pitch - Yaw)/4.0);
    T4 = UGAIN*(Thrust0 + (Thrust + Roll + Pitch + Yaw)/4.0);
  }
  else
  {
    M5.dis.drawpix(0, YELLOW);
    T1 = 0.0f;
    T2 = 0.0f;
    T3 = 0.0f;
    T4 = 0.0f;
  }

  if (T1 > 250.0f)T1 = 250.0f;
  if (T1 < 0.0f) T1 = 0.0f;
  if (T2 > 250.0f)T2 = 250.0f;
  if (T2 < 0.0f) T2 = 0.0f;
  if (T3 > 250.0f)T3 = 250.0f;
  if (T3 < 0.0f) T3 = 0.0f;
  if (T4 > 250.0f)T4 = 250.0f;
  if (T4 < 0.0f) T4 = 0.0f;

  ledcWrite(FR_CH, (uint32_t)T1);
  ledcWrite(FL_CH, (uint32_t)T4);
  ledcWrite(RR_CH, (uint32_t)T2);
  ledcWrite(RL_CH, (uint32_t)T3);

  sensor_read();
  Serial.printf("%f %f %f %f %f %f %f\n\r", Time, Wp, Wq, Wr, Ax, Ay, Az);
  Time = Time + 0.004;
  delay(4);
}