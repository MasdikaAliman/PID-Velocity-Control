#include <Encoder.h>
#include "math.h"
#define IN1 6
#define IN2 7
#define  max_rpm 816.0
#define ppr 448.0

Encoder myEnc(20, 28);

//   avoid using pins with LEDs attached
float targetRPM = -300;
long oldPulse = 0;
double last_time;
float eintegral = 0;
double deltaT =0;
void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  last_time = micros();
}

long oldPosition  = -999;

void loop() {
  // target = (200.0 / 816.0) * 255;
  long newPulse = myEnc.read();
  // Serial.println(newPulse);
  deltaT = micros()-last_time;
  last_time = micros();
  double vel = ((newPulse - oldPulse)/ppr)* 2*M_PI ;
  oldPulse = newPulse;
  // Serial.println(deltaT);
  // Serial.println(vel);
  double rps = vel/(deltaT/1e6);
  double rpm_ref = (rps / (2*M_PI)) *60;

  float u = PID_calculate(targetRPM, rpm_ref, 1.5, 0.05);
  int dir = 1;
  if(u < 0)
    dir = -1;
  int pwm = fabs((u / max_rpm) * 255);
  
  if(pwm > 255)
    pwm = 255;

  setMotor(dir, pwm , 1, IN1,IN2);
  // analogWrite(IN1, target);
  // digitalWrite(IN2,LOW);
  Serial.print("Target RPM = ");
  Serial.print(targetRPM);
  Serial.print("   out RPM = ");
  Serial.println(rpm_ref);
}

float PID_calculate(float target, float refTarget, float kp, float ki){
    float output;
    float error = target - refTarget;
    float propartional = kp*error;
    eintegral += error;

    output = propartional + ki * eintegral;
    return output;
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  // analogWrite(pwm,pwmVal); // Motor speed
  if (dir == 1) {
    // Turn one way
    analogWrite(in1, pwmVal);
    // digitalWrite(in2,LOW);
  } else if (dir == -1) {
    // Turn the other way
    // digitalWrite(in1,LOW);
    // digitalWrite(in2,HIGH);
    analogWrite(in2, pwmVal);

  } else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

