#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 accelgyro(0x68);

int16_t ax, ay, az;

// MPU control/status vars
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 181.6;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 100;  
double Ki = 0; 
double Kd = 1;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int in1 = 6;
int in2 = 5;
int in3 = 4;
int in4 = 3;
int enA = 9;
int enB = 10;
int vel = 200;
int minVel = 20;

int motorAConstant = 1;
int motorBConstant = 1;

//Volatile means subject to change independent of code
volatile bool mpuInterrupt = false; //check voltahe high/low to the interupt PIN

void dmpDataReady(){
  mpuInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  Wire.begin();

  Serial.begin(38400);
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  devStatus = accelgyro.dmpInitialize();

  //Factory offset can tweak to make smoother
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  accelgyro.setZAccelOffset(1688);
  if(devStatus == 0){
    accelgyro.setDMPEnabled(true);

    //enable Arduino Interupt detection
    attachInterrupt(0, dmpDataReady, RISING);
  
    packetSize = accelgyro.dmpGetFIFOPacketSize();
  
    //Setting up PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
    Serial.println("DMP Initialization successful");
  }else{
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

}

void loop() {

  // put your main code here, to run repeatedly:
  while (!mpuInterrupt && fifoCount < packetSize)
  {
  //no mpu data - performing PID calculations and output to motors 
  pid.Compute();
  move(output, minVel);
  //Serial.print("Check");
  
  
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();
  
  // get current FIFO count
  fifoCount = accelgyro.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
  // reset so we can continue cleanly
  accelgyro.resetFIFO();
  Serial.println(F("FIFO overflow!"));
  
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
  
  // read a packet from FIFO
  accelgyro.getFIFOBytes(fifoBuffer, packetSize);
  
  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
  
  accelgyro.dmpGetQuaternion(&q, fifoBuffer);
  accelgyro.dmpGetGravity(&gravity, &q);
  accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
  input = ypr[1] * 180/M_PI + 180;
  } 


}

void clockwise(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void counterclockwise(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

//PID controls output so must ensure it is within bounds
void move(int spd, int minSpeed){

  int direction = 1;

  if(spd < 0){
    direction = -1;

    spd = max(spd, -255);
  }else{
    spd = min(spd, 255);
  }

  int realSpeed = max(minSpeed, abs(spd));

  analogWrite(enA, realSpeed * motorAConstant);
  analogWrite(enB, realSpeed * motorBConstant);

  if(spd > 0){
    clockwise();
  }else{
    counterclockwise();
  }
  Serial.print("check");
}
