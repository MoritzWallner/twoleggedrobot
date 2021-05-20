/*  Firmware written by Moritz Wallner in 2021
 *  This firmware controls one of the most useful robots out there.
 *  Github: https://github.com/MoritzWallner
 */

/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //default address 0x40
#define SERVOMIN  80 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)

//latest angles
float axis0_angle = 0;
float axis1_angle = 0;
float axis2_angle = 0;
float axis3_angle = 0;
float axis4_angle = 0;
float axis5_angle = 0;

//This function moves the specified servo motor.
void setServo(int servo, float deg)
{
  pwm.setPWM(servo, 0, map(deg, 0, 180, SERVOMIN, SERVOMAX));
}

//This function moves the robot to predefined positions.
void move(String pos)
{
  if (pos == "stand")
  {
    move(90, 90, 3, 10, 90, 97);
    return true;
  }
  if (pos == "crouch")
  {
    move(60, 120, 90, 100, 30, 30);
    return true;
  }
  return false;
}

//This function moves all axis to the set angle. All axis reach the desired positon at the same time.
void move(float a0, float a1, float a2, float a3, float a4, float a5)
{
  //calculate the distance every axis needs to travel
  float axis0_distance = a0 - axis0_angle;
  float axis1_distance = a1 - axis1_angle;
  float axis2_distance = a2 - axis2_angle;
  float axis3_distance = a3 - axis3_angle;
  float axis4_distance = a4 - axis4_angle;
  float axis5_distance = a5 - axis5_angle;

  Serial.print("axis0_distance = ");
  Serial.print(axis0_distance);
  Serial.print(" a0 = ");
  Serial.println(a0);

  Serial.print("axis1_distance = ");
  Serial.print(axis1_distance);
  Serial.print(" a1 = ");
  Serial.println(a1);

  Serial.print("axis2_distance = ");
  Serial.print(axis2_distance);
  Serial.print(" a2 = ");
  Serial.println(a2);

  Serial.print("axis3_distance = ");
  Serial.print(axis3_distance);
  Serial.print(" a3 = ");
  Serial.println(a3);

  Serial.print("axis4_distance = ");
  Serial.print(axis4_distance);
  Serial.print(" a4 = ");
  Serial.println(a4);

  Serial.print("axis5_distance = ");
  Serial.print(axis5_distance);
  Serial.print(" a5 = ");
  Serial.println(a5);

  //find out which axis turn the furthest
  float maxDistance = max(max(max(max(max(abs(axis0_distance), abs(axis1_distance)), abs(axis2_distance)), abs(axis3_distance)), abs(axis4_distance)), abs(axis5_distance));
  Serial.print("maxDistance = ");
  Serial.println(maxDistance);
  if (maxDistance == 0) return; //increase speed and prevent division by zero

  //move the axis step by step so that the robot does not fall over
  for (int steps = 0; steps <= maxDistance; steps += 1)
  {
    setServo(0, axis0_angle + (axis0_distance / maxDistance) * steps);
    setServo(1, axis1_angle + (axis1_distance / maxDistance) * steps);
    setServo(2, axis2_angle + (axis2_distance / maxDistance) * steps);
    setServo(3, axis3_angle + (axis3_distance / maxDistance) * steps);
    setServo(4, axis4_angle + (axis4_distance / maxDistance) * steps);
    setServo(5, axis5_angle + (axis5_distance / maxDistance) * steps);
    Serial.print("steps = ");
    Serial.println(steps);
    delay(10); //comfortable speed. Too slow is boring and too fast is dangerous
  }
  axis0_angle = a0;
  axis1_angle = a1;
  axis2_angle = a2;
  axis3_angle = a3;
  axis4_angle = a4;
  axis5_angle = a5;
  Serial.println("-- moved --");
}

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); //set the frequency to 50 Hz
  delay(100);
  Serial.println("Initial position");

  setServo(0, 90);
  setServo(1, 90);
  setServo(2, 3);
  setServo(3, 10);
  setServo(4, 90);
  setServo(5, 97);

  axis0_angle = 90;
  axis1_angle = 90;
  axis2_angle = 3;
  axis3_angle = 10;
  axis4_angle = 90;
  axis5_angle = 97;

  delay(3000); //wait 3 s sothat the robot is standing still
}

void loop() {
  move("crouch");
  move("stand");
}
