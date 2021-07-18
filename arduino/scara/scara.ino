
#include <MultiStepper.h>

/*
   Arduino based SCARA Robot 
   by Dejan, www.HowToMechatronics.com
   AccelStepper: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html

*/
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 A3

#define home_speed 50
#define home_acceleration 50

void homeing();
void stop();
void move();
void publish_heartbeat();
void publish_done();

// Define the stepper motors and the pins the will use
AccelStepper stepper1(1, 2, 5); // (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepper4(1, 12, 13);

Servo gripperServo;  // create servo object to control a servo


double L1 = 228; // L1 = 228mm
double L2 = 136.5; // L2 = 136.5mm
double theta1, theta2, phi, z, gripper_value;

int stepper1Position, stepper2Position, stepper3Position, stepper4Position;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;

byte inputValue[5];
int k = 0;

String content = "";
int data[10];


void setup() {
  Serial.begin(115200);
  Serial.println("Hello world!");

  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  // NOA TEST
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  // END NOA TEST

  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(4000);
  stepper4.setAcceleration(2000);

  gripperServo.attach(A0, 600, 2500);
  // initial servo value - open gripper
  data[6] = 180;
  gripperServo.write(data[6]);
  delay(1000);
  data[5] = 100;
}

void loop() {
  // Serial.println("should publish heartbeat");
  publish_heartbeat();
  delay(25); // Sometimes the data sent by publish_heartbeat got corrupted. This seems to have alleviated this problem

  if (Serial.available()) {
    content = Serial.readString(); // Read the incomding data from Processing
    Serial.print("Received: ");
    Serial.println(content);
    // Extract the data from the string and put into separate integer variables (data[] array)
    for (int i = 0; i < 10; i++) {
      int index = content.indexOf(","); // locate the first ","
      data[i] = atol(content.substring(0, index).c_str()); //Extract the number from start to the ","
      content = content.substring(index + 1); //Remove the number from the string
    }
    /*
     data[0] - STOP 
     data[1] - HOME 
     data[2] - MOVE 
     data[3] - Joint 1 angle
     data[4] - Joint 2 angle
     data[5] - Joint 3 angle
     data[6] - Z position
     data[7] - Gripper value
     data[8] - Speed value
     data[9] - Acceleration value
    */
    Serial.print("data[1]: ");
    Serial.println(data[1]);

    // STOP
    if (data[0] == 1) {
      stop();
    }
    // Home
    else if (data[1] == 1) {
      Serial.println("should home");
      homeing();
    }
    // Alter gripper, joints and z, i.e MOVE
    else if (data[2] == 1) {
      move();
    }
  }
}

void move(){
  stepper1Position = data[3] * theta1AngleToSteps;
  stepper2Position = data[4] * theta2AngleToSteps;
  stepper3Position = data[5] * phiAngleToSteps;
  stepper4Position = data[6] * zDistanceToSteps;

  stepper1.setSpeed(data[8]);
  stepper2.setSpeed(data[8]);
  stepper3.setSpeed(data[8]);
  stepper4.setSpeed(data[8]);

  stepper1.setAcceleration(data[9]);
  stepper2.setAcceleration(data[9]);
  stepper3.setAcceleration(data[9]);
  stepper4.setAcceleration(data[9]);

  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  stepper3.moveTo(stepper3Position);
  stepper4.moveTo(stepper4Position);

  gripperServo.write(data[7]);

  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position || stepper3.currentPosition() != stepper3Position || stepper4.currentPosition() != stepper4Position) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
  delay(100);

  publish_done();
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();         // get one character
  }
}

void publish_heartbeat(){
  // printf("HEARTBEAT %f %f %f %f %f\r\n", theta1, theta2, phi, z, gripper_value); // J1 J2 J3 z gripper_value
  Serial.print("HEARTBEAT ");
  Serial.print(theta1);
  Serial.print(" ");
  Serial.print(theta2);
  Serial.print(" ");
  Serial.print(phi);
  Serial.print(" ");
  Serial.print(z);
  Serial.print(" ");
  Serial.println(gripper_value);
}

void publish_done(){
  Serial.println(F("DONE"));
}

void stop(){ 
  printf("Stop not implemented\n");
}

void homeing() {
  // To ensure slow and safe homeing set the velocity and acceleration to something low
  // TODO: Actually do the above


  Serial.println(F("Gonna home stepper 4"));
  // Homing Stepper4
  while (digitalRead(limitSwitch4) != 1) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(17000); // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper4.moveTo(10000);
  while (stepper4.currentPosition() != 10000) {
    stepper4.run();
  }
  Serial.println(F("Done homing stepper 4"));
  

  // Homing Stepper3
  Serial.println(F("Gonna home stepper 3"));
  while (digitalRead(limitSwitch3) != 1) {
    stepper3.setSpeed(-1100);
    stepper3.runSpeed();
    stepper3.setCurrentPosition(-1662); // When limit switch pressed set position to 0 steps
  }
  delay(20);

  stepper3.moveTo(0);
  while (stepper3.currentPosition() != 0) {
    stepper3.run();
  }
  Serial.println(F("Done homing stepper 3"));

  // Homing Stepper2
  Serial.println(F("Gonna home stepper 2"));
  while (digitalRead(limitSwitch2) != 1) {
    stepper2.setSpeed(-1300);
    stepper2.runSpeed();
    stepper2.setCurrentPosition(-5420); // When limit switch pressed set position to -5440 steps
  }
  delay(20);

  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }
  Serial.println(F("Done homing stepper 2"));

  // Homing Stepper1
  // Serial.println(F("Gonna home stepper 1"));
  // while (digitalRead(limitSwitch1) != 1) {
  //   stepper1.setSpeed(-1200);
  //   stepper1.runSpeed();
  //   stepper1.setCurrentPosition(-3955); // When limit switch pressed set position to 0 steps
  // }
  // delay(20);
  // stepper1.moveTo(0);
  // while (stepper1.currentPosition() != 0) {
  //   stepper1.run();
  // }
  // Serial.println(F("Done homing stepper 1"));
  Serial.println(F("At home"));
  publish_done();
}
