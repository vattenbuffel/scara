#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 A3

// How many messages that should be stored
#define N_CMD_STORE_MAX 10

// With gripper
// #define z_height_start_mm 155
// With pen holder
#define z_height_start_mm 155

// Macro to calculate how many steps n mm corresponds to in z
#define MM_TO_STEPS_Z(n) ((int)((n)*zDistanceToSteps))
// Macro to calculate how many mm n steps corresponds to in z
#define STEPS_TO_MM_Z(n) ((int)((n) / zDistanceToSteps))

// Macro to calculate how many steps deg degrees corresponds to with angle_to_steps = ang_to_steps
#define DEG_TO_STEPS_BASE(deg, ang_to_steps) ((int)((deg)*ang_to_steps))
//Macro to calculate deg to steps in theta1
#define DEG_TO_STEPS_THETA1(deg) (DEG_TO_STEPS_BASE((deg), theta1AngleToSteps))
//Macro to calculate deg to steps in theta2
#define DEG_TO_STEPS_THETA2(deg) (DEG_TO_STEPS_BASE((deg), theta2AngleToSteps))
//Macro to calculate deg to steps in theta3
#define DEG_TO_STEPS_PHI(deg) (DEG_TO_STEPS_BASE((deg), phiAngleToSteps))

// Macro to calculate how many degrees n steps corresponds to with angle_to_steps = ang_to_steps
#define STEPS_TO_DEG_BASE(n, ang_to_steps) (((float)(n) / ang_to_steps))
//Macro to calculate deg to steps in theta1
#define STEPS_TO_DEG_THETA1(n) (STEPS_TO_DEG_BASE((n), theta1AngleToSteps))
//Macro to calculate deg to steps in theta2
#define STEPS_TO_DEG_THETA2(n) (STEPS_TO_DEG_BASE((n), theta2AngleToSteps))
//Macro to calculate deg to steps in phi
#define STEPS_TO_DEG_PHI(n) (STEPS_TO_DEG_BASE((n), phiAngleToSteps))

// Macro to calculate the next value of Cmd_cur_i
#define CMD_CUR_I_INC(i) (((i) + 1) % (N_CMD_STORE_MAX))
// Macro to calculate the next value of Cmd_end_i
#define CMD_END_I_INC(i) (CMD_CUR_I_INC((i)))

// How often to send heartbeat, measured in loop iterations
#define HEARTBEAT_INTERVAL 100000
// How often to read data, measured in loop iterations
#define READ_SERIAL_INTERVAL 10000

struct MSG {
  bool stop;
  bool home;
  bool move;
  float J1;
  float J2;
  float phi;
  float z;
  int gripper_value;
  float v_J1;
  float v_J2;
  float v_phi;
  float v_z;
  float a_J1;
  float a_J2;
  float a_phi;
  float a_z;
  float accuracy;  // How close the robot has to come to the goal positions before starting to transition to the next movement
};

struct CMD {
  MSG msg;
  int J1_goal;
  int J2_goal;
  int phi_goal;
  int z_goal;
};





void homeing();
void stop();
bool move();
void publish_heartbeat();
void publish_done();

// Define the stepper motors and the pins the will use
AccelStepper stepper1(1, 2, 5);    // phi, (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);    // theta1
AccelStepper stepper3(1, 4, 7);    // theta2
AccelStepper stepper4(1, 12, 13);  // z

Servo gripperServo;  // create servo object to control a servo


double L1 = 228;    // L1 = 228mm
double L2 = 136.5;  // L2 = 136.5mm
double theta1, theta2, phi, z, gripper_value;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;

bool moving = false;

String string_received;


CMD cmds[N_CMD_STORE_MAX];
int n_cmd_in_storage = 0;
int cmd_cur_i = 0;
int cmd_end_i = 0;


void setup() {
  Serial.begin(115200);
  Serial.println(F("Hello world!"));

  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(4000);
  stepper4.setAcceleration(2000);

  // gripperServo.attach(A0, 600, 2500);
  gripperServo.attach(A0);
  // initial servo value - open gripper
  gripper_value = 0;
  gripperServo.write(gripper_value);
  // homeing();
  // delay(10000000);
}
unsigned long t_start = micros();
void loop() {

  // Serial.println(F("should publish heartbeat");
  publish_heartbeat();

  read_msg();

  if (n_cmd_in_storage > 0) {
    // STOP
    if (cmds[cmd_cur_i].msg.stop) {
      stop();
    }
    // Home
    else if (cmds[cmd_cur_i].msg.home) {
      homeing();
    }
    // Alter gripper, joints and z, i.e MOVE
    else if (cmds[cmd_cur_i].msg.move) {
      if (!moving && n_cmd_in_storage) {
        set_v_a_gripper();
      }
      bool done_with_cur_cmd = move();
      // Prepare to perform next cmd or when starting from stationary
      if (done_with_cur_cmd && n_cmd_in_storage) {
        set_v_a_gripper();
      }
      // All cmds have been executed so currently stationary
      if (!n_cmd_in_storage)
        moving = false;
    }
  }
}

void set_v_a_gripper() {
  // Update speed, acc and gripper value
  stepper1.moveTo(cmds[cmd_cur_i].phi_goal);
  stepper2.moveTo(cmds[cmd_cur_i].J1_goal);
  stepper3.moveTo(cmds[cmd_cur_i].J2_goal);
  stepper4.moveTo(cmds[cmd_cur_i].z_goal);

  stepper1.setAcceleration(cmds[cmd_cur_i].msg.a_phi);
  stepper2.setAcceleration(cmds[cmd_cur_i].msg.a_J1);
  stepper3.setAcceleration(cmds[cmd_cur_i].msg.a_J2);
  stepper4.setAcceleration(cmds[cmd_cur_i].msg.a_z);

  // Set the speed and sign of the speed for each motor
  stepper1.setSpeed(cmds[cmd_cur_i].msg.v_phi * ((cmds[cmd_cur_i].msg.phi > phi) ? 1 : -1));
  stepper2.setSpeed(cmds[cmd_cur_i].msg.v_J1 * ((cmds[cmd_cur_i].msg.J1 > theta1) ? 1 : -1));
  stepper3.setSpeed(cmds[cmd_cur_i].msg.v_J2 * ((cmds[cmd_cur_i].msg.J2 > theta2) ? 1 : -1));
  Serial.print(F("J2 vel: "));
  Serial.println(cmds[cmd_cur_i].msg.v_J2 * ((cmds[cmd_cur_i].msg.J2 > theta2) ? 1 : -1));
  stepper4.setSpeed(cmds[cmd_cur_i].msg.v_z * ((cmds[cmd_cur_i].msg.z > z) ? 1 : -1));

  gripperServo.write(cmds[cmd_cur_i].msg.gripper_value);
}

void read_msg() {
  // Only read at set intervals
  static long counter = 0;
  counter += 1;
  if (counter > READ_SERIAL_INTERVAL){ 
    counter = 0;
    return;
  }
  
  // Read the message from the serial and parse it into a cmd
  bool full_msg = false;
  if (n_cmd_in_storage < N_CMD_STORE_MAX)
    full_msg = read_serial();
  if (full_msg) {
    parse_msg();
    string_received = "";
  }
}

bool read_serial() {
  char c;
  while (Serial.available()) {
    c = Serial.read();
    string_received += c;
    // Serial.print(F("String received so far: "));
    // Serial.println(string_received);
    // Serial.print("Char received: '");
    // Serial.print(c);
    // Serial.println("'");

    if ('\n' == c) {
      // Serial.print(F("Read end of string"));
      Serial.print(F("Full string read: "));
      Serial.println(string_received);
      return true;
    }
  }
  return false;
}

void parse_msg() {
  int n_data = 17;
  float data[n_data];
  // Extract the data from the string and put into a msg
  for (int i = 0; i < n_data; i++) {
    int index = string_received.indexOf(",");  // locate the first ","
    if (index == -1 && i != n_data - 1) {
      Serial.println(F("Invalid msg received"));
    }
    data[i] = atoi(string_received.substring(0, index).c_str());  //Extract the number from start to the ","
    string_received = string_received.substring(index + 1);       //Remove the number from the string
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
  cmds[cmd_end_i].msg.stop = (bool)data[0];
  cmds[cmd_end_i].msg.home = (bool)data[1];
  cmds[cmd_end_i].msg.move = (bool)data[2];
  cmds[cmd_end_i].msg.J1 = data[3];
  cmds[cmd_end_i].msg.J2 = data[4];
  cmds[cmd_end_i].msg.phi = data[5];
  cmds[cmd_end_i].msg.z = data[6];
  cmds[cmd_end_i].msg.gripper_value = (int)data[7];
  cmds[cmd_end_i].msg.v_J1 = data[8];
  cmds[cmd_end_i].msg.v_J2 = data[8];
  cmds[cmd_end_i].msg.v_phi = data[8];
  cmds[cmd_end_i].msg.v_z = data[8];
  cmds[cmd_end_i].msg.a_J1 = data[9];
  cmds[cmd_end_i].msg.a_J2 = data[9];
  cmds[cmd_end_i].msg.a_phi = data[9];
  cmds[cmd_end_i].msg.a_z = data[9];
  // cmds[cmd_end_i].msg.accuracy = data[0];
  cmds[cmd_end_i].J1_goal = DEG_TO_STEPS_THETA1(cmds[cmd_end_i].msg.J1);
  cmds[cmd_end_i].J2_goal = DEG_TO_STEPS_THETA2(cmds[cmd_end_i].msg.J2);
  cmds[cmd_end_i].phi_goal = DEG_TO_STEPS_PHI(cmds[cmd_end_i].msg.phi);
  cmds[cmd_end_i].z_goal = MM_TO_STEPS_Z(cmds[cmd_end_i].msg.z);

  // Serial.println(F("Received cmd: "));
  // cmd_print(&cmds[cmd_end_i]);

  n_cmd_in_storage += 1;
  cmd_end_i = CMD_END_I_INC(cmd_end_i);

  // Serial.print(F("Currently: "));
  // Serial.print(n_cmd_in_storage);
  // Serial.print(F(" cmds in storage"));
  // Serial.print(F(". Cmd_cur_i: "));
  // Serial.print(cmd_cur_i);
  // Serial.print(F(". Cmd_end_i: "));
  // Serial.println(cmd_end_i);
}

void msg_print(MSG *msg) {
  Serial.print(F("msg->stop: "));
  Serial.println(msg->stop);
  Serial.print(F("msg->home: "));
  Serial.println(msg->home);
  Serial.print(F("msg->move: "));
  Serial.println(msg->move);
  Serial.print(F("msg->J1: "));
  Serial.println(msg->J1);
  Serial.print(F("msg->J2: "));
  Serial.println(msg->J2);
  Serial.print(F("msg->phi: "));
  Serial.println(msg->phi);
  Serial.print(F("msg->z: "));
  Serial.println(msg->z);
  Serial.print(F("msg->gripper_value: "));
  Serial.println(msg->gripper_value);
  Serial.print(F("msg->v_J1: "));
  Serial.println(msg->v_J1);
  Serial.print(F("msg->v_J2: "));
  Serial.println(msg->v_J2);
  Serial.print(F("msg->v_phi: "));
  Serial.println(msg->v_phi);
  Serial.print(F("msg->v_z: "));
  Serial.println(msg->v_z);
  Serial.print(F("msg->a_J1: "));
  Serial.println(msg->a_J1);
  Serial.print(F("msg->a_J2;: "));
  Serial.println(msg->a_J2);
  Serial.print(F("msg->a_phi: "));
  Serial.println(msg->a_phi);
  Serial.print(F("msg->a_z: "));
  Serial.println(msg->a_z);
  Serial.print(F("msg->accuracy: "));
  Serial.println(msg->accuracy);
}

void cmd_print(CMD *cmd) {
  Serial.println(F("cmd->msg: {"));
  msg_print(&cmd->msg);
  Serial.println(F("}"));
  Serial.print("cmd->J1_goa: l");
  Serial.println(cmd->J1_goal);
  Serial.print("cmd->J2_goal: ");
  Serial.println(cmd->J2_goal);
  Serial.print("cmd->phi_goal: ");
  Serial.println(cmd->phi_goal);
  Serial.print("cmd->z_goal: ");
  Serial.println(cmd->z_goal);
}

bool move() {
  // Check if robot is close enough to move on to the next msg
  bool done_with_cur_idx = false;


  if (abs(stepper2.currentPosition() - cmds[cmd_cur_i].J1_goal) <= cmds[cmd_cur_i].msg.accuracy) {
    // Serial.println(F("Finish with J1"));
    if (abs(stepper3.currentPosition() - cmds[cmd_cur_i].J2_goal) <= cmds[cmd_cur_i].msg.accuracy) {
      // Serial.println(F("Finish with J2"));
      // if (abs(stepper1.currentPosition() - cmds[cmd_cur_i].phi_goal) <= DEG_TO_STEPS_PHI(cmds[cmd_cur_i].msg.accuracy)){
      if (abs(stepper4.currentPosition() - cmds[cmd_cur_i].z_goal) <= cmds[cmd_cur_i].msg.accuracy) {
        // Serial.println(F("Finish with z"));

        done_with_cur_idx = true;
        // Serial.println(F("finish moving"));
        done_with_msg();
      }
      // }
    }
  }

  // Move the motors
  moving = false;
  if (stepper2.currentPosition() != cmds[cmd_cur_i].J1_goal)
    stepper2.runSpeed();
  {
    moving = true;
  }
  if (stepper3.currentPosition() != cmds[cmd_cur_i].J2_goal) {
    // Serial.println(F("Stepping J2"));
    stepper3.runSpeed();
    moving = true;
  }
  if (stepper4.currentPosition() != cmds[cmd_cur_i].z_goal) {
    stepper4.runSpeed();
    moving = true;
  }

  theta1 = STEPS_TO_DEG_THETA1(stepper2.currentPosition());
  theta2 = STEPS_TO_DEG_THETA2(stepper3.currentPosition());
  phi = STEPS_TO_DEG_PHI(stepper1.currentPosition());
  z = STEPS_TO_MM_Z(stepper4.currentPosition());

  return done_with_cur_idx;
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();                  // get one character
  }
}

void publish_heartbeat() {
  static long counter = -1;
  counter++;
  if (counter>HEARTBEAT_INTERVAL) {
    counter = 0;
    Serial.print(F("HEARTBEAT "));
    Serial.print(theta1);
    Serial.print(F(" "));
    Serial.print(theta2);
    Serial.print(F(" "));
    Serial.print(phi);
    Serial.print(F(" "));
    Serial.print(z);
    Serial.print(F(" "));
    Serial.println(gripper_value);
  }
}

void publish_done() {
  Serial.println(F("DONE"));
}

void stop() {
  Serial.println(F("Stop not implemented"));
}

void done_with_msg() {
  cmd_cur_i = CMD_CUR_I_INC(cmd_cur_i);
  n_cmd_in_storage -= 1;
  Serial.print(F("Finish msg. Currently: "));
  Serial.print(n_cmd_in_storage);
  Serial.print(F(" cmds in storage"));
  Serial.print(F(". Cmd_cur_i: "));
  Serial.print(cmd_cur_i);
  Serial.print(F(". Cmd_end_i: "));
  Serial.println(cmd_end_i);
  publish_done();
}

void homeing() {
  // To ensure slow and safe homeing set the velocity and acceleration to something low
  // TODO: Actually do the above


  Serial.println(F("Gonna home stepper 4"));
  // Homing Stepper4
  for (int i=0; i < 2; i++){
    while (digitalRead(limitSwitch4) != 1) {
      stepper4.setSpeed(1500);
      stepper4.runSpeed();
    }
    stepper4.setCurrentPosition(z_height_start_mm*zDistanceToSteps);
    delay(20);
    int goal_pos = stepper4.currentPosition() - MM_TO_STEPS_Z(50);
    stepper4.moveTo(goal_pos);
      stepper4.setSpeed(-1500);
    while (stepper4.currentPosition() != goal_pos) {
      stepper4.runSpeed();
    }
    delay(100);
  }
  z = STEPS_TO_MM_Z(stepper4.currentPosition());
  Serial.println(F("Finish homing stepper 4"));



  // Homing Stepper3
  Serial.println(F("Gonna home stepper 3"));
  for (int i = 0; i < 2; i++) {
    while (digitalRead(limitSwitch3) != 1) {
      stepper3.setSpeed(-1100);
      stepper3.runSpeed();
    }
    stepper3.setCurrentPosition(-5500);
    delay(20);
    int goal_pos = -DEG_TO_STEPS_THETA2(90);
    stepper3.moveTo(goal_pos);
    stepper3.setSpeed(1100);
    while (stepper3.currentPosition() != goal_pos) {
      stepper3.runSpeed();
    }
    delay(100);
  }
  theta2 = STEPS_TO_DEG_THETA2(stepper3.currentPosition());
  Serial.println(F("Finish homing stepper 3"));

  // Homing Stepper2
  Serial.println(F("Gonna home stepper 2"));
  for (int i=0; i < 2; i++){
    while (digitalRead(limitSwitch2) != 1) {
      stepper2.setSpeed(-1300);
      stepper2.runSpeed();
    }
    stepper2.setCurrentPosition(-8800);
    delay(20);
    int goal_pos = DEG_TO_STEPS_THETA1(-90);
    stepper2.moveTo(goal_pos);
    stepper2.setSpeed(1300);
    while (stepper2.currentPosition() != goal_pos) {
      stepper2.runSpeed();
    }
    delay(100);
  }
  theta1 = STEPS_TO_DEG_THETA1(stepper2.currentPosition());
  Serial.println(F("Finish homing stepper 2"));


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
  // Serial.println(F("Finish homing stepper 1"));
  Serial.println(F("At home"));
  done_with_msg();
}
