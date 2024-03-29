#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 A3

#define READY_TO_RECEIVE_STR F("<r>\n")

// How many messages that should be stored
#define N_CMD_STORE_MAX 5

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

struct MSG {
  int cmd;
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
  float J1_accuracy;
  float J2_accuracy;
  float phi_accuracy;
  float z_accuracy;
};


enum cmd_types{
  HOME_ENUM = 0,
  MOVE_ENUM,
  STOP_ENUM
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
String string_send = "";
int string_send_size = 0;


CMD cmds[N_CMD_STORE_MAX];
int n_cmd_in_storage = 0;
int cmd_cur_i = 0;
int cmd_end_i = 0;


void setup() {
  Serial.begin(115200);
  while (!Serial){;}
  add_data_to_send(F("Hello world!\n"));
  send_ready_to_receive();

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

void loop() {

  // add_data_to_send(F("should publish heartbeat"));
  publish_heartbeat();

  read_msg();
  send_data();

  if (n_cmd_in_storage > 0) {
    // STOP
    if (cmds[cmd_cur_i].msg.cmd == STOP_ENUM) {
      stop();
    }
    // Home
    else if (cmds[cmd_cur_i].msg.cmd == HOME_ENUM) {
      homeing();
    }
    // Alter gripper, joints and z, i.e MOVE
    else if (cmds[cmd_cur_i].msg.cmd == MOVE_ENUM) {
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
  stepper4.setSpeed(cmds[cmd_cur_i].msg.v_z * ((cmds[cmd_cur_i].msg.z > z) ? 1 : -1));

  gripperServo.write(cmds[cmd_cur_i].msg.gripper_value);
}

void read_msg() {  
  static unsigned long counter = 0;
  counter++;
  if (counter < 1000)
    return
  counter = 0;

  // Read the message from the serial and parse it into a cmd
  bool full_msg = false;
  if (n_cmd_in_storage < N_CMD_STORE_MAX)
    full_msg = read_serial();
  if (full_msg) {
    // debug_print("Received string: ");
    // for (int i = 0; i < string_received.length(); i++)
    //   debug_print(String(string_received.charAt(i)));
    // Serial.println();
    // unsigned long start_time = micros();
    parse_msg();
    // unsigned long end_time = micros();
    // debug_print(F("It took: "));
    // debug_print(String((end_time - start_time)/1000));
    // debug_print(F(" ms to parse\n"));
    string_received = "";
    // debug_print(F("String length after finish parsing and string_received = '': "));
    // debug_print(String(string_received.length()));
    // debug_print(F("\n"));
    // debug_print("Finish parse msg\n");
  }
}

bool read_serial() {
  // Only read 1 char per call
  static char n_read = 0;
  bool done = false;
  if (Serial.available()) {
    char c = Serial.read();
    string_received += c;
    n_read++;
    // add_data_to_send(F("String received so far: "));
    // Serial.println(string_received);
    // add_data_to_send(F("Char received: '"));
    // Serial.print(c);
    // Serial.println(("'"));

    if ('\n' == c) {
      // add_data_to_send(F("Read end of string. "));
      // add_data_to_send(F("Full string read: "));
      // Serial.println(string_received);
      done = true;
      n_read = 0;
    }
  }
  // if 64 chars have been read or '\n' has been found then all possible data has been read and should signal that more can be read
  if (done || n_read == 64){
    send_ready_to_receive();
    n_read = 0;
  }

  return done;
}

void parse_msg() {
  int n_data = 15;
  float data[n_data];
  // Extract the data from the string and put into a msg
  for (int i = 0; i < n_data; i++) {
    int index = string_received.indexOf(",");  // locate the first ","
    if (index == -1 && i != n_data - 1) {
      debug_print(F("Invalid msg received\n"));
    }
    data[i] = atof(string_received.substring(0, index).c_str());  //Extract the number from start to the ","
    string_received = string_received.substring(index + 1);       //Remove the number from the string
    // debug_print(F("Data["));
    // debug_print(String(i));
    // debug_print(F("]: "));
    // debug_print(String(data[i]));
    // debug_print(F("\n"));
    
    // debug_print(F("String left: "));
    // for (int i = 0; i < string_received.length(); i++)
    //     debug_print(String(string_received.charAt(i)));
  }
  /*
    data[0] - cmd_type [cmd enum] 
    data[1] - Joint 1 angle
    data[2] - Joint 2 angle
    data[3] - Joint 3 angle
    data[4] - Z position
    data[5] - Gripper value
    data[6] - J1_vel value
    data[7] - J2_vel value
    data[8] - J3_vel value
    data[9] - z_vel value
    data[10] - J1_acc value
    data[11] - J2_acc value
    data[12] - J3_acc value
    data[13] - z_acc value
    data[14] - accuracy, how close the arduino has to get to the position before starting to move to the next cmd
  */
  cmds[cmd_end_i].msg.cmd = (int)data[0];
  cmds[cmd_end_i].msg.J1 = data[1];
  cmds[cmd_end_i].msg.J2 = data[2];
  cmds[cmd_end_i].msg.phi = data[3];
  cmds[cmd_end_i].msg.z = data[4];
  cmds[cmd_end_i].msg.gripper_value = (int)data[5];
  // Kinda bad that the the converted velocities and not raw are stored in msg. The raw should be stored in msg and the converted should be in cmd
  cmds[cmd_end_i].msg.v_J1 = DEG_TO_STEPS_THETA1(data[6]); // Convert the data from degrees/s to steps/s
  cmds[cmd_end_i].msg.v_J2 = DEG_TO_STEPS_THETA2(data[7]); // Convert the data from degrees/s to steps/s
  cmds[cmd_end_i].msg.v_phi = DEG_TO_STEPS_PHI(data[8]); // Convert the data from degrees/s to steps/s
  cmds[cmd_end_i].msg.v_z = MM_TO_STEPS_Z(data[9]); // Convert the data from mm/s to steps/s
  cmds[cmd_end_i].msg.a_J1 = DEG_TO_STEPS_THETA1(data[10]);
  cmds[cmd_end_i].msg.a_J2 = DEG_TO_STEPS_THETA2(data[11]);
  cmds[cmd_end_i].msg.a_phi = DEG_TO_STEPS_PHI(data[12]);
  cmds[cmd_end_i].msg.a_z = MM_TO_STEPS_Z(data[13]);
  cmds[cmd_end_i].msg.accuracy = data[14];
  
  cmds[cmd_end_i].J1_goal = DEG_TO_STEPS_THETA1(cmds[cmd_end_i].msg.J1);
  cmds[cmd_end_i].J2_goal = DEG_TO_STEPS_THETA2(cmds[cmd_end_i].msg.J2);
  cmds[cmd_end_i].phi_goal = DEG_TO_STEPS_PHI(cmds[cmd_end_i].msg.phi);
  cmds[cmd_end_i].z_goal = MM_TO_STEPS_Z(cmds[cmd_end_i].msg.z);
  cmds[cmd_end_i].J1_accuracy = DEG_TO_STEPS_THETA1(cmds[cmd_end_i].msg.accuracy);
  cmds[cmd_end_i].J2_accuracy = DEG_TO_STEPS_THETA2(cmds[cmd_end_i].msg.accuracy);
  cmds[cmd_end_i].phi_accuracy = DEG_TO_STEPS_PHI(cmds[cmd_end_i].msg.accuracy);
  cmds[cmd_end_i].z_accuracy = MM_TO_STEPS_Z(cmds[cmd_end_i].msg.accuracy);

  // add_data_to_send(F("Received cmd: \n"));
  // cmd_print(&cmds[cmd_end_i]);

  n_cmd_in_storage += 1;
  cmd_end_i = CMD_END_I_INC(cmd_end_i);
}

void msg_print(MSG *msg) {
  debug_print(F("msg->cmd: "));
  debug_print(String(msg->cmd));
  debug_print(F("\nmsg->J1: "));
  debug_print(String(msg->J1));
  debug_print(F("\nmsg->J2: "));
  debug_print(String(msg->J2));
  debug_print(F("\nmsg->phi: "));
  debug_print(String(msg->phi));
  debug_print(F("\nmsg->z: "));
  debug_print(String(msg->z));
  debug_print(F("\nmsg->gripper_value: "));
  debug_print(String(msg->gripper_value));
  debug_print(F("\nmsg->v_J1: "));
  debug_print(String(msg->v_J1));
  debug_print(F("\nmsg->v_J2: "));
  debug_print(String(msg->v_J2));
  debug_print(F("\nmsg->v_phi: "));
  debug_print(String(msg->v_phi));
  debug_print(F("\nmsg->v_z: "));
  debug_print(String(msg->v_z));
  debug_print(F("\nmsg->a_J1: "));
  debug_print(String(msg->a_J1));
  debug_print(F("\nmsg->a_J2: "));
  debug_print(String(msg->a_J2));
  debug_print(F("\nmsg->a_phi: "));
  debug_print(String(msg->a_phi));
  debug_print(F("\nmsg->a_z: "));
  debug_print(String(msg->a_z));
  debug_print(F("\nmsg->accuracy: "));
  debug_print(String(msg->accuracy));
  debug_print(F("\n"));
}

void cmd_print(CMD *cmd) {
  debug_print(F("cmd->msg: {\n"));
  msg_print(&cmd->msg);
  debug_print(F("}\n"));
  debug_print(F("\ncmd->J1_goal: "));
  debug_print(String(cmd->J1_goal));
  debug_print(F("\ncmd->J2_goal: "));
  debug_print(String(cmd->J2_goal));
  debug_print(F("\ncmd->phi_goal: "));
  debug_print(String(cmd->phi_goal));
  debug_print(F("\ncmd->z_goal: "));
  debug_print(String(cmd->z_goal));
  debug_print(F("\ncmd->J1_accuracy: "));
  debug_print(String(cmd->J1_accuracy));
  debug_print(F("\ncmd->J2_accuracy: "));
  debug_print(String(cmd->J2_accuracy));
  debug_print(F("\ncmd->phi_accuracy: "));
  debug_print(String(cmd->phi_accuracy));
  debug_print(F("\ncmd->z_accuracy: "));
  debug_print(String(cmd->z_accuracy));
  debug_print(F("\n"));
}

bool move() {
  // Check if robot is close enough to move on to the next msg
  bool done_with_cur_idx = false;


  if (abs(stepper2.currentPosition() - cmds[cmd_cur_i].J1_goal) <= cmds[cmd_cur_i].J1_accuracy) {
    // add_data_to_send(F("Finish with J1"));
    if (abs(stepper3.currentPosition() - cmds[cmd_cur_i].J2_goal) <= cmds[cmd_cur_i].J2_accuracy) {
      // add_data_to_send(F("Finish with J2"));
      // if (abs(stepper1.currentPosition() - cmds[cmd_cur_i].phi_goal) <= DEG_TO_STEPS_PHI(cmds[cmd_cur_i].J3_accuracy)){
      if (abs(stepper4.currentPosition() - cmds[cmd_cur_i].z_goal) <= cmds[cmd_cur_i].z_accuracy) {
        // add_data_to_send(F("Finish with z\n"));

        done_with_cur_idx = true;
        // add_data_to_send(F("finish moving"));
        done_with_cmd();
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
    // add_data_to_send(F("Stepping J2"));
    stepper3.runSpeed();
    moving = true;
  }
  if (stepper4.currentPosition() != cmds[cmd_cur_i].z_goal) {
    stepper4.runSpeed();
    moving = true;
  }

  if (!moving || done_with_cur_idx){
    // add_data_to_send(F("Updating position"));
    theta1 = STEPS_TO_DEG_THETA1(stepper2.currentPosition());
    theta2 = STEPS_TO_DEG_THETA2(stepper3.currentPosition());
    phi = STEPS_TO_DEG_PHI(stepper1.currentPosition());
    z = STEPS_TO_MM_Z(stepper4.currentPosition());
  }
  return done_with_cur_idx;
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();                  // get one character
  }
}

void publish_heartbeat() {
  static unsigned long counter = HEARTBEAT_INTERVAL-1;
  counter++;
  if (counter>HEARTBEAT_INTERVAL) {
    counter = 0;
    add_data_to_send(F("HEARTBEAT "));
    add_data_to_send(String(theta1));
    add_data_to_send(F(" "));
    add_data_to_send(String(theta2));
    add_data_to_send(F(" "));
    add_data_to_send(String(phi));
    add_data_to_send(F(" "));
    add_data_to_send(String(z));
    add_data_to_send(F(" "));
    add_data_to_send(String(gripper_value));
    add_data_to_send(F("\n"));
  }
}

void publish_done() {
  add_data_to_send(F("DONE\n"));
}

void stop() {
  add_data_to_send(F("Stop not implemented\n"));
}

void send_ready_to_receive(){
  add_data_to_send(READY_TO_RECEIVE_STR);
}

void add_data_to_send(String s){
  // Serial.print("string to add to string_send: ");
  // Serial.println(s);
  string_send_size += s.length();
  string_send += s;
  // Serial.print("string_send: ");
  // Serial.println(string_send);
}

void send_data(){
  static unsigned long counter = 0;
  counter++;
  if (counter < 250)
    return

  counter = 0;
  if (string_send_size){    
    char c = string_send.charAt(0);
    string_send.remove(0, 1);
    if (c == '\n')
      Serial.println();
    else
      Serial.print(c);
    string_send_size--;
  }
}

void debug_print(String s){
  // Print all of string_send and print the debug string
  add_data_to_send(s);
  while (string_send_size){
    send_data();
  }
}

void done_with_cmd() {
  cmd_cur_i = CMD_CUR_I_INC(cmd_cur_i);
  n_cmd_in_storage -= 1;
  // add_data_to_send(F("Finish cmd. Currently: "));
  // add_data_to_send(String(n_cmd_in_storage));
  // add_data_to_send(F(" cmds in storage"));
  // add_data_to_send(F(". Cmd_cur_i: "));
  // add_data_to_send(String(cmd_cur_i));
  // add_data_to_send(F(". Cmd_end_i: "));
  // add_data_to_send(String(cmd_end_i));
  // add_data_to_send(F("\n"));
  publish_done();
}

void homeing() {
  // To ensure slow and safe homeing set the velocity and acceleration to something low
  // TODO: Actually do the above


  add_data_to_send(F("Gonna home stepper 4\n"));
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
  add_data_to_send(F("Finish homing stepper 4\n"));



  // Homing Stepper3
  add_data_to_send(F("Gonna home stepper 3\n"));
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
  add_data_to_send(F("Finish homing stepper 3\n"));

  // Homing Stepper2
  add_data_to_send(F("Gonna home stepper 2\n"));
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
  add_data_to_send(F("Finish homing stepper 2\n"));


  // Homing Stepper1
  // add_data_to_send(F("Gonna home stepper 1\n"));
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
  // add_data_to_send(F("Finish homing stepper 1\n"));
  add_data_to_send(F("At home\n"));
  done_with_cmd();
}
