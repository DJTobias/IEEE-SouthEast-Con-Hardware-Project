// =========================
// Author: Ryan A. Dellana
// Created: May 1, 2016
// Modified: September 13, 2016
//
// This is work-in-progress code written for the Scorpion robot.
// Scorpion was built at North Carolina A&T as our entry to the 2016 IEEE Southeast Con Hardware competition.
// Most of the AI runs in a separate computer running Ubuntu 14.04, while real-time reflexes are handled here.
// =========================

#include <Servo.h> 
#define irSensorPin A0       // IR sensor on this pin
#define irPin2 A1
#include <math.h>
#include <string.h>

Servo br;  // back-right wheel
Servo bl;  // back-left wheel
Servo fl;  // front-left wheel
Servo fr;  // front-right wheel
Servo lshoulder, lelbow, lgrip;
Servo rshoulder, relbow, rgrip;
Servo necktilt, neckyaw;
Servo lfin, rfin;
const int FOLD_TILT = 1700;            // folded down.
const int FOLD_LEFT_SHOULDER = 850;
const int FOLD_RIGHT_SHOULDER = 1950;
const int FOLD_LEFT_ELBOW = 2100;
const int FOLD_RIGHT_ELBOW = 700;
const int LEFT_GRIPPER_OPEN = 1550;    
const int LEFT_GRIPPER_CLOSED = 2150;
const int RIGHT_GRIPPER_OPEN = 600;
const int RIGHT_GRIPPER_CLOSED = 1400;
const int TILT_UP = 820;               // straight up.
const int TILT_DOWN = 1100;            // slightly down.
const int PAN_CENTER = 1500;
const int PAN_RIGHT = 2400;
const int PAN_LEFT = 600;
const int LEFT_SHOULDER_UP = 1380;
const int LEFT_SHOULDER_DOWN = 1900;   // ~30 degrees.
const int RIGHT_SHOULDER_UP = 1450;
const int RIGHT_SHOULDER_DOWN = 900;
const int RIGHT_ELBOW_STRAIGHT = 1050;
const int RIGHT_ELBOW_90 = 1920;
const int RIGHT_ELBOW_DOWN = 2200;     // max down ~110
const int LEFT_ELBOW_STRAIGHT = 1750;  // TODO
const int LEFT_ELBOW_90 = 940;
const int LEFT_ELBOW_DOWN = 1500;      // TODO
const int LEFT_FIN_EXTEND = 2170;
const int RIGHT_FIN_EXTEND = 1040;
const int FOLD_LEFT_FIN = 1350;
const int FOLD_RIGHT_FIN = 1800;
int br_ = 1500, bl_ = 1500, fl_ = 1500, fr_ = 1500;
// current positions:
int lshoulder_ = FOLD_LEFT_SHOULDER, rshoulder_ = FOLD_RIGHT_SHOULDER;
int lelbow_ = FOLD_LEFT_ELBOW, relbow_ = FOLD_RIGHT_ELBOW;
int rgrip_ = RIGHT_GRIPPER_OPEN, lgrip_ = LEFT_GRIPPER_OPEN;
int lfin_ = FOLD_LEFT_FIN, rfin_ = FOLD_RIGHT_FIN;
int necktilt_ = TILT_DOWN, neckyaw_ = PAN_CENTER;
// target positions:
int lshoulder_t = FOLD_LEFT_SHOULDER, rshoulder_t = FOLD_RIGHT_SHOULDER;
int lelbow_t = FOLD_LEFT_ELBOW, relbow_t = FOLD_RIGHT_ELBOW;
int rgrip_t = RIGHT_GRIPPER_OPEN, lgrip_t = LEFT_GRIPPER_OPEN;
int lfin_t = FOLD_LEFT_FIN, rfin_t = FOLD_RIGHT_FIN;
int necktilt_t = TILT_DOWN, neckyaw_t = PAN_CENTER;
int servoSpeed = 10; // pulse-width change in microseconds per refresh
int left_ir_buffer[5] = {7, 7, 7, 7, 7};
int right_ir_buffer[5] = {7, 7, 7, 7, 7};
int buffer_index = 0;
const int switchPin = 42;
const int leftIRPin = A1;
const int rightIRPin = A0; // ???
const int leftGripEmitPin = 40;
const int rightGripEmitPin = 38;
const int leftGripRecievePin = 36;
const int rightGripRecievePin = 32;
const int leftLaserPin = 52;
const int rightLaserPin = 53;

// ------------------ Packet Management -------------------------
const char BEGIN_PACKET = '\t'; const char END_PACKET = '\n';
const int bufferSize = 36; // 32 bytes data, 2 byte checksum
char inputBuffer[bufferSize]; char outputBuffer[bufferSize];
byte bufferIndex = 0; // Current position in input buffer. Logical size of buffer.
boolean unreadMail = false;
byte outputBufferIndexContentEnd = 0;
String inputContent = "";
unsigned long heartbeat = millis();
const int heartbeatTimeout = 120; // 120 miliseconds until PC is considered dead.
boolean commTimeout = false;
int my_id = 42; // valid id may range from 0 -> 94 inclusive.  36 = D  33 = A
boolean bits[6] = {false, false, false, false, false, false};
// ---------------------------------------------------------------
String response = "";

void setup() {
  // ---------------- Packet Management --------------------------
  Serial.begin(9600);
  // -------------------------------------------------------------
  setup_aux();
}

void setup_aux() { 
  pinMode(switchPin, INPUT);
  pinMode(leftGripEmitPin, OUTPUT);
  pinMode(rightGripEmitPin, OUTPUT);
  pinMode(leftGripRecievePin, INPUT);
  pinMode(rightGripRecievePin, INPUT);
  pinMode(leftLaserPin, OUTPUT);
  pinMode(rightLaserPin, OUTPUT);
  digitalWrite(switchPin, HIGH); // activate pullup resistor.
  br.attach(10); bl.attach(11); fl.attach(12); fr.attach(13);
  lshoulder.attach(2); lelbow.attach(7); lgrip.attach(6);
  rshoulder.attach(4); relbow.attach(9); rgrip.attach(8);
  necktilt.attach(3); neckyaw.attach(5);
  lfin.attach(24); rfin.attach(25);
} 

/*
RUN_   # tunnel routine. returns map type. (left-hand / right-hand)
LF__###### (track left wall distance/speed/amount) (returns when complete)
RF__###### (track right wall distance/speed/amount) (...)
SQ__## (square off with wall at distance) (returns true when complete)
DR__ (drop right arm payload) (returns when complete)
DL__ (drop left arm payload) (...)
GR__ (grasping reflex) (returns success/fail)
TANK###### (drive tank left/volt right/volt time) 
STAN###### (sideways tank front/volt back/volt time) right pointing arrow
*/
void loop() {
  
  if (!pollSwitch()) {
    boolean lefty = tunnelRun();
    readyStance();
    boolean laser = false;
    for (int i = 0; i < 10; ++i)
    {
      setLasers(laser);
      laser = !laser;
    }
    setLasers(true);
    refreshServos(200);
    if (lefty) {
      for (int i = 0; i < 4; ++i) {
        follow(-60, 8.0, 100);
        follow(60, 8.0, 100);
      }
    }
    else {
      for (int i = 0; i < 4; ++i) {
        follow(60, 9.0, 100);
        follow(-60, 8.0, 100);
      }    
    }
    driveReverse(60, 50, true);
    lelbow_t = LEFT_ELBOW_STRAIGHT;
    relbow_t = RIGHT_ELBOW_STRAIGHT;
    setLeftShoulder(20);
    setRightShoulder(20);
    refreshServos(100);
    turnRight(100, 150, true);
    for (int i = 0; i < 5; ++i) {
      necktilt_ = TILT_DOWN;
      necktilt_t = TILT_DOWN;
      refreshServos(20);
      necktilt_ = TILT_UP;
      necktilt_t = TILT_UP;
      refreshServos(20); 
    }
    delay(1000000);
  }
  else {
    doNothing();
  }
  /*
  readAvailableBytes();
  if (commTimeout) {
    // what to do with comm timeout.
  }
  if (unreadMail) { // new packet available.
    unreadMail = false;
    response = "";
    if (inputContent.length() == 10) { // validate input packet length.
      String oppcode = inputContent.substring(0,4);
      if (oppcode.equals("RUN_")) {
        // perform tunnel run. Return maze configuration.
        sendPacket("confirm");
        boolean mapType = tunnelRun();
        if (mapType) {
          sendPacket("lefthand");
        }
        else {
          sendPacket("righthand");
        }
      }
      else if (oppcode.equals("LF__")) {
        // follow wall left
        sendPacket("confirm");
        int volt_ = parse95(inputContent.charAt(4), inputContent.charAt(5));
        int time_ = parse95(inputContent.charAt(6), inputContent.charAt(7));
        int dist_ = parse95(inputContent.charAt(8), inputContent.charAt(9));
        double volt = volt_/9024.0;
        double time = time_/9024.0;
        double dist = dist_/9024.0;
        follow(volt*-1, dist, time);
        sendPacket("done");
      }
      else if (oppcode.equals("RF__")) {
        // follow wall right
        sendPacket("confirm");
        int volt_ = parse95(inputContent.charAt(4), inputContent.charAt(5));
        int time_ = parse95(inputContent.charAt(6), inputContent.charAt(7));
        int dist_ = parse95(inputContent.charAt(8), inputContent.charAt(9));
        double volt = volt_/9024.0;
        double time = time_/9024.0;
        double dist = dist_/9024.0;
        follow(volt, dist, time);
        sendPacket("done");
      }
      else if (oppcode.equals("SQ__")) {
        // square off with front wall
        sendPacket("confirm");
        int dist_ = parse95(inputContent.charAt(4), inputContent.charAt(5));        
        double dist = dist_/9024.0;
        square(dist, 0.5, 1.0, 20);
        sendPacket("done");
      }
      else if (oppcode.equals("DR__")) {
        // drop right block
        sendPacket("confirm");
        dropRight();
        sendPacket("done");
      }
      else if (oppcode.equals("DL__")) {
        // drop left block
        sendPacket("confirm");
        dropLeft();
        sendPacket("done");
      }
      else if (oppcode.equals("GR__")) {
        // perform gripping reflex
        sendPacket("confirm");
        boolean result = graspingReflex();
        if (result) {
          sendPacket("success");
        }
        else {
          sendPacket("failure");
        }
      }
      else if (oppcode.equals("TANK")) {
        // drive differential steering
        sendPacket("confirm");
        int lvolt_ = parse95(inputContent.charAt(4), inputContent.charAt(5));
        int rvolt_ = parse95(inputContent.charAt(6), inputContent.charAt(7));
        int time_ = parse95(inputContent.charAt(8), inputContent.charAt(9));
        int lvolt = lvolt_-100; // 100 means zero. 0 means -100
        int rvolt = rvolt_-100;
        int time = time_;
        driveTank(lvolt, rvolt, time, true); // TODO consider optional coasting.
      }
      else if (oppcode.equals("STAN")) {
        // drive differential steering sideways
        sendPacket("confirm");
        int fvolt_ = parse95(inputContent.charAt(4), inputContent.charAt(5));
        int bvolt_ = parse95(inputContent.charAt(6), inputContent.charAt(7));
        int time_ = parse95(inputContent.charAt(8), inputContent.charAt(9));
        int fvolt = fvolt_-100;
        int bvolt = bvolt_-100;
        int time = time_;
        driveTankSideways(fvolt, bvolt, time, true); // TODO see above.
      }
      else {
        // oppcode not recognized
        sendPacket("invalid");
      }
      // Examples:
      //tv_m1 = parse95(inputContent.charAt(0), inputContent.charAt(1)) - 127; // Get motor control values.
      //tv_m2 = parse95(inputContent.charAt(2), inputContent.charAt(3)) - 127;
      //base95ToBits(inputContent.charAt(4), bits);
      //response = response + decDigitTo95(constrain(left_encoder_transition_count, 0, 94));
      //response = response + decDigitTo95(constrain((int)round(l_wheel_velocity*4.0), 0, 94));
      //response = response + decTo95(voltage);
      //response = response + decDigitTo95(constrain(getInches(backLeft), 0, 94));
      //response = response + bitsToBase95(bits);
    }
    if (inputContent.length() == 0)
      sendPacket(decDigitTo95(my_id));
  }
  */
  // </comm> ---------------------------------
  // <events> --------------------------------
  //poll_arm();
  //sonar_events();
  //check_encoders();
  //motor_events();
  //voltage = analogRead(voltage_sensor);
  delayMicroseconds(50);
  // </events> ---------------------------------
}

void base95ToBits(char c, boolean bits[]) {
  int i = b95ToDec(c);
  int d = 32;
  for (int j = 0; j <= 5; ++j) {
    bits[j] = (i / d == 1);
    i = i % d;
    d = d / 2;
  }
}

String bitsToBase95(boolean bits[]) {
  int i = 0;
  int d = 32;
  for (int j = 0; j <= 5; ++j) {
    if (bits[j])
      i = i + d;
    d = d / 2;
  }
  return decDigitTo95(i);
}

// ---------------------------------- Packet Management Functions ---------------------------------
String getContent(byte contentEnd) {
  String content = "";
  for (int i = 1; i <= contentEnd; ++i)
    content.concat(inputBuffer[i]);
  return content;
}
boolean sendPacket(String content) {
  if (content.length() > 32 || content.indexOf(BEGIN_PACKET) != -1 || content.indexOf(END_PACKET) != -1)
    return false;
  int i = 0;
  for (i = 0; i < content.length(); ++i)
    outputBuffer[i + 1] = content.charAt(i);
  outputBufferIndexContentEnd = i;
  sendPacketAux();
  return true;
}
void sendPacketAux() {
  createPacket();
  for (int i = 0; i < sizeof(outputBuffer); ++i) {
    if (outputBuffer[i] == END_PACKET) {
      Serial.write(END_PACKET);
      break;
    }
    Serial.write(outputBuffer[i]);
  }
}
void createPacket() {
  outputBuffer[0] = BEGIN_PACKET;
  int chkSum = 0;
  for (int i = 1; i <= outputBufferIndexContentEnd; ++i)
    chkSum += byte(outputBuffer[i]);
  chkSum = chkSum % 95;
  String chkSum95 = decDigitTo95(chkSum);
  //if (chkSum < 36)
  //  chkSumHex = "0" + chkSumHex;
  outputBuffer[outputBufferIndexContentEnd + 1] = chkSum95.charAt(0);
  // outputBuffer[outputBufferIndexContentEnd + 2] = chkSum95.charAt(1);
  outputBuffer[outputBufferIndexContentEnd + 2] = END_PACKET;
}
// Returns index of end of content if checksum is correct; Otherwise returns -1.
int openPacket() {
  int endPos = 0;
  for (int i = 0; i < sizeof(inputBuffer); ++i) {
    if (inputBuffer[i] == END_PACKET) {
      endPos = i;
      break;
    }
  }
  int chksumIn = b95ToDec(inputBuffer[endPos - 1]);
  int myChecksum = 0;
  for (int i = 1; i < endPos - 1; ++i)
    myChecksum += byte(inputBuffer[i]);
  myChecksum = myChecksum % 95;
  if (myChecksum == chksumIn)
    return (endPos - 2);
  else
    return -1;
}
// takes two sequential base 36 characters and parses them into the equivalent integer value.
int parse95(char byte1, char byte2) {
  return (b95ToDec(byte1) * 95) + b95ToDec(byte2);
}
byte b95ToDec(char c) {
  byte b = byte(c);
  if (b >= 32 && b <= 126)
    return b - 32;
  else
    return 0;
}
// Convert positive integer from 0 -> 1295 into a 2-digit base-36 string.
String decTo95(int i) {
  if (i <= 9024)
    return decDigitTo95(i / 95) + decDigitTo95(i % 95);
  else
    return "00";
}
String decDigitTo95(int d) {
  return String(char(d + 32));
}
void readAvailableBytes() {
  if (millis() - heartbeat > heartbeatTimeout)
    commTimeout = true;
  if (!unreadMail) {
    char inChar;
    while (Serial.available() > 0) {
      inChar = Serial.read();
      if (bufferIndex > 0) { // already reading packet.
        if (inChar == END_PACKET) {
          heartbeat = millis(); // any packet qualifies as a heartbeat.
          commTimeout = false;
          inputBuffer[bufferIndex] = END_PACKET;
          if (bufferIndex == 1) // must be a heartbeat.
            bufferIndex = 0;
          else {
            int contentEnd = openPacket();
            if (contentEnd != -1) { // if packet not rejected.
              inputContent = getContent(contentEnd);
              unreadMail = true;
              bufferIndex = 0;
            }
            else
              packetError();
          }
        }
        else if (bufferIndex >= bufferSize - 1) // buffer overflow.
          packetError();
        else
          inputBuffer[bufferIndex++] = inChar;
      }
      else if (inChar == BEGIN_PACKET) // beginning of packet.
        inputBuffer[bufferIndex++] = BEGIN_PACKET;
    }
  }
}
void packetError() {
  bufferIndex = 0; // flush buffer.
  sendPacket(String("\r")); // let PC know there was a problem.
}
// ------------------------------------------------------------------------------------------------

boolean pollSwitch() {
  boolean state = digitalRead(switchPin);
  delayMicroseconds(100);
  while (state != digitalRead(switchPin)) {
    state = digitalRead(switchPin);
    delayMicroseconds(100);
  }
  return state;
}

void pollGrippers(boolean &left, boolean &right) {
  tone(rightGripEmitPin, 38000, 8);  // TODO consider trying 1 instead of 8. Might work on newer arduino.
  delay(1);
  right = digitalRead(rightGripRecievePin); // TODO add false positive filtration.
  delay(1);
  tone(leftGripEmitPin, 38000, 8);              // IRLED 38 kHz for at least 1 ms
  delay(1);
  left = digitalRead(leftGripRecievePin);
}

void setLasers(boolean val) {
  digitalWrite(leftLaserPin, val);
  digitalWrite(rightLaserPin, val);
}

void driveAll_(int v, int cycles) {
  driveAll_(v, v, v, v, cycles);
}
void driveAll_(int p1, int p2, int p3, int p4, int cycles) {
  for (int i = 0; i < cycles; i++) {
    br.writeMicroseconds(p1);
    bl.writeMicroseconds(p2);
    fl.writeMicroseconds(p3);
    fr.writeMicroseconds(p4);
    delay(15);
  }
}

void driveAll(int v, int cycles, boolean brake) {
  driveAll(v, v, v, v, cycles, brake);
}
void driveAll(int p1, int p2, int p3, int p4, int cycles, boolean brake) {
  driveAll_(map(p1, -100, 100, 1000, 2000), map(p2, -100, 100, 2000, 1000), map(p3, -100, 100, 1000, 2000), map(p4, -100, 100, 2000, 1000), cycles);
  if (brake) {
    int b1 = brakeVal(p1);
    int b2 = brakeVal(p2); 
    int b3 = brakeVal(p3);
    int b4 = brakeVal(p4);
    driveAll_(map(b1, -100, 100, 1000, 2000), map(b2, -100, 100, 2000, 1000), map(b3, -100, 100, 1000, 2000), map(b4, -100, 100, 2000, 1000), 3);
  }
}
int brakeVal(int v) {
  if (v == 0) return 0;
  if (v > 0) return -6;
  return 6;
}

void driveForward(int cycles) {
  driveForward(30, cycles, true);
}
void driveForward(int s, int cycles, boolean brake) {
  driveAll(s, cycles, brake);
}
void driveReverse(int cycles) {
  driveReverse(30, cycles, true);
}
void driveReverse(int s, int cycles, boolean brake) {
  driveAll(s*-1, cycles, brake);
}
void driveRight(int s, int cycles, boolean brake) {
  driveAll(s, s*-1, s, s*-1, cycles, brake);
}
void driveRight(int cycles) {
  driveRight(30, cycles, true);
}
void driveLeft(int s, int cycles, boolean brake) {
  driveAll(s*-1, s, s*-1, s, cycles, brake);
}
void driveLeft(int cycles) {
  driveLeft(30, cycles, true);
}
void turnLeft(int s, int cycles, boolean brake) {
  driveAll(s, s*-1, s*-1, s, cycles, brake); // rotate left
}
void turnLeft(int cycles) {
  turnLeft(30, cycles, true);
}
void turnRight(int s, int cycles, boolean brake) {
  driveAll(s*-1, s, s, s*-1, cycles, brake); // rotate right
}
void turnRight(int cycles) {
  turnRight(30, cycles, true);
}
void driveTank(int left, int right, int cycles, boolean brake) {
  driveAll(right, left, left, right, cycles, brake);
}
// right is considered "forward" here.
void driveTankSideways(int front, int back, int cycles, boolean brake) {
  driveAll(back, back*-1, front, front*-1, cycles, brake);
}

void doNothing() {
  setLasers(true);
  refreshServos(50);
  setLasers(false);
  refreshServos(50);
}

boolean tunnelRun() {
  boolean left_handed = false;
  fold();
  refreshServos(120);
  double left, right;
  for (int i = 0; i < 10; ++i) {
    readIR2(left, right);
  }
  if (left > right) {
    left_handed = true;
  }
  /*
  // drive forward fast until both IRs are obscured.
  while (left > 15.0 || right > 15.0) { // TODO adjust this value <<<<
    driveForward(40, 2, false);
    readIR2(left, right);
  }
  // continue driving forward until one of the IRs is no longer obscured (depends on map handedness).
  while (!(left > 15.0 || right > 15.0)) { // TODO adjust this value <<<<
    driveForward(40, 2, false);
    readIR2(left, right);
  }
  */
  // move a bit further.
  driveForward(60, 165, true);
  // move left/right for a second or two to get into middle of map.
  if (left_handed) {
    driveLeft(46, 50, true);
  }
  else {
    driveRight(60, 50, true);
  }
  return left_handed;
}

void arrayCopy(int arr[], int arr2[], int s) {
  for (int i = 0; i < s; ++i) {
    arr2[i] = arr[i];
  }
}

// 0 means straight up. 100 means as far down as it goes (~30 degrees above horizontal)
void setLeftShoulder(int v) {
  int range = LEFT_SHOULDER_DOWN - LEFT_SHOULDER_UP;
  lshoulder_t = int(LEFT_SHOULDER_UP + range*(v/100.0));
}
// 0 means straight up. 100 means as far down as it goes (~30 degrees above horizontal)
void setRightShoulder(int v) {
  int range = RIGHT_SHOULDER_DOWN - RIGHT_SHOULDER_UP;
  rshoulder_t = int(RIGHT_SHOULDER_UP + range*(v/100.0));
}
// 0 means straight. 100 means 90 degrees bent.
void setLeftElbow(int v) {
  int range = LEFT_ELBOW_90 - LEFT_ELBOW_STRAIGHT;
  lelbow_t = int(LEFT_ELBOW_STRAIGHT + range*(v/100.0));
}
// 0 means straight. 100 means 90 degrees bent.
void setRightElbow(int v) {
  int range = RIGHT_ELBOW_90 - RIGHT_ELBOW_STRAIGHT;
  relbow_t = int(RIGHT_ELBOW_STRAIGHT + range*(v/100.0));
}

boolean graspingReflex() {
  if (positionGrip()) {
    pickUp();
    return true;
  }
  else {
    return false;
  }
}

// returns true if blocks found by ir beams.
boolean positionGrip() {
  graspBeginPose();
  refreshServos(100);
  boolean left, right;
  // move elbows down.
  int ext = 50;
  while (ext <= 100) {
    ext += 1;
    setLeftElbow(ext);
    setRightElbow(ext);
    refreshServos(1);
    pollGrippers(left, right);
    if (left && right) {
      ext += 5;
      setLeftElbow(ext);
      setRightElbow(ext);
      refreshServos(20);
      return true;
    }
  }
  // move shoulders down.
  ext = 0;
  while (ext <= 100) {
    ext += 1;
    setLeftShoulder(ext);
    setRightShoulder(ext);
    refreshServos(1);
    pollGrippers(left, right);
    if (left && right) {
      ext += 5;
      setLeftShoulder(ext);
      setRightShoulder(ext);
      refreshServos(20);
      return true;
    }
  }
  return false;
}

void pickUp() {
  lgrip_t = LEFT_GRIPPER_CLOSED;
  rgrip_t = RIGHT_GRIPPER_CLOSED;
  refreshServos(100);
  setLeftShoulder(0);
  setRightShoulder(0);
  setLeftElbow(0);
  setRightElbow(0);
  refreshServos(100);
}

void dropRight() {
  setRightShoulder(0);
  setRightElbow(100);
  refreshServos(100);
  rgrip_t = RIGHT_GRIPPER_OPEN;
  refreshServos(100);
}

void dropLeft() {
  setLeftShoulder(0);
  setLeftElbow(100);
  refreshServos(100);
  lgrip_t = LEFT_GRIPPER_OPEN;
  refreshServos(100);
}

void gripSensorTest() {
  readyStance();
  refreshServos(1);
  boolean left, right;
  pollGrippers(left, right);
  if (left || right) {
    if (left)
      lgrip_t = LEFT_GRIPPER_CLOSED;
    if (right)
      rgrip_t = RIGHT_GRIPPER_CLOSED;
    refreshServos(100);
    lgrip_t = LEFT_GRIPPER_OPEN;
    rgrip_t = RIGHT_GRIPPER_OPEN;
    refreshServos(100);
  }
}

void fullDemo() {
  servodemo();
  mecanumDemo();
}

void squareDemo() {
  lfin_t = LEFT_FIN_EXTEND;
  rfin_t = RIGHT_FIN_EXTEND;
  refreshServos(100);
  while (!pollSwitch()) {
    double dist, diff;
    readIR(dist, diff);
    if (abs(diff) > 0.8 || abs(dist-9.5) > 1.0)
      square(9.5, 1.0, 1.0, 3);
    driveLeft(40, 10, false);
  }
}

void square(int dist_, int angle_tolerance, int dist_tolerance, int time_thresh) {
  if (lfin_ != LEFT_FIN_EXTEND) {
    lfin_t = LEFT_FIN_EXTEND;
    rfin_t = RIGHT_FIN_EXTEND;
    refreshServos(100);
  }
  int still_count = 0;
  double dist, diff;
  boolean still = false;
  while (still_count < time_thresh) {
    still = false;
    readIR(dist, diff);
    if (dist > dist_+1.0) {
      driveForward(30, 2, false);
    }
    else if (dist < dist_-1.0) {
      driveReverse(30, 2, false);
    }
    else {
      if (abs(diff) > angle_tolerance)
      {
        int magnitude = constrain(int(diff*4.0), 15, 30);
        if (diff > 0.0)
          turnLeft(magnitude, 1, false);
        else
          turnRight(magnitude, 1, false);
      }
      else if (abs(dist - dist_) > dist_tolerance) {
        int dist_diff = dist - dist_;
        int magnitude = constrain(int(abs(dist_diff)*5.0), 15, 30);
        if (dist_diff > 0.0)
          driveForward(magnitude, 1, false);
        else
          driveReverse(magnitude, 1, false);
      }
      else {
        still = true;
        holdStill();
        delay(15);
      }
    }
    if (still) 
      ++still_count;
    else
      still_count = 0;
  }
}

void squareDemo2() {
  double left, right;
  readIR2(left, right);
  int left_speed = int((left-10.0)*10.0)*(abs(left-10.0) > 0.5);
  int right_speed = int((right-10.0)*10.0)*(abs(right-10.0) > 0.5);
  if (left_speed > 0)
    left_speed = constrain(left_speed, 15, 30);
  if (left_speed < 0)
    left_speed = constrain(left_speed, -15, -30);
  if (right_speed > 0)
    right_speed = constrain(right_speed, 15, 30);
  if (right_speed < 0)
    right_speed = constrain(right_speed, -15, -30);
  driveTank(left_speed, right_speed, 1, false);
}

// I know this works for negative voltages (left), but not
// sure about positive voltages (right)
void follow(int volt_, double dist_, double time_) {
  if (lfin_ != LEFT_FIN_EXTEND || rfin_ != RIGHT_FIN_EXTEND) {
    lfin_t = LEFT_FIN_EXTEND;
    rfin_t = RIGHT_FIN_EXTEND;
    refreshServos(100);
  }
  double dist, diff;
  for (int i = 0; i < time_; ++i) {
    readIR(dist, diff);
    if (dist > dist_+1.0) {
      driveForward(40, 1, false);
    }
    else if (dist < dist_-1.0) {
      driveReverse(40, 1, false);
    }
    else {
      driveTankSideways(volt_-int((5.0*diff)), volt_+int((5.0*diff)), 1, false);
    }
    refreshServos(1);
  }
}

void mecanumDemo() {
  driveForward(100);
  driveAll(0, 100, true);
  driveReverse(100);
  driveAll(0, 100, true);
  driveLeft(100);
  driveAll(0, 100, true);
  driveRight(100);
  driveAll(0, 100, true);
  turnLeft(100);
  driveAll(0, 100, true);
  turnRight(100);
  driveAll(0, 100, true);
  driveAll(30, 0, 30, 0, 100, true);
  driveAll(0, 100, true);
  driveAll(-30, 0, -30, 0, 100, true);
  driveAll(0, 100, true);
  driveAll(0, 30, 0, 30, 100, true);
  driveAll(0, 100, true);
  driveAll(0, -30, 0, -30, 100, true);
  driveAll(0, 100, true);
}

void servodemo() {
  // ready position
  readyStance();
  refreshServos(300);
  // raise your hands
  lelbow_t = LEFT_ELBOW_STRAIGHT;
  relbow_t = RIGHT_ELBOW_STRAIGHT;
  refreshServos(200);
  // lower hands
  lelbow_t = LEFT_ELBOW_90;
  relbow_t = RIGHT_ELBOW_90;
  refreshServos(200);
  // make some fists and crack neck.
  rgrip_t = RIGHT_GRIPPER_CLOSED;
  lgrip_t = LEFT_GRIPPER_CLOSED;
  neckyaw_t = PAN_LEFT;
  lfin_t = LEFT_FIN_EXTEND;
  rfin_t = RIGHT_FIN_EXTEND;
  refreshServos(200);
  // relax your fists and look forward
  lgrip_t = LEFT_GRIPPER_OPEN;
  rgrip_t = RIGHT_GRIPPER_OPEN;
  neckyaw_t = PAN_CENTER;
  lfin_t = FOLD_LEFT_FIN;
  rfin_t = FOLD_RIGHT_FIN;
  refreshServos(200);
  // fold up
  lshoulder_t = FOLD_LEFT_SHOULDER;
  rshoulder_t = FOLD_RIGHT_SHOULDER;
  lelbow_t = FOLD_LEFT_ELBOW;
  relbow_t = FOLD_RIGHT_ELBOW;
  lgrip_t = LEFT_GRIPPER_CLOSED;
  rgrip_t = RIGHT_GRIPPER_CLOSED;
  necktilt_t = FOLD_TILT;
  refreshServos(300);
}

void fold() {
  necktilt_t = FOLD_TILT;
  neckyaw_t = PAN_CENTER;
  lshoulder_t = FOLD_LEFT_SHOULDER;
  rshoulder_t = FOLD_RIGHT_SHOULDER;
  lelbow_t = FOLD_LEFT_ELBOW;
  relbow_t = FOLD_RIGHT_ELBOW;
  lgrip_t = LEFT_GRIPPER_CLOSED;
  rgrip_t = RIGHT_GRIPPER_CLOSED;
  lfin_t = FOLD_LEFT_FIN;
  rfin_t = FOLD_RIGHT_FIN;
}

void readyStance() {
  necktilt_t = TILT_UP;
  neckyaw_t = PAN_CENTER;
  lshoulder_t = LEFT_SHOULDER_UP;
  rshoulder_t = RIGHT_SHOULDER_UP;
  lelbow_t = LEFT_ELBOW_90;
  relbow_t = RIGHT_ELBOW_90;
  lgrip_t = LEFT_GRIPPER_OPEN;
  rgrip_t = RIGHT_GRIPPER_OPEN;
  lfin_t = FOLD_LEFT_FIN;
  rfin_t = FOLD_RIGHT_FIN;
}

void graspBeginPose() {
  necktilt_t = TILT_UP;
  neckyaw_t = PAN_CENTER;
  lshoulder_t = LEFT_SHOULDER_UP;
  rshoulder_t = RIGHT_SHOULDER_UP;
  setLeftElbow(50);
  setRightElbow(50);
  lgrip_t = LEFT_GRIPPER_OPEN;
  rgrip_t = RIGHT_GRIPPER_OPEN;
  lfin_t = FOLD_LEFT_FIN;
  rfin_t = FOLD_RIGHT_FIN;
}

void refreshServos(int cycles) {
  servos(neckyaw_t, necktilt_t, lshoulder_t, rshoulder_t, lelbow_t, relbow_t, lgrip_t, rgrip_t, lfin_t, rfin_t, cycles, servoSpeed);
}

void servos(int pan, int tilt, int lsh, int rsh, int lelb, int relb, int lgr, int rgr, int lfi, int rfi, int cycles, int rate) {
  for (int i = 0; i < cycles; i++) {
    necktilt_ = ramp(necktilt_, tilt, rate);
    neckyaw_ = ramp(neckyaw_, pan, rate);
    lshoulder_ = ramp(lshoulder_, lsh, rate);
    lelbow_ = ramp(lelbow_, lelb, rate);
    lgrip_ = ramp(lgrip_, lgr, rate);
    rshoulder_ = ramp(rshoulder_, rsh, rate);
    relbow_ = ramp(relbow_, relb, rate);
    rgrip_ = ramp(rgrip_, rgr, rate);
    lfin_ = ramp(lfin_, lfi, rate);
    rfin_ = ramp(rfin_, rfi, rate);
    servos_(neckyaw_, necktilt_, lshoulder_, rshoulder_, lelbow_, relbow_, lgrip_, rgrip_, lfin_, rfin_);
  }
}

void servos_(int pan, int tilt, int lsh, int rsh, int lelb, int relb, int lgr, int rgr, int lfi, int rfi) {
  necktilt.writeMicroseconds(tilt);
  neckyaw.writeMicroseconds(pan);
  lshoulder.writeMicroseconds(lsh);
  lelbow.writeMicroseconds(lelb);
  lgrip.writeMicroseconds(lgr);
  rshoulder.writeMicroseconds(rsh);
  relbow.writeMicroseconds(relb);
  rgrip.writeMicroseconds(rgr);
  lfin.writeMicroseconds(lfi);
  rfin.writeMicroseconds(rfi);
  holdStill();
  delay(15);
}

int ramp(int current, int target, int inc) {
  if (abs(current - target) <= inc)
     return target;
  if (target > current)
    return current + inc;
  else
    return current - inc;
}

void holdStill() {
  br.writeMicroseconds(1500);
  bl.writeMicroseconds(1500);
  fl.writeMicroseconds(1500);
  fr.writeMicroseconds(1500);
}

// =====================

void readIR(double &dist, double &diff) {
  int ir1 = 0, ir2 = 0;
  irRead(ir1, ir2);
  double res = convertToInches(ir1);
  double res2 = convertToInches(ir2);
  dist = (res+res2)/2.0; // distance in inches
  diff = res-res2; // differential in inches
}

void readIR2(double &left, double &right) {
  int ir1 = 0, ir2 = 0;
  irRead(ir1, ir2);
  left = convertToInches(ir1);
  right = convertToInches(ir2);
}

void irRead(int &left, int &right) {
  left_ir_buffer[buffer_index] = analogRead(leftIRPin);
  right_ir_buffer[buffer_index] = analogRead(rightIRPin);
  buffer_index = (buffer_index + 1)%5;
  int temp[5];
  arrayCopy(left_ir_buffer, temp, 5);
  sort(temp, 5);
  left = temp[2];
  arrayCopy(right_ir_buffer, temp, 5);
  right = temp[2];
}

double convertToInches(double res){
  return 500.0/73 *log(4628/(5*res));
}

// shitty bubble-sort O(n^2)
void sort(int a[], int size) {
  for(int i=0; i<(size-1); i++) {
    for(int o=0; o<(size-(i+1)); o++) {
      if(a[o] > a[o+1]) {
        int t = a[o];
        a[o] = a[o+1];
        a[o+1] = t;
      }
    }
  }
}

