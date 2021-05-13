#include <Wire.h>

// LaDis = laser distance sensor .. I2C SDA A4, SCL A5, VCC +5V, GND
#include <VL53L1X.h>
VL53L1X laDis;

// PCF8574 Pin Expander
#include "PCF8574.h"
#define PCF_ADDR 0x20
PCF8574 pcf(PCF_ADDR);

// Arduino motor shield L293D
#include <AFMotor.h>
#include <Servo.h> 

// DC motors connected to M1, M2
AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
// 5V DC power for stepper motor on M3
AF_DCMotor stepperDC(3);

#define MIN_MOTOR_SPEED 50
#define MAX_MOTOR_SPEED 150
#define MAX_STEPPER_DC 160 // setSpeed(90) reduces the battery 12.14V to 5.07V for stepper motor
int demoMotorSpeed = 0;
int motorSpeed = MIN_MOTOR_SPEED;

// NeoPixel LED Strips
#include <Adafruit_NeoPixel.h>
#define LED_FRONT_PIN 2
#define NR_FRONT_LED 8 // front 8 LEDs strip
#define LED_REAR_PIN A3
#define NR_REAR_LED 2 // rear 2 LEDs

#define BLINK_PERIOD 600 // 600 ms
#define BLINK_ONTIME 300 // 300 ms

Adafruit_NeoPixel stripFront = Adafruit_NeoPixel(NR_FRONT_LED, LED_FRONT_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripRear = Adafruit_NeoPixel(NR_REAR_LED, LED_REAR_PIN, NEO_GRB + NEO_KHZ800); 

// LED color definitions
uint32_t RED, GREEN, BLUE, BRIGHT_WHITE, WHITE, BLACK, ORANGE;

#define OFF 0x0000
#define BLINK_RIGHT 0x0101
#define BLINK_LEFT 0x0202
#define BRAKES 0x0404
#define LIGHTS_1 0x1000
#define LIGHTS_2 0x2000
#define LIGHTS_3 0x4000
#define SOLID 0x00
#define BLINK 0x01
#define BINARY 0x02
#define POLICE 0x03

void setupLED() {
  stripFront.begin();
  stripRear.begin();
  RED = stripFront.Color(0xFF, 0, 0);
  GREEN = stripFront.Color(0, 0xFF, 0);
  BLUE = stripFront.Color(0, 0, 0xFF);
  WHITE = stripFront.Color(0x4F, 0x4F, 0x4F);
  BRIGHT_WHITE = stripFront.Color(0xFF, 0xFF, 0xFF);
  BLACK = 0;
  ORANGE = stripFront.Color(20, 4, 0);
}

static void lightChase(Adafruit_NeoPixel &strip, uint32_t c) {
  strip.clear();
  for(uint16_t i=0; i<strip.numPixels()+4; i++) {
      strip.setPixelColor(i  , c); // Draw new pixel
      strip.setPixelColor(i-4, 0); // Erase pixel a few steps back
      strip.show();
      delay(25); // FIXME no delays allowed
  }
}

static void lightColor(Adafruit_NeoPixel &strip, uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}

static void lightTwoColors(Adafruit_NeoPixel &strip, uint32_t c1, uint32_t c2) {
  int half = strip.numPixels() / 2;
  for(uint16_t i = 0; i < strip.numPixels(); i++) {
    if (i < half) {
      strip.setPixelColor(i, c1);
    }
    else {
      strip.setPixelColor(i, c2);
    }
  }
  strip.show();
}

static void lightFourColors(Adafruit_NeoPixel &strip, uint32_t c1, uint32_t c2, uint32_t c3, uint32_t c4) {
  for(uint16_t i = 0; i < strip.numPixels(); i++) { // cyklus i bude mít hodnoty 0,1,2,3,4,5,6,7
    if (i < 2) {
      strip.setPixelColor(i, c1);
    }
    else if (i < 4) {
      strip.setPixelColor(i, c2);
    }
    else if (i < 6) {
      strip.setPixelColor(i, c3);
    }
    else {
      strip.setPixelColor(i, c4);
    }
  }
  strip.show();
}

static void lightBinary(Adafruit_NeoPixel &strip, uint32_t c, byte value) {
  int val = value;
  for(uint16_t i = 0; i < strip.numPixels(); i++) {
    
    if (val & 0x01) {
        //Serial.print(1);
        strip.setPixelColor(i, c);
    }
    else {
        //Serial.print(0);
        strip.setPixelColor(i, BLACK);
    }
    val = val >> 1;
  }
  strip.show();
}

static void lightThreeColors(Adafruit_NeoPixel &strip, uint32_t c1, uint32_t c2, uint32_t c3) {
  for(uint16_t i = 0; i < strip.numPixels(); i++) { // cyklus i bude mít hodnoty 0,1,2,3,4,5,6,7
    if (i == 2 || i == 5) {
      strip.setPixelColor(i, 0);
    }
    if (i < 2) {
      strip.setPixelColor(i, c1);
    }
    else if (i < 5) {
      strip.setPixelColor(i, c2);
    }
    else {
      strip.setPixelColor(i, c3);
    }
  }
  strip.show();
}

static uint32_t blinkColor(uint32_t color1, uint32_t color2) {
  bool lightOn = (millis() % BLINK_PERIOD) < BLINK_ONTIME;
  if (lightOn) {
    return color1;
  }
  else {
    return color2;
  }
}

static void lightMeter(Adafruit_NeoPixel &strip, uint32_t c, int d) {
  int ledCount = (2 * d * strip.numPixels()) / 200; // magic formula to calculate the number of LEDS
  //Serial.println(ledCount);
  for(uint16_t i = 0; i < strip.numPixels(); i++) {
    if (i < ledCount) {
      strip.setPixelColor(i, c);
    }
    else {
      strip.setPixelColor(i, 0);
    }
  }
  strip.show();
}

byte readODOState() {
  byte result = pcf.read8();
  return result;
}

byte readTerminalSwitchState() {
  return pcf.read(4);
}

int readLaDis() {
  int result = laDis.read();
  // FIXME do not measure always, check the time period first
  return result;
}

int clampMotorSpeed(int mSpeed) {
  int result = mSpeed;
  if (int mSpeed = 0) {
    result = 0;
  }
  else if (result > MAX_MOTOR_SPEED) {
    result = MAX_MOTOR_SPEED;
  }
  else if (result < MIN_MOTOR_SPEED) {
    result = MIN_MOTOR_SPEED;
  }
  return result;
}

void setLRMotors(int leftDir, int rightDir, int leftSpeed, int rightSpeed) {
  motorL.run(leftDir);
  motorR.run(rightDir);
  motorL.setSpeed(clampMotorSpeed(leftSpeed));
  motorR.setSpeed(clampMotorSpeed(rightSpeed));
}

void setLRMotors(int leftDir, int rightDir, int bothSpeed) {
  setLRMotors(leftDir, rightDir, bothSpeed, bothSpeed);
}

void setupMotors() {
  // init DC motors M1, M2
  motorL.setSpeed(0);
  motorL.run(RELEASE);
  motorR.setSpeed(0);
  motorR.run(RELEASE);
  // stepper motor DC on
  stepperDC.setSpeed(MAX_STEPPER_DC);
  stepperDC.run(RELEASE);
}
  
// stepper motor and driver
#define STEP_IN1 A0 // stepper driver pin in1 is connected to expander pin 4
#define STEP_IN2 A1 
#define STEP_IN3 A2 
#define STEP_IN4 A3 

// stepper one turn clockwise
void stepperCW() {
  for (int k = 0; k < 8; k++) {
    step(k);
  }
}

// stepper one turn counter clockwise
void stepperCCW() {
  for (int k = 7; k >= 0; k--) {
    step(k);
  }
}

// stepper stop
void stepperStop() {
  stepperWrite(LOW, LOW, LOW, LOW);
}

void stepperWrite(int n1, int n2, int n3, int n4) {
  digitalWrite(STEP_IN1, n1);
  digitalWrite(STEP_IN2, n2);
  digitalWrite(STEP_IN3, n3);
  digitalWrite(STEP_IN4, n4);
}

// stepper control sequence and delay can differ each vendor
void step(int step_nr){
  switch (step_nr) {
    case 0:
      stepperWrite(LOW, LOW, LOW, HIGH); // 0001
      break;
    case 1:
      stepperWrite(LOW, LOW, HIGH, HIGH); // 0011
      break;
    case 2:
      stepperWrite(LOW, LOW, HIGH, LOW); // 0010
      break;
    case 3:
      stepperWrite(LOW, HIGH, HIGH, LOW); // 0110
      break;
    case 4:
      stepperWrite(LOW, HIGH, LOW, LOW); // 0100
      break;
    case 5:
      stepperWrite(HIGH, HIGH, LOW, LOW); // 1100
      break;
    case 6:
      stepperWrite(HIGH, LOW, LOW, LOW); // 1000
      break;
    case 7:
      stepperWrite(HIGH, LOW, LOW, HIGH); // 1001
      break;
  }
  delayMicroseconds(800);
}

void batteryPerformance() {
      // driveState changes every 500ms, repetition period is 5s (states 0..9)
      int seconds = millis() / 500;
      int driveState = seconds % 10;
      switch (driveState) {
        case 0: { // sleep
          break;
        }
        case 1: {
          demoMotorSpeed = clampMotorSpeed(demoMotorSpeed++);
          //setLRMotors(FORWARD, BACKWARD, demoMotorSpeed);
          setLRMotors(FORWARD, FORWARD, demoMotorSpeed);
          break;
        }
        case 2: {
          break;
        }
        case 3: {
          demoMotorSpeed = clampMotorSpeed(demoMotorSpeed--);
          //setLRMotors(FORWARD, BACKWARD, demoMotorSpeed);
          setLRMotors(FORWARD, FORWARD, demoMotorSpeed);
          break;
        }
        case 4: { // sleep
          break;
        }
        case 5: {
          demoMotorSpeed = clampMotorSpeed(demoMotorSpeed++);
          //setLRMotors(BACKWARD, FORWARD, demoMotorSpeed);
          setLRMotors(BACKWARD, BACKWARD, demoMotorSpeed);
          break;
        }
        case 6: { 
          break;
        }
        case 7: {
          demoMotorSpeed = clampMotorSpeed(demoMotorSpeed--);
          //setLRMotors(BACKWARD, FORWARD, demoMotorSpeed);
          setLRMotors(BACKWARD, BACKWARD, demoMotorSpeed);
          break;
        }
        case 8: { // sleep;
          break;
        }        
      }
}

void setupStepper() {
  pinMode(STEP_IN1, OUTPUT);
  pinMode(STEP_IN2, OUTPUT);
  pinMode(STEP_IN3, OUTPUT);
  pinMode(STEP_IN4, OUTPUT);
}

void setupLaDis() {
  laDis.setTimeout(500);
  if (!laDis.init()) {
    Serial.println("Failed to detect and initialize Laser Distance Sensor!");
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  laDis.setDistanceMode(VL53L1X::Long);
  laDis.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  laDis.startContinuous(50);
}

#define HELP_00 "  See the readme.md for more advanced controls."
/*
 * Commented out, as it takes too much memory.
 * 
#define HELP_01 "  ---=== HELP - Simple Controls ===---"
#define HELP_02 "  0       .. STOP ALL (motors, lift, lights)"
#define HELP_03 "  w,s,a,d .. forward, backward, left, right"
#define HELP_04 "  +-      .. increase/decrease DC motor speed"
#define HELP_05 "  e,q     .. lift one floor UP/DOWN"
#define HELP_06 "  <space> .. STOP all motors (DC L/R, stepper)"
#define HELP_07 "  1,2,3,4 .. light modes [POLICE, GREEN FLASH, BRIGHT WHITE, BLUE BIN 0xAA]"
#define HELP_08 "  5       .. lights off"
#define HELP_09 "  l,k     .. La(ser)Dis(tance) Measurement ON/OFF"
#define HELP_10 "  i       .. toggle debug lights"
#define HELP_11 ""
#define HELP_12 "  ---=== HELP - Advanced Controls ===---"
#define HELP_13 "  #STOP#  .. STOP ALL (motors, lift, lights)"
#define HELP_14 "  #M([F|B],[F|B],0x00,0x00)#  .. Set motors direction (F/B) and speed (HEX 0x00 .. 0xFF)"
#define HELP_15 "                                 M(leftDir,rightDir,leftSpeed,rightSpeed)"
#define HELP_16 "                                 The speed is clamped to a range min = 50, max = 150."
#define HELP_17 "                                 M(F,B,0x50,0xFF) .. left motor FORWARD speed 80, right motor BACKWARD speed 255"
#define HELP_18 ""
#define HELP_19 ""
*/
void info() {
  Serial.println("/* ");
  Serial.println(HELP_00);
  /*
  Serial.println(HELP_01);
  Serial.println(HELP_02);
  Serial.println(HELP_03);
  Serial.println(HELP_04);
  Serial.println(HELP_05);
  Serial.println(HELP_06);
  Serial.println(HELP_07);
  Serial.println(HELP_08);
  Serial.println(HELP_09);
  Serial.println(HELP_10);
  Serial.println(HELP_11);
  Serial.println(HELP_12);
  Serial.println(HELP_13);
  Serial.println(HELP_14);
  Serial.println(HELP_15);
  Serial.println(HELP_16);
  Serial.println(HELP_17);
  Serial.println(HELP_18);
  Serial.println(HELP_19);
  */
  Serial.println(" */");
}

#define STATUS_REFRESH 100 // Status report refresh period in ms
#define LADIS_REFRESH 200 // LaDis refresh period in ms

long lTimestamp = 0; // last timestamp
long laDisTimestamp = 0;

void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps
  info();
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  setupLaDis(); // setup Laser Distance Sensor
  pcf.begin();  // init PCF8574
  setupLED();
  setupMotors();
  setupStepper();
  delay(500);
  lTimestamp = millis();
  //startDemo();
}

// GLOBAL Variables
#define DEMO_MAX_STATE 6 // the last state to reach
#define DEMO_SECONDS 5

// drive mode
bool demo = false;
unsigned int state = 100; // drive mode
//int state = 99; // battery test
unsigned long demoStartTime = 0;

int lastState = 999;
String message = "";
byte ledMode = SOLID;
uint32_t ledColor;
byte ledValue;
bool debugLights = false;
int liftDirection = 0;
int laserDistance = 0;
bool laDisEnabled = false;
byte sensorODO = 0;
byte lastODO = 0;
int odoCntL = 0;
int odoCntR = 0;

#define LIFT_TOP_POSITION 1024
int liftPosition = 0;

void say(String msg) {
  Serial.print("/* ");
  Serial.print(msg);
  Serial.print(" */");
  Serial.println();
}

void sayCmd(char cmd, String action) {
  Serial.print("/* Command: '");
  Serial.print(cmd);
  Serial.print("' .. ");
  Serial.print(action);
  Serial.print(" */");
  Serial.println();
}

void sayStatus() {
  Serial.print("/*");
  Serial.print(" Motor speed: ");
  Serial.print(motorSpeed);
  
  Serial.print(" Lift direction, position: ");
  Serial.print(liftDirection);
  
  Serial.print(", ");
  Serial.print(liftPosition);
  
  Serial.print(" ODO & Switch: ");
  Serial.print(sensorODO, BIN);

  Serial.print(" ODO_LeftRight(");
  Serial.print(odoCntL);
  Serial.print(",");
  Serial.print(odoCntR);
  Serial.print(")");
  
  Serial.print(" La(ser)Dis(tance): ");
  Serial.print(laserDistance);
  
  Serial.println(" */");
}

void sayStatusBIN() {
  Serial.print("Lift(");
  Serial.print(liftDirection, DEC);
  Serial.print(",");
  Serial.print(liftPosition, DEC);
  Serial.print(")");
  
  Serial.print(" ODO(");
  Serial.print(odoCntL);
  Serial.print(",");
  Serial.print(odoCntR);
  Serial.print(")");
  
  Serial.print(" LaDis(");
  Serial.print(laserDistance, DEC);
  Serial.print(")");
  
  Serial.println();
}

void startDemo() {
  demo = true;
  state = 0;
  demoStartTime = millis();  
}

void stopDemo() {
  demo = false;
  state = 100;
}

long getDemoMillis() {
  return (millis() - demoStartTime);
}

void shutdownAll() {
  shutdownLights();
  shutdownMotors();
}

void shutdownLights() {
  ledMode = SOLID;
  ledColor = BLACK;
  ledValue = 0;
  lightColor(stripFront, BLACK);
  //lightColor(stripRear, BLACK);
}

void shutdownMotors() {
  demoMotorSpeed = 0;
  motorL.run(RELEASE);
  motorR.run(RELEASE);
  stepperStop();
  stepperDC.run(RELEASE);  
}

void lightsHandler() {
  switch (ledMode) {
    case POLICE:
      lightTwoColors(stripFront, blinkColor(RED, BLACK), blinkColor(BLACK, BLUE)); // Police
      break;
    case BLINK:
      lightColor(stripFront, blinkColor(0, ledColor)); // Blinking white front lights
      break;
    case SOLID:
      lightColor(stripFront, ledColor);
      break;
    case BINARY:
      lightBinary(stripFront, ledColor, ledValue);
      break;
  }
}

void liftHandler() {
  if ((liftDirection < 0) && (readTerminalSwitchState() == 0)) {
    liftDirection = 0; // stop the lift on bottom position
    liftPosition = 0;
  }
  if ((liftDirection > 0) && (liftPosition >= LIFT_TOP_POSITION)) {
    liftDirection = 0; // stop the lift on top position
  }
  if (liftDirection == 0) {
    stepperStop();
    stepperDC.run(RELEASE);
  }
  else {
    stepperDC.run(BACKWARD);
  }
  if (liftDirection > 0) {
    // UP
    stepperCW();
    liftPosition++;
  }
  if (liftDirection < 0) {  
    // DOWN
    stepperCCW();
    liftPosition--;
  }  
}

int getODOIncDec(byte value) {
  // 013201320132
  switch (value) {
    case 0x01: 
    case 0x13: 
    case 0x32: 
    case 0x20: 
      return -1;
    case 0x02: 
    case 0x23: 
    case 0x31: 
    case 0x10:
      return 1;
  }
  return 0;
}

void handleODO() {
  lastODO = sensorODO;
  sensorODO = readODOState();
  /*Serial.print("last ODO: ");
  Serial.print(lastODO, BIN);
  Serial.print(" current ODO: ");
  Serial.print(sensorODO, BIN);
  Serial.println();
  Serial.println(((lastODO & 0x03) << 4) | (sensorODO & 0x03), BIN);*/
  odoCntR += getODOIncDec(((lastODO & 0x03) << 4) | (sensorODO & 0x03));
  odoCntL -= getODOIncDec(((lastODO & 0x0C) << 2) | ((sensorODO & 0x0C) >> 2)); // reverse direction -= for the left wheel
}

void loopHandler() {
  switch (state) {
    case 0: { // idle
      message = "Hello Robot!";
      break;
    }
    case 1: { // Police
      message = "This is Police";
      lightTwoColors(stripFront, blinkColor(RED, BLACK), blinkColor(BLACK, BLUE)); // Police
      break;
    }
    case 2: { // front white lights
      message = "Let the lights shine!";
      lightColor(stripFront, WHITE); // White front lights on
      break;
    }
    case 3: { // ODO meter, sensors
      message = "ODO meter, sensors";
      handleODO();
      lightBinary(stripFront, RED, sensorODO);
      break;
    }
    case 4: { // Move the rover
      message = "Move the rover";
      int seconds = getDemoMillis() / 1000;
      int driveState = seconds % DEMO_SECONDS;
      switch (driveState) {
        case 0: {
          demoMotorSpeed = clampMotorSpeed(demoMotorSpeed++);
          setLRMotors(FORWARD, BACKWARD, demoMotorSpeed);
          break;
        }
        case 1: {
          demoMotorSpeed = clampMotorSpeed(demoMotorSpeed--);
          setLRMotors(FORWARD, BACKWARD, demoMotorSpeed);
          break;
        }
        case 2: {
          break;
        }
        case 3: {
          demoMotorSpeed = clampMotorSpeed(demoMotorSpeed++);
          setLRMotors(BACKWARD, FORWARD, demoMotorSpeed);
          break;
        }
        case 4: {
          demoMotorSpeed = clampMotorSpeed(demoMotorSpeed--);
          setLRMotors(BACKWARD, FORWARD, demoMotorSpeed);
          break;
        }        
      }
      break;
    }
    case 5: { // lift up and down
      stepperDC.run(BACKWARD);
      message = "Lift UP/DOWN";
      int seconds = getDemoMillis() / 1000;
      int driveState = seconds % DEMO_SECONDS;
      switch (driveState) {
        case 0: // move UP
        case 1:
          stepperCW();
          break;
        case 2: // stop
          stepperStop();
          break;
        case 3: // move DOWN
        case 4:
          stepperCCW();
          break;
      }
      break;
    }
    case 6: { // Laser Distance measurement
      message = "Laser distance measurement";
      laserDistance = readLaDis();
      lightMeter(stripFront, ORANGE, laserDistance);
      break;
    }
    case 99: { // battery performance
      message = "Battery performance";
      lightTwoColors(stripFront, blinkColor(GREEN, BLACK), blinkColor(BLACK, GREEN));
      batteryPerformance();
      break;
    }
    case 100: {
      message = "Drive mode FULL control";
      lightsHandler();
      liftHandler();
      handleODO();
      if ((laDisEnabled) && ((lTimestamp - laDisTimestamp) > LADIS_REFRESH)) { // do not read the distance too often
        laserDistance = laDis.read();
        laDisTimestamp = lTimestamp;
      }
      if (debugLights) {
        lightBinary(stripFront, RED, sensorODO);
      }
      break;
    }
    /*
    case 110: { // Custom lights
      message = "Drive mode Lights";
      lightsHandler();
      break;
    }
    case 120: { // lift UP/DOWN
      message = "Drive mode Lift UP/DOWN";
      liftHandler();
      break;
    }
    case 130: { // Robot FORWARD/BACKWARD/LEFT/RIGHT
      message = "Drive mode Robot FORWARD/BACKWARD/LEFT/RIGHT";
      break;
    }
    case 140: { // Laser Distance Sensor
      handleODO();
      if ((laDisEnabled) && ((lTimestamp - laDisTimestamp) > LADIS_REFRESH)) { // do not read the distance too often
        laserDistance = laDis.read();
        laDisTimestamp = lTimestamp;
      }
      lightBinary(stripFront, RED, sensorODO);
    }
    */
  }
}

bool cmdReading = false;
String cmdStr = "";

int StrToHex(char str[]) {
  return (int) strtol(str, 0, 16);
}

int charToDir(char ch) {
  return ch == 'F' ? FORWARD : BACKWARD;
}

void loopController() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmdReading) {
      if (cmd == '#') { // advanced command #...# END
        cmdReading = false;
      }
      else {
        cmdStr += cmd;
        return;
      }
    }
    else {
      if (cmd == '#') { // advanced command #...# START
        cmdReading = true;
        cmdStr = "";
        return;
      }
    }
    /*if (cmd == '#') {
      say("Advanced Command");
      say(cmdStr);
      return;
    }*/
    if (cmdStr[0] == 'M') {
      // #M(F,B,0x10,0x10)#
      // #M(F,F,0xFF,0xFF)#
      // #M(B,B,0x50,0x50)#
      char temp[3];
      temp[2] = '\0';
      strncpy(temp, &cmdStr[8], 2);
      int leftSpeed = (int) strtol(temp, 0, 16);
      strncpy(temp, &cmdStr[13], 2);
      int rightSpeed = (int) strtol(temp, 0, 16);
      setLRMotors(charToDir(cmdStr[2]), charToDir(cmdStr[4]), leftSpeed, rightSpeed);   
    }
    if (cmd == '0' || cmdStr == "STOP") {
      
      sayCmd(cmd, "Stop ALL");
      shutdownAll();
      liftDirection = 0;
      stopDemo();
    }
    if (cmd == ' ') {
      sayCmd(cmd, "Stop Motors");
      shutdownMotors();
      liftDirection = 0;
      stopDemo();
    }
    if (cmd == '1') {
      sayCmd(cmd, "Light mode POLICE");
      state = 100;
      ledMode = POLICE;
    }
    if (cmd == '2') {
      sayCmd(cmd, "Light mode GREEN blink");
      state = 100;
      ledMode = BLINK;
      ledColor = GREEN;
    }
    if (cmd == '3' ) {
      sayCmd(cmd, "Light mode Full bright WHITE");
      state = 100;
      ledMode = SOLID;
      ledColor = WHITE;
    }
    if (cmd == '4' ) {
      sayCmd(cmd, "Light mode BLUE BIN 0xAA");
      state = 100;
      ledMode = BINARY;
      ledColor = BLUE;
      ledValue = 0xAA;
    }
    if (cmd == '5' ) {
      sayCmd(cmd, "Lights OFF");
      state = 100;
      shutdownLights();
    }
    if (cmd == '+' ) {
      motorSpeed = clampMotorSpeed(motorSpeed + 10);      
    }
    if (cmd == '-' ) {
      motorSpeed = clampMotorSpeed(motorSpeed - 10);
    }
    if (cmd == 'e') {
      sayCmd(cmd, "Lift UP");
      state = 100;
      liftDirection = 1;
    }
    if (cmd == 'q') {
      sayCmd(cmd, "Lift DOWN");
      state = 100;
      liftDirection = -1;
    }
    if (cmd == 'w') {
      sayCmd(cmd, "Robot FORWARD");
      state = 100;
      setLRMotors(FORWARD, FORWARD, motorSpeed);
    }
    if (cmd == 's') {
      sayCmd(cmd, "Robot BACKWARD");
      state = 100;
      setLRMotors(BACKWARD, BACKWARD, motorSpeed);
    }
    if (cmd == 'a') {
      sayCmd(cmd, "Robot LEFT");
      state = 100;
      setLRMotors(BACKWARD, FORWARD, motorSpeed);
    }
    if (cmd == 'd') {
      sayCmd(cmd, "Robot RIGHT");
      state = 100;
      setLRMotors(FORWARD, BACKWARD, motorSpeed);
    }
    if (cmd == 'l') {
      sayCmd(cmd, "Laser Distance Sensor ENABLED");
      state = 100;
      laDisEnabled = true;
    }
    if (cmd == 'k') {
      sayCmd(cmd, "Laser Distance Sensor DISABLED");
      state = 100;
      laDisEnabled = false;
    }
    if (cmd == 'i') {
      sayCmd(cmd, "Toggle Debug LIGHTS");
      state = 100;
      debugLights = !debugLights;
    }
    if (cmd == 'x') {
      sayCmd(cmd, "DEMO mode");
      shutdownAll();
      startDemo();
    }
    if (!cmdReading) {
    cmdStr = "";
    }
    sayStatus();  
  }
}

void loop() {
  if (demo) {
    if (lastState != state) {
      shutdownAll();
    }
  }
  
  loopController();
  loopHandler();

  if (demo) {
    if (lastState != state) {
      say(message);
    }
  }
  lastState = state;
  if (demo) {    
    state = ((getDemoMillis() / (DEMO_SECONDS * 1000)) % (DEMO_MAX_STATE + 1));
  }

  long t = millis();
  if ((t - lTimestamp) > STATUS_REFRESH) {
    sayStatusBIN();
    lTimestamp = t;
  }
}
