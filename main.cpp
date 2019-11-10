#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Array.h>

Servo myServo;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *clawMotor = AFMS.getMotor(3);
Adafruit_DCMotor *liftMotor = AFMS.getMotor(4);

Array<int, 50> stack;

// Analogue Magnet Sensor Pin 
const int magSensor = 0;

// LEDs
const int redPin = 13;
const int amberPin = 12;
const int greenPin = 11;

// Ultrasonic Sensor
const int trigPin = 2;
const int echoPin = 4;

//const double PI = 3.14159;
const double D = 7.5/100;
const int CAR_WIDTH = 22; // THIS IS KEY

const int RPM_MAX[2] = {40, 18};
const int SPEED[2] = {210, 255};

int INSTR_DELAY = 150;

//Left-Right drop-off distance
int TARGET_DIST = 15;

// Maximum search distance
int MAX_SEARCH_DIST = 150;
/////////////////////////////////////////////////////////

int count = 0;
boolean complete = false;

// search angles
int angleStart = 50;
int angleEnd = 120;
int angleStep = 2; // 1 meh, 2 works very well, 3 works well, 5 not so much
int delay_time = 150;
int data_len = (angleEnd-angleStart)/angleStep + 1;
bool redouble = false;

// changes the orientation of the bot before scanning
bool change_angle = true;
// local search - shortest distance within the range
bool allow_local_search = false;

int angle_thresh = 10;
int d_thresh = 15;

// Select algorithm
int algo = 0; //  0 is short x, 1 is max dx2, 2 is smallpos dx2

//////////////////////////////////////////////////////////

double calculateDistDelay(int distance, int motor=0){
  double num_rots = (distance/(PI*D))/100;
  double RPM_w = RPM_MAX[motor] * SPEED[motor]/255;
  double del_t = num_rots/RPM_w * 60 * 1000; // mins to seconds to milliseconds
  return del_t;
}

double calculateAngleDelay(int angle, int gear_ratio=1, int motor=0){
  angle *= gear_ratio;
  double num_rots = angle/180;
  double RPM_w = RPM_MAX[motor] * SPEED[motor]/255;
  double del_t = num_rots/RPM_w * 60 * 1000; // mins to seconds to milliseconds
  return del_t;
}

void moveForwardBackward(int distance, bool forward){
  int del_t = calculateDistDelay(distance);

  leftMotor->setSpeed(SPEED[0]);
  rightMotor->setSpeed(SPEED[0]);
  
  if (forward == true){
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
  }else{
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
  }
  
  delay(del_t);
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  delay(INSTR_DELAY);
}


void turnRight(int angle, bool forward){
  double distance = CAR_WIDTH * angle * PI/180;
  int del_t = calculateDistDelay(distance);

  leftMotor->setSpeed(SPEED[0]);
    
  if (forward == true){
    leftMotor->run(FORWARD);
  }else{
    leftMotor->run(BACKWARD);
  }

  delay(del_t);
  leftMotor->setSpeed(0);
  delay(INSTR_DELAY);
}

void turnLeft(int angle, bool forward){
  double distance = CAR_WIDTH * angle * PI/180;
  int del_t = calculateDistDelay(distance);

  rightMotor->setSpeed(SPEED[0]);
  
  if (forward == true){
    rightMotor->run(FORWARD);
  }else{
    rightMotor->run(BACKWARD);
  }
  
  delay(del_t);
  rightMotor->setSpeed(0);
  delay(INSTR_DELAY);
}

/////////////////////////////////////////////////////
///// Useful C++/Arduino Utility Functions //////////
/////////////////////////////////////////////////////

void logScan(int i, int distance){
  //Serial.print(i);
  //Serial.print(",");
  //Serial.print(distance);
  //Serial.print(".");
  //Serial.print("\n");
  
  // serial plotter
  Serial.println(distance);
  Serial.print(" ");
}

void printArr(int *arr, int size_arr){
  Serial.print("[");
  for ( int i=0;  i < size_arr;  i++ ){
    Serial.print(arr[i]);
    Serial.print(", ");
  }
  Serial.print("]\n");
}

int *adjDiff(int *array, int size_arr){
  int *dx = new int[size_arr];
  for ( int i=1;  i < size_arr;  i++ ){
    dx[i] = array[i] - array[i-1];
  }
  return dx;
}

int arrSmallPos(int *arr, int size_arr){
  int index = 0;
  int smallest = arrMax(arr, size_arr);
  for ( int i=0;  i < size_arr;  i++ ){
    if ( arr[i] < smallest && arr[i] > 0){
      index = i;
      smallest = arr[index];
    }
  }
  return index;
}

int arrMin(int *arr, int size_arr){
  int index = 0;
  int smallest = arr[index];
  for ( int i=0;  i < size_arr;  i++ ){
    if ( arr[i] < smallest ){
      index = i;
      smallest = arr[index];
    }
  }
  return index;
}

int arrMax(int *arr, int size_arr){
  int index = 0;
  int largest = arr[index];
  for ( int i=0;  i < size_arr;  i++ ){
    if ( arr[i] > largest ){
      index = i;
      largest = arr[index];
    }
  }
  return index;
}

/////////////////////////////////////////////////////

// converts ultrasound angle to left/right
// usAngle => 179 --- 90 --- 0
int angleToDir(int usAngle){
  return 1-2*floor(usAngle/90);
}

int angleToStraight(int usAngle){
  // usAngle => 180 --- 90 --- 0
  // angleToStraight => -90 --- 0 --- 90
  return abs(90 - usAngle);
}

// instruction encoder for storing in stack
void execInstr(int dir, int mag, int pol, bool update=true){
  
  bool forward;
  
  if (pol == 1){
    forward = true;
  }else if (pol == -1){
    forward = false;
  }
  
  if (dir == 0){
    if (mag > MAX_SEARCH_DIST){
      mag = MAX_SEARCH_DIST; 
    }
    moveForwardBackward(mag, forward);
  }else if (dir == -1){
    turnLeft(mag, forward);
  }else if (dir == 1){
    turnRight(mag, forward);
  }
  
  if (update == true){
    stack.push_back(dir);
    stack.push_back(mag);
    stack.push_back(pol);
  }
}

// Function for calculating the distance measured by the Ultrasonic sensor
int ultraDist(){ 
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  int distance = duration*0.0343/2;
  return distance;
}


int *scanView(int start_t, int end_t, int step_size, int delay_time, bool redouble){
  int data_len = (end_t-start_t)/step_size + 1;
  int *data_array = new int[data_len];
  int distance;
  
  // rotates the servo motor from start_t to end_t degrees
  for(int i=start_t; i<=end_t; i+=step_size){  
    int index = (i - start_t)/step_size;

    myServo.write(i);
    // Measure distance from ultrasonic sensor
    distance = ultraDist();
    delay(delay_time);
    
    data_array[index] = distance;

    //if (i < data_len-20){
    logScan(i, distance);
    //}
  }

  if (redouble == true){
    // Repeats the previous lines from 165 to 15 degrees
    for(int i=end_t; i>start_t; i-=step_size){  
      int index = (i - start_t)/step_size;
      
      myServo.write(i);
      distance = ultraDist();
      delay(delay_time);
      
      data_array[index] += distance;
      //average the distance using the previous scan
      data_array[index] /= 2;
      
      //logScan(i, distance);
    }
  }
  delay(500);
  //reset ultrasound position
  myServo.write(90);
  return data_array;
}

void pickUp(){
  // PICK MINE
  // clawMotor close
  clawSwitch(false);
  
  // lift motor up
  liftClaw(true);
}

void dropOff(bool live){
  
  if (live == true){
      turnLeft(170, true);
      moveForwardBackward(TARGET_DIST, true);
  }else{
      turnRight(160, true);
      moveForwardBackward(TARGET_DIST, true);
  }

  // DROP MINE
  // clawmotor open
  clawSwitch(true);
  
  if (live == true){
      moveForwardBackward(TARGET_DIST, false);
      turnLeft(170, false);
  }else{
      moveForwardBackward(TARGET_DIST, false);
      turnRight(160, false);
  }
  
  // lift motor down
  liftClaw(false);
  
}

boolean detectMag(){
  boolean live = false;
  int sensorValue = analogRead(magSensor);
  
  if (sensorValue >= 300){
    live = true;
  }
  return live;
}

void startPos(){
  moveForwardBackward(15, true);
  turnLeft(90, true);
  moveForwardBackward(140, true);
  turnRight(90, true);
}

void endPos(){
  turnRight(90, false);
  moveForwardBackward(140, false);
  turnLeft(90, false);
  moveForwardBackward(15, false);
}

void liftClaw(bool up){
  liftMotor->setSpeed(SPEED[1]);
  if (up == true){
    liftMotor->run(FORWARD);
  }else{
    liftMotor->run(BACKWARD);
  }
  int LIFT_DELAY = 2300;
  delay(LIFT_DELAY);
  liftMotor->setSpeed(0);
}

void clawSwitch(bool open_up){
  clawMotor->setSpeed(SPEED[1]);
  if (open_up == true){
    clawMotor->run(FORWARD);
  }else{
    clawMotor->run(BACKWARD);
  }
  int CLAW_DELAY = 2000;
  delay(CLAW_DELAY);
  clawMotor->setSpeed(0);
}

void setup() {
  
  pinMode(amberPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  
  digitalWrite(amberPin, LOW); 
  digitalWrite(redPin, LOW); 
  digitalWrite(greenPin, HIGH); 
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
  Serial.begin(9600);
  myServo.attach(10); // Defines on which pin is the servo motor attached
  //myServo.write(90);
  AFMS.begin();
  delay(3000);

  // open claw
  clawSwitch(true);
  // go to start position
  startPos();

  delay(500);
}

void loop() {

  if (change_angle == true){
    turnRight(30, true);
  }
  
  count += 1;
  ///////////////SCAN VIEW//////////////
  int *scan = scanView(angleStart, angleEnd, angleStep, delay_time, redouble);
  
  printArr(scan, data_len);
  
  // delta distance
  int *scan_dx = adjDiff(scan, data_len);
  //scan_dx[0] = scan[0];

  printArr(scan_dx, data_len);
  
  // 2nd delta distance
  int *scan_dx2 = adjDiff(scan_dx, data_len); 
  //scan_dx2[0] = 0;
  //scan_dx2[data_len-1] = 0;
  
  printArr(scan_dx2, data_len);

  int id2m;
  // get angle index of maximum second delta
  if (algo == 0){
    id2m = arrMin(scan, data_len);
    //allow_local_search = true;
    angle_thresh = 0;
    d_thresh = 14;
  }else if(algo == 1){
    id2m = arrMax(scan_dx2, data_len);
    allow_local_search = true;
    angle_thresh = 0;
    d_thresh = 15;
  }else if(algo == 2){
    id2m = arrSmallPos(scan_dx2, data_len);
    allow_local_search = true;
    angle_thresh = 0;
    d_thresh = 15;
  }
  //id2m += 1;

  if (allow_local_search == true){
    // Local shortest distance
    int local_search = 4;
    if (id2m >= local_search and id2m <= data_len-1-local_search ){
      int local_dist[2*local_search+1] = {scan[id2m-4], scan[id2m-3], scan[id2m-2], scan[id2m-1], scan[id2m], scan[id2m+1], scan[id2m+2], scan[id2m+3], scan[id2m+4]};
      int local_ind = arrMin(local_dist, 2*local_search+1);
      id2m += local_ind - local_search;
    }
  }
  
  ///////////////////////////////////////

  // For testing
  //delay(500000);
  
  //////////////SEEK MINE////////////////
  int angle_index = id2m;
  int dist = scan[angle_index];
  //angle is the angle given by the ultrasound
  int angle = angleStart + angleStep*angle_index;
  Serial.println(angle);

  angle_thresh += (-angle/45);
  angle += angle_thresh;
  // error in distance for different angles - smallest error at 0
  d_thresh -= angle/45;
  dist -= d_thresh;
  
  ////STATIC WARNING LIGHT//
  digitalWrite(amberPin, HIGH);
  delay(3000);
  digitalWrite(amberPin, LOW); 
  //////////////////

  // TURN
  execInstr(angleToDir(angle), angleToStraight(angle), 1);
  // FORWARD
  execInstr(0, dist, 1);
  /////////////////////////////////////

  // detect mine and check if live - analog input from sensing circuit
  boolean live = false;
  
  for (int i=0; i<10; i++){
    live = detectMag();
    delay(100);
  }
  
  if (live){
    ////RED LIGHT ON//
    digitalWrite(redPin, HIGH);
    //////////////////
  }else{
    ////FLASH GREEN LIGHT//
    digitalWrite(greenPin, LOW);
    delay(500);
    digitalWrite(greenPin, HIGH); 
    //////////////////
  }
  
  pickUp();
  
  // Create LIFO instructions from stack and exec reverse instructions
  for(int i = stack.size()-1; i > -1; i-=3) {     
    execInstr(stack[i-2], stack[i-1], -1*stack[i], false);
    stack.remove(i);
    stack.remove(i-1);
    stack.remove(i-2);
  }

  if (change_angle == true){
    turnRight(30, false);
  }
  
  //(go to left or right box, drop mine and return back to start point)
  dropOff(live);

  if (live){
    ////RED LIGHT OFF//
    digitalWrite(redPin, LOW);
    //////////////////
  }
  
  complete = (count == 5);
  if (complete){
    endPos();
    exit(0);
  }

}
