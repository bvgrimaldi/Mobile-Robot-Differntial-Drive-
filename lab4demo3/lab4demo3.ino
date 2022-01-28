

/*
 * MEAM510 Lab 4 demo
 * March 2021
 * 
 * Base code:Mark Yim, Modifications for project: Brian Grimaldi
 * University of Pennsylvania
 * copyright (c) 2021 All Rights Reserved
 */
#include <WiFi.h>
#include "html510.h"
#include "joyJS.h"
#include "tankJS.h"
#include "Adafruit_VL53L0X.h"
#include <ESP32Servo.h>

WiFiServer server(80);
const char* ssid     = "#Skyroam_1t9";
const char* password = "55687127";

//const char* ssid     = "Fios-8HKzr";
//const char* password = "plow823thud34mrs";

const char *body;

Servo leftServo;
int leftServoAngle;
Servo rightServo;
int rightServoAngle;

int servoOpen = 0;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();



/********************/
/* HTML510  web   */
void handleFavicon(){
  sendplain(""); // acknowledge
}

void handleRoot() {
  sendhtml(body);
}

void handleSwitch() { // Switch between JOYSTICK and TANK mode
  String s="";
  static int toggle=0;
  if (toggle) body = joybody;
  else body = tankbody;
  toggle = !toggle;
  sendplain(s); //acknowledge
}

#define RIGHT_CHANNEL0      6 // use first channel of 16  
#define RIGHT_CHANNEL1      7
#define LEFT_CHANNEL2       8
#define LEFT_CHANNEL3       9
#define SERVOPIN1    5
#define SERVOPIN2    10
#define SERVOPIN3    23
#define SERVOPIN4    18
#define SERVOFREQ    60
#define LEDC_RESOLUTION_BITS  12
#define LEDC_RESOLUTION  ((1<<LEDC_RESOLUTION_BITS)-1) 
#define FULLBACK -2547//LEDC_RESOLUTION*10*60/10000
#define SERVOOFF  LEDC_RESOLUTION*15*60/10000
#define FULLFRONT  2547//LEDC_RESOLUTION*20*60/10000

int leftservo, rightservo;

int pd_error =0;

int auto_state = 0;

int switchState = 0;

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {            
  uint32_t duty =  LEDC_RESOLUTION * min(value, valueMax) / valueMax;   
  ledcWrite(channel, duty);  // write duty to LEDC 
}

void updateServos() {
  if(auto_state == 1)
  {
    if(leftservo<0){
      ledcAnalogWrite(LEFT_CHANNEL2, leftservo, LEDC_RESOLUTION);
      ledcAnalogWrite(LEFT_CHANNEL3, 0, LEDC_RESOLUTION);
    }
    else {
      ledcAnalogWrite(LEFT_CHANNEL2, 0, LEDC_RESOLUTION);
      ledcAnalogWrite(LEFT_CHANNEL3, leftservo, LEDC_RESOLUTION);
    }
  
    if(rightservo<0) {
      ledcAnalogWrite(RIGHT_CHANNEL0, 0, LEDC_RESOLUTION); 
      ledcAnalogWrite(RIGHT_CHANNEL1, rightservo, LEDC_RESOLUTION); 
    }
    else  {
      ledcAnalogWrite(RIGHT_CHANNEL0, rightservo, LEDC_RESOLUTION); 
      ledcAnalogWrite(RIGHT_CHANNEL1, 0, LEDC_RESOLUTION); 
    }
  }
  else{
    if(leftservo<0){
      ledcAnalogWrite(LEFT_CHANNEL2, leftservo, LEDC_RESOLUTION);
      ledcAnalogWrite(LEFT_CHANNEL3, 0, LEDC_RESOLUTION);
    }
    else {
      ledcAnalogWrite(LEFT_CHANNEL2, 0, LEDC_RESOLUTION);
      ledcAnalogWrite(LEFT_CHANNEL3, leftservo, LEDC_RESOLUTION);
    }
  
    if(rightservo<0) {
      ledcAnalogWrite(RIGHT_CHANNEL0, rightservo, LEDC_RESOLUTION); 
      ledcAnalogWrite(RIGHT_CHANNEL1, 0, LEDC_RESOLUTION); 

    }
    else  {
      ledcAnalogWrite(RIGHT_CHANNEL0, 0, LEDC_RESOLUTION); 
      ledcAnalogWrite(RIGHT_CHANNEL1, rightservo, LEDC_RESOLUTION); 
    }
  }

  

  leftServo.write(leftServoAngle);
  rightServo.write(rightServoAngle);
  delay(100);
  
  
  
}

/************************/
/* joystick mode  code  */

int leftarm, rightarm;
int x,y;

void handleJoy() {
  int left, right;
  x = getVal(); // from -50 to +50
  y = getVal();
  String s = String(x) + "," + String(y);

  left = x - y;
  right = x + y;

  leftservo = map(left, -50, 50, FULLBACK, FULLFRONT);
  rightservo = map(right, -50, 50, FULLBACK, FULLFRONT); 

  sendplain(s);
  Serial.printf("received X,Y:=%d,%d\n",x,y); 
}

void handleArmdown() {
  // Open/Close Gripper
  Serial.println("gripper command received");

  
  if(servoOpen == 0){
    leftServoAngle = 0;
    rightServoAngle = 72;
    servoOpen = 1;
  }

  else{
    leftServoAngle = 85;
    rightServoAngle = 0;
    servoOpen = 0;
  }
  
  sendplain(""); //acknowledge
}

void handleArmup() {
  // do something? 
  if (auto_state == 0){
    Serial.println("Entering Wall folowing mode");
    auto_state = 1;
  }

  else{
    Serial.println("Exiting Wall folowing mode");
    auto_state = 0;
    leftservo = 0;
    rightservo = 0;

  }
  
  sendplain(""); //acknowledge
}

void handleWallFollow(){

  VL53L0X_RangingMeasurementData_t measure;

  int grabbedCan = 0;

  float touchSensor = digitalRead(27);
  float flexSensor = analogRead(37);
  lox.rangingTest(&measure, false);
  int measureToWall = measure.RangeMilliMeter;

  leftServoAngle = 0;
  rightServoAngle = 0;

  switchState = 0;

  Serial.println(flexSensor);

  //GRAB CAN MODE

  ///////////////////////////////////
  

  if (touchSensor ==0) {
    Serial.println("Switch depressed");
    switchState = 1;
   
  
    
    //open claws fully

    leftServoAngle = 85;
    rightServoAngle = 0;
    
    //Move forward for 50 ms
    leftservo = 50;
    rightservo = 25;
    leftservo = map(leftservo, -50, 50, FULLBACK, FULLFRONT);
    rightservo = map(rightservo, -50, 50, FULLBACK, FULLFRONT); 

    for(int i = 0; i<3;i++){
      updateServos();
      delay(100);
    }

    
    //Close claws
    
    leftServoAngle = 0;
    rightServoAngle = 72;
    
    //Backup??? or turn 180 and move forward, hopefully not hitting wall
    rightservo = -25;
    leftservo = -25;
    leftservo = map(leftservo, -50, 50, FULLBACK, FULLFRONT);
    rightservo = map(rightservo, -50, 50, FULLBACK, FULLFRONT); 

    for(int i=0; i<3; i++)
    {
      updateServos();
      delay(100);
    }

    String s = "Hopefully grabbed can :)";
    sendplain(s);

    auto_state = 0;
    leftservo = 0;
    rightservo = 0;
   }

  ////////////////////////////////////

  //WALL TURNING MODE

  else if (flexSensor < 2600){
  //turn roughly 90 degrees
    Serial.println("flexsensor bent");
    

    
    //Backup car
    leftservo = -25;
    rightservo = -25;
    leftservo = map(leftservo, -50, 50, FULLBACK, FULLFRONT);
    rightservo = map(rightservo, -50, 50, FULLBACK, FULLFRONT); 
    
    for(int i = 0; i<1; i++){
      updateServos();
      delay(30);
    }
    //Turn slight left
  
    leftservo = -25;
    rightservo = 50;
    leftservo = map(leftservo, -50, 50, FULLBACK, FULLFRONT);
    rightservo = map(rightservo, -50, 50, FULLBACK, FULLFRONT); 
    
    for(int i= 0;i<6;i++){
      updateServos();
      delay(100);
    }
    


    leftservo = 0;
    rightservo = 0;

    String s = "Turning...";
    sendplain(s);
  
   }


  //PID CONTROLLER
  /////////////////////////////////////

  else{
    if(measureToWall<180)
      Serial.println("Within wall range");
    
    int d_error = measureToWall - 150;
    int u = .1*(d_error) + 0.4*(d_error- pd_error);
    pd_error = d_error;

    if (grabbedCan == 0){
      
      leftservo = 30 + u;
      rightservo = 30 - u;
    }

    else{
      leftservo = -20 - u;
      rightservo = -20   + u;
      
    }
    leftservo = map(leftservo, -50, 50, FULLBACK, FULLFRONT);
    rightservo = map(rightservo, -50, 50, FULLBACK, FULLFRONT); 
    
    updateServos();
    
  }
  
  
  /////////////////////////////////////
}

/*********************/
/* tank mode  code  */
int leftstate, rightstate;
long lastLeverMs;

void handleLever() {
  leftarm = getVal();
  rightarm = getVal();
  leftstate = getVal();
  rightstate = getVal();
  String s = String(leftarm) + "," + String(rightarm) + "," +
             String(leftstate) + "," + String(rightstate);

 //  if (leftarm) do something?
 //  if (rightarm) do something?
 
  if (leftstate>0)      leftservo =  FULLBACK;
  else if (leftstate<0) leftservo =  FULLFRONT;
  else                  leftservo =  SERVOOFF; 
  
  if (rightstate>0)      rightservo =  FULLFRONT; 
  else if (rightstate<0) rightservo =  FULLBACK; 
  else                   rightservo =  SERVOOFF; 
  
  lastLeverMs = millis(); //timestamp command
  /*sendplain(s);
  Serial.printf("received %d %d %d %d \n",leftarm, rightarm, leftstate, rightstate); // move bot  or something
  */

  if(switchState ==1){
    sendplain("Can grabbed");
  }

  else{
  sendplain(s);
  Serial.printf("received %d %d %d %d \n",leftarm, rightarm, leftstate, rightstate); // move bot  or something
  
  }
}

void setup() 
{
  Serial.begin(115200);

  
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, password);
  WiFi.config(IPAddress(192, 168, 43, 115), // change the last number to your assigned number
              IPAddress(192, 168, 1, 1),
              IPAddress(255, 255, 255, 0));
  while(WiFi.status()!= WL_CONNECTED ) { 
    delay(500); Serial.print("."); 
  }
  Serial.println("WiFi connected"); 
  Serial.printf("Use this URL http://%s/\n",WiFi.localIP().toString().c_str());
  server.begin();                  //Start server
  

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  pinMode(27,INPUT_PULLUP);
  pinMode(32,OUTPUT);
  digitalWrite(32,HIGH);

 //GRIPPER INITIALIZATION

 
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  leftServo.attach(4, 1000, 2000);
  rightServo.attach(25, 1000, 2000);

  
  leftServo.setPeriodHertz(50);      // Standard 50hz servo
  rightServo.setPeriodHertz(50);      // Standard 50hz servo
  

  //closed position of grippers default;

 

 // SLEEP PIN FOR MOTORS
 
  pinMode(14,OUTPUT);
  digitalWrite(14,HIGH);
 
  //SETUP LEFT AND RIGHT MOTORS

  //////////////////////////////////////////
  
  ledcSetup(RIGHT_CHANNEL0, SERVOFREQ, LEDC_RESOLUTION_BITS); // channel, freq, bits
  ledcAttachPin(SERVOPIN1, RIGHT_CHANNEL0);
  ledcSetup(RIGHT_CHANNEL1, SERVOFREQ, LEDC_RESOLUTION_BITS); // channel, freq, bits
  ledcAttachPin(SERVOPIN2, RIGHT_CHANNEL1);
  
  ledcSetup(LEFT_CHANNEL2, SERVOFREQ, LEDC_RESOLUTION_BITS); // channel, freq, bits
  ledcAttachPin(SERVOPIN3, LEFT_CHANNEL2);
  ledcSetup(LEFT_CHANNEL3, SERVOFREQ, LEDC_RESOLUTION_BITS); // channel, freq, bits
  ledcAttachPin(SERVOPIN4, LEFT_CHANNEL3);

  //////////////////////////////////////////

 // HTML510 initialization
 
  attachHandler("/joy?val=",handleJoy);
  attachHandler("/armup",handleArmup);
  attachHandler("/armdown",handleArmdown);
  attachHandler("/switchmode",handleSwitch);
  attachHandler("/lever?val=",handleLever);
  body = joybody;
  
  attachHandler("/favicon.ico",handleFavicon);
  attachHandler("/ ",handleRoot);
  
}



void loop()
{ 

  static long lastWebCheck = millis();
  static long lastServoUpdate = millis();
  uint32_t ms;


  ms = millis();
  if (ms-lastWebCheck > 2){ 
    serve(server,body);    
    lastWebCheck = ms;
  }
  if (ms-lastServoUpdate > 1000/SERVOFREQ) {
    
    if (auto_state == 1){//IN WALL FOLLOW MODE

      handleWallFollow();

    }
    updateServos();
    lastServoUpdate = ms;
    
  }
  
}
