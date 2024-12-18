#include <cmath>
#include "BluetoothSerial.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <WiFi.h>
#include <Wire.h>   

//--------------------------------------------------------VARIABLE DECLARATION--------------------------------------------------------//

//--------------------------------------------------------Bluetooth
BluetoothSerial SerialBT;
int led = 2;
char command;


//--------------------------------------------------------GPS & Compass
HardwareSerial SerialGPS(2);
TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(56789);

//phone's GPS location output variables
String targetLat_str;
String targetLon_str;
float targetLat;
float targetLon;

//car's GPS location output variables
float currentLat;
float currentLon;
bool locValid = false;

//compass bearing, current bearing and difference
float currentBearing;
float targetBearing;
float bearingDiff;
float bearingRange = 20.0;

//target follow distance, angle and current distance
float followDistance = 2.0;
float currentDistance;

//time elapsed
int startTime;
int currentTime;
int elapsedTime;


//--------------------------------------------------------Ultrasonic
const int trigPin = 33;
const int echoPin = 34;
long frontDistance = 1000;
long minFrontDistance = 20;


//--------------------------------------------------------Motor
// Motor A
const int motor1Pin1 = 5;
const int motor1Pin2 = 17;
const int enable1Pin = 16;

//Motor B
const int motor2Pin1 = 18;
const int motor2Pin2 = 19;
const int enable2Pin = 21;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle1 = 200;
int dutyCycle2 = 200;

//-------------------------------------------------------------OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//--------------------------------------------------------FUNCTIONS--------------------------------------------------------//

//--------------------------------------------------------Bluetooth
String getInteFromString(String s){
  String inte;
  int i = 0;
  while(i<s.length() && s[i]!='.'){
    inte += s[i];
    i++;
  }
  return inte;
}


String getDeciFromString(String s){
  String deci;
  int i = 0;
  while(i<s.length() && s[i]!='.'){
    i++;
  }
  i++;
  int j = 0;
  while(i<s.length()){
    deci += s[i];
    i++;
    j++;
  }
  return deci;
}


float stringToInte(String s){
  float result = 0.0;
  int weight = 0;
  int num;
  for(int i = s.length()-1; i>=0; i--){
    num = charToNum(s[i]);
    result += num*pow(10,weight);
    weight += 1;
  }
  return result;
}


float stringToDeci(String s){
  float result = 0.0000000000;
  int weight = -1;
  int num;
  for(int i=0; i<s.length(); i++){
    num = charToNum(s[i]);
    result += num*pow(10,weight);
    weight -= 1;
  }
  return result;
}


int charToNum(char c){
  int code = c;
  switch(code){
    case 48:
      return 0;
      break;
    case 49:
      return 1;
      break;
    case 50:
      return 2;
      break;
    case 51:
      return 3;
      break;
    case 52:
      return 4;
      break;
    case 53:
      return 5;
      break;
    case 54:
      return 6;
      break;
    case 55:
      return 7;
      break;
    case 56:
      return 8;
      break;
    case 57:
      return 9;
      break;
  }
}


float stringToFloat(String s){
  String inte = getInteFromString(s);
  String deci = getDeciFromString(s);
  float result = stringToInte(inte)+stringToDeci(deci);
  return result;
}



//--------------------------------------------------------GPS & Compass

void getCurrentLocation(){
    while (SerialGPS.available() > 0) {
        currentTime = millis();
        elapsedTime = (currentTime - startTime)/1000;
        char c = SerialGPS.read();
        gps.encode(c);
    }
    if (gps.location.isValid()) {
      Serial.print("LAT=");
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
      Serial.println(currentLat, 6);
      Serial.print("LONG=");
      Serial.println(currentLon, 6);
      locValid = true;
      display.print("Latitude = ");
      display.println(currentLat, 6);
      display.print("Longitude = ");
      display.println(currentLon, 6);
      display.display();
    }
    else {
      Serial.println("Not Valid");
      locValid = false;
    //  display.println("Searching for GPS...");
   //   display.println(elapsedTime);
    //  display.display();
  }
  delay(1000);
}


void getCurrentBearing(){
    sensors_event_t event; 
    mag.getEvent(&event);  
    float heading = atan2(event.magnetic.y, event.magnetic.x);

    float declinationAngle = 0.001745;
    heading += declinationAngle;
    
    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI;
    
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
    heading -= 2*PI;
    
    // Convert radians to degrees for readability.
    currentBearing = heading * 180/M_PI;
}

double degreesToRadians(double degrees){
  return degrees*3.141592659/180.0;
}

void followCalculation(){
  double earthRadiusKm = 6371.0;
  
  double latDiff = degreesToRadians(targetLat - currentLat);
  double lonDiff = degreesToRadians(targetLon - currentLon);

  double currentLatRad = degreesToRadians(currentLat);
  double targetLatRad = degreesToRadians(targetLat);

  double a = sin(latDiff/2)*sin(latDiff/2)+sin(lonDiff/2)*sin(lonDiff/2)*cos(currentLatRad)*cos(targetLatRad);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  currentDistance = earthRadiusKm * c * 1000.0;

  float y = sin(lonDiff)*cos(targetLatRad);
  float x = cos(currentLatRad)*sin(targetLatRad)-sin(currentLatRad)*cos(targetLatRad)*cos(lonDiff);

  int targetBearingDeg = atan2(y,x)/M_PI*180;

  targetBearing = (targetBearingDeg + 360) % 360;

  if (targetBearing < 0){
    targetBearing = 360.0 + targetBearing;
  }

  bearingDiff = currentBearing - targetBearing;
}


//--------------------------------------------------------Ultrasonic
void checkFrontDistance(){
  long duration = 100000;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration=pulseIn(echoPin, HIGH);

  frontDistance= duration*0.0342/2;
  //Serial.print("distance=");
  //Serial.println(distance);
  
}


//--------------------------------------------------------Motor
void moveForward(){
  ledcWrite(pwmChannel1, dutyCycle1);
  ledcWrite(pwmChannel2, dutyCycle1);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  delay(100);
}


void panLeft(){
  ledcWrite(pwmChannel1, dutyCycle2);
  ledcWrite(pwmChannel2, dutyCycle1);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  delay(100);
}


void panRight(){
  ledcWrite(pwmChannel1, dutyCycle1);
  ledcWrite(pwmChannel2, dutyCycle2);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  delay(100);
}


void turnLeft(){
  ledcWrite(pwmChannel1, dutyCycle1);
  ledcWrite(pwmChannel2, dutyCycle1);

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  delay(350);
}


void turnRight(){
  ledcWrite(pwmChannel1, dutyCycle1);
  ledcWrite(pwmChannel2, dutyCycle1);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  delay(350);
}


void turnAround(){
  ledcWrite(pwmChannel1, dutyCycle1);
  ledcWrite(pwmChannel2, dutyCycle1);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  delay(700);
}


void turn(){
  ledcWrite(pwmChannel1, dutyCycle2);
  ledcWrite(pwmChannel2, dutyCycle2);

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  delay(100);
}


void stop1(){
  ledcWrite(pwmChannel1, dutyCycle1);
  ledcWrite(pwmChannel2, dutyCycle1);

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void stop(){
  ledcWrite(pwmChannel1, dutyCycle1);
  ledcWrite(pwmChannel2, dutyCycle1);

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}


void turnToTargetBearing(){
   getCurrentLocation();
   getCurrentBearing();
   followCalculation();
   display.clearDisplay();
   display.setCursor(0,0);
   display.print("Current Bearing: ");
   display.println(currentBearing);
   display.print("Target Bearing: ");
   display.println(targetBearing);
   display.display();
   while(bearingDiff>bearingRange){
    turn();
    stop1();
    delay(50);
    if(SerialBT.available()){
                command = SerialBT.read();
                if (command == 'c'){
                    stop();
                    break;
                }
            }
    getCurrentLocation();
    getCurrentBearing();
    followCalculation();
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("------------");
    display.print("Current Bearing: ");
    display.println(currentBearing);
    display.print("Target Bearing: ");
    display.println(targetBearing);
    display.display();
   }
   stop();

}


void goToTarget(){
  getCurrentLocation();
  followCalculation();
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("---goToTarget---");
  display.print("Current Distance: ");
  display.println(currentDistance);
  display.display();
  while(currentDistance>followDistance){
    moveForward();
    if(SerialBT.available()){
                command = SerialBT.read();
                if (command == 'c'){
                    stop();
                    break;
                }
            }
    getCurrentLocation();
    followCalculation();
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("------------");
    display.print("Current Distance: ");
    display.println(currentDistance);
    display.display();
  }
  stop();
}


/*
void goToTarget(){
  getCurrentLocation();
  getCurrentBearing();
  followCalculation();
  //checkFrontDistance();
  display.println("---goToTarget---");
  display.print("Current Bearing: ");
  display.println(currentBearing);
  display.print("Current Distance: ");
  display.println(currentDistance);
  display.display();
  while(currentDistance>followDistance){
    display.println("In the go to target loop");
    display.display();
    if(bearingDiff < bearingRange){
      moveForward();
    }
    else if(bearingDiff <= 90 ){
      panRight();
    }
    else if(bearingDiff>90 && bearingDiff<270){
      turnAround();
    }
    else{
      panLeft();
    }
    getCurrentLocation();
    getCurrentBearing();
    followCalculation();
    //checkFrontDistance();
    if(SerialBT.available()){
      command = SerialBT.read();
      if (command == 'c'){
        stop();
        break;
      }
    }
  }
  display.println("Broken out of the loop");
  display.display();
  stop();
}
*/




//--------------------------------------------------------SETUP--------------------------------------------------------//
void setup(){
  //------------------------------------------------------Bluetooth
    SerialBT.begin("ESP32Car");  //bluetooth device name
    Serial.begin(9600);
    pinMode(led,OUTPUT);

  
  //------------------------------------------------------GPS & Compass
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
    if(!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while(1);
    }
    startTime = millis();
    
  
  //------------------------------------------------------Ultrasonic
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  //Serial.begin(9600);
  
    
  //------------------------------------------------------Motor
  //sets the pins for motors as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  //configure LED PWM functionalitites
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  //attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

//-----------------------------------------------------OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE); 
}

//--------------------------------------------------------LOOP--------------------------------------------------------//
void loop(){
  /*
  while (locValid == false){
    getCurrentLocation();
  }
  */
  
  if(SerialBT.available()){
    command = SerialBT.read();
    Serial.print("Command:");
    Serial.println(command);
    
    //LED
    if(command=='A'){
      digitalWrite(led,HIGH);
      delay(100);
    }
    else if(command=='B'){
      digitalWrite(led,LOW);
    }
    
    //follow
    else if(command=='a'){
      //read location data
      String targetLat_str = "";
      String targetLon_str = "";
      char digit = SerialBT.read();
      while(digit!='x'){
        targetLat_str += digit;
        digit = SerialBT.read();
      }
      
      digit = SerialBT.read();
      while(digit!='x'){
        targetLon_str += digit;
        digit = SerialBT.read();
      }

      //convert to float
      targetLat = stringToFloat(targetLat_str);
      targetLon = stringToFloat(targetLon_str);
      
      //print location data
      Serial.print("targetLat:");
      Serial.println(targetLat,9);
      Serial.print("targetLon:");
      Serial.println(targetLon,9);
      Serial.println("waypoint set");
    }

    else if(command=='b'){
      Serial.println("go to waypoint");
      goToTarget();
    }

    //Remote Control
    else if(command=='d'){
        Serial.println("pan left");
        //checkFrontDistance();
        while(true){
            panLeft();
            //checkFrontDistance();
            if(SerialBT.available()){
                command = SerialBT.read();
                if (command == 'c'){
                    stop();
                    break;
                }
            }
        }
    }
    else if(command=='e'){
        Serial.println("move forward");
        //checkFrontDistance();
        while(true){
            moveForward();
            //checkFrontDistance();
            if(SerialBT.available()){
                command = SerialBT.read();
                if (command == 'c'){
                    stop();
                    break;
                }
            }
        }
    }
    else if(command=='f'){
        Serial.println("pan left");
        //checkFrontDistance();
        while(true){
            panRight();
            //checkFrontDistance();
            if(SerialBT.available()){
                command = SerialBT.read();
                if (command == 'c'){
                    stop();
                    break;
                }
            }
        }
    }
    else if(command=='g'){
        Serial.println("turn left");
        turnLeft();
        stop();
    }
    else if(command=='h'){
        Serial.println("turn around");
        turnAround();
        stop();
    }
    else if(command=='i'){
        Serial.println("turn right");
        turnRight();
        stop();
    }
    else if(command=='j'){
        Serial.println("turn to target bearing");
        turnToTargetBearing();
        stop();
    }
  }
}
