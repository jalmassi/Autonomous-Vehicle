#include <Servo.h>

#include <QTRSensors.h>

//Justin Almassi 
//Servo things
//===============================================================
#include <Servo.h>
#include <QTRSensors.h>

QTRSensors qtr;
Servo myservo;
int pos = 0;

//===============================================================
// right line sensor: 16, left line sensor: 53
// right light sensor: 15, left light sensor: 14


int Echo2 = A4;
int Trig2 = A5;

int Distance = 0;
int Distance1 = 0;
int Distance2 = 0;

//==============================
int Left_motor_go=8;    
int Left_motor_back=9;    

int Right_motor_go=6;  
int Right_motor_back=7;  

int Right_motor_en=5;  
int Left_motor_en=11;  

/*Set Button port*/
int key=13;


const int lightR = A6;
const int lightL = A7;
const int lightC = A1;

const int lightSideR = A3;
const int lightSideL = A2;


const int rightSensor = 53;
const int leftSensor = 16;

unsigned int lL;
unsigned int rR;
unsigned int cC;

int rSensor;
int lSensor;

int rLightSide;
int lLightSide;

int rStartingLightSide;
int lStartingLightSide;

bool foundBall = false;

uint16_t position;

bool lightOn = false;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


void setup()
{
  Serial.begin(9600);
 
  pinMode(Left_motor_go, OUTPUT); // PIN 8 (PWM)
  pinMode(Left_motor_back, OUTPUT); // PIN 9 (PWM)
  pinMode(Right_motor_go, OUTPUT); // PIN 6 (PWM)
  pinMode(Right_motor_back, OUTPUT); // PIN 7 (PWM)
  pinMode(Right_motor_en,OUTPUT);// PIN 5 (PWM)
  pinMode(Left_motor_en,OUTPUT);// PIN 11 (PWM)
  
//  myservo.write(90);

  pinMode(Echo2, INPUT);    // Set Echo port mode
  pinMode(Trig2, OUTPUT);   // Set Trig port mode


  pinMode(lightL, INPUT); // Set left Line Walking Infrared sensor as input
  pinMode(lightR, INPUT); // Set Right Line Walking Infrared sensor as input
  pinMode(lightC, INPUT); // Set left Line Walking Infrared sensor as input

  pinMode(lightSideR, INPUT);
  pinMode(lightSideL, INPUT);

  pinMode(rightSensor, INPUT);
  pinMode(leftSensor, INPUT);
  
  myservo.attach(10);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A15,A14,A13,A12,A11,A10,A9,A8}, SensorCount);
  qtr.setEmitterPin(20);

  //delay(500);
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH); 

 for (uint16_t i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(20);
  }
  digitalWrite(20, LOW); // turn off Arduino's LED to indicate we are through with calibration

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

  rStartingLightSide = digitalRead(lightSideR);
  lStartingLightSide = digitalRead(lightSideL);
  rStartingLightSide -= 50;
  
}


void run(int time)     // ahead
{
 analogWrite(Left_motor_en,160);
  analogWrite(Right_motor_en,160);  // Right motor enable
  digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  digitalWrite(Right_motor_back,LOW);   
  //analogWrite(Right_motor_go,200);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  //analogWrite(Right_motor_back,0); 
  digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  //analogWrite(Left_motor_go,137);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  //analogWrite(Left_motor_back,0);
  delay(time * 50); 
}

void brake(int time)         //STOP
{
  digitalWrite(Right_motor_go,LOW);//Stop the right motor
  digitalWrite(Right_motor_back,LOW);
  digitalWrite(Left_motor_go,LOW);//Stop the left motor
  digitalWrite(Left_motor_back,LOW);
  delay(time * 200);  //Running time can be adjusted  
}

void left(int time)        //turn left
{
  analogWrite(Left_motor_en,100);
  analogWrite(Right_motor_en,120);  // Right motor enable
  digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  digitalWrite(Right_motor_back,LOW);   
  //analogWrite(Right_motor_go,200);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  //analogWrite(Right_motor_back,0); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  //analogWrite(Left_motor_go,137);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  //analogWrite(Left_motor_back,0);
  delay(time * 10); 
}
void spin_left(int time)   //Left rotation
{
  analogWrite(Left_motor_en,100);
  analogWrite(Right_motor_en,100);  // Right motor enable
  //digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  //digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,255);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back,0); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,0);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,255);
  delay(time * 100);
}

void line_left(int time)   //Left rotation
{
  analogWrite(Left_motor_en,120);
  analogWrite(Right_motor_en,200);  // Right motor enable
  //digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  //digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,255);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back,0); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,0);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,255);
  delay(time * 65);
}

void line_right(int time)   //Left rotation
{
  analogWrite(Left_motor_en,120);
  analogWrite(Right_motor_en,120);  // Right motor enable
  //digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  //digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,0);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back,255); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,255);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,0);
  delay(time * 65); 
}

void right(int time)      //turn right
{
 analogWrite(Left_motor_en,150);
 analogWrite(Right_motor_en,150);  // Right motor enable
  //digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  //digitalWrite(Right_motor_back,LOW);   
  //analogWrite(Right_motor_go,200);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  //analogWrite(Right_motor_back,0); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,137);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,0);
  delay(time * 10); 
}

void spin_right(int time)   //Right rotation
{
analogWrite(Left_motor_en,150);
  analogWrite(Right_motor_en,150);  // Right motor enable
  //digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  //digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,0);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back,255); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,255);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,0);
  delay(time * 100); 
}

void back(int time)   //back off
{
  analogWrite(Left_motor_en,100);
  analogWrite(Right_motor_en,100);  // Right motor enable
  //digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  //digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,0);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back,255); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,0);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,255);
  delay(time * 100); 
}

//==========================================================

void spin(int time){
  digitalWrite(Right_motor_go,LOW); //right motor back off
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Right_motor_go,0);
  analogWrite(Right_motor_back,150);// PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(Right_motor_en,165);
  digitalWrite(Left_motor_go,LOW);  //left motor back off
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,200);
  analogWrite(Left_motor_back,0);// PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(Left_motor_en,165);
  delay(time * 100);
}



/*void Distance_test()   // Measuring front distance
{
  digitalWrite(Trig, LOW);    // set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // set trig port low level
  float Fdistance = pulseIn(Echo, HIGH);  // Read echo port high level time(unit:μs)
  Fdistance= Fdistance/58;       // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******
                                 //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                 // ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  Serial.print("Distance:");      //Output Distance(cm)
  Serial.println(Fdistance);         //display distance
  Distance = Fdistance;
}  

void Distance_test1()   // Measuring front distance
{
  digitalWrite(Trig1, LOW);    // set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(Trig1, HIGH);  // set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(Trig1, LOW);    // set trig port low level
  float Fdistance = pulseIn(Echo1, HIGH);  // Read echo port high level time(unit:μs)
  Fdistance= Fdistance/58;       // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******
                                 //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                 // ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  Serial.print("Distance1:");      //Output Distance(cm)
  Serial.println(Fdistance);         //display distance
  Distance1 = Fdistance;
}*/

int Distance_test2()   // Measuring front distance
{
  digitalWrite(Trig2, LOW);    // set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(Trig2, HIGH);  // set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(Trig2, LOW);    // set trig port low level
  float Fdistance = pulseIn(Echo2, HIGH);  // Read echo port high level time(unit:μs)
  Fdistance= Fdistance/58;       // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******/
                                 //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                 // ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  Serial.print("Distance2:");      //Output Distance(cm)
  Serial.println(Fdistance);         //display distance
  Distance2 = Fdistance;
}  
//==========================================================

void keysacn()
{
  int val;  
  val=digitalRead(key);// Reads the button ,the level value assigns to val
  while(digitalRead(key))// When the button is not pressed
  {
    val=digitalRead(key);
  }
  while(!digitalRead(key))// When the button is pressed
  {
    delay(10); //delay 10ms
    val=digitalRead(key);// Reads the button ,the level value assigns to val
    if(val==LOW)  //Double check the button is pressed
    {
       
      //digitalWrite(beep,HIGH);//The buzzer sounds
      delay(50);//delay 50ms
      while(!digitalRead(key)) //Determine if the button is released or not
        ;//digitalWrite(beep,LOW);//mute
    }
    else
      ;//digitalWrite(beep,LOW);//mute
  }
}
/*main loop*/
//================================
bool lefty = false;
bool righty = false;
int count = 0;

void left_corner(){
  
    if(sensorValues[7] >(sensorValues[4] + 500) && sensorValues[6] > (sensorValues[4] + 100) && righty == false){
      if(count>7){
        line_left(4);
        lefty = true;
        righty = false;
        count = 0;
        }
     else{
        line_left(2);
      }
    }
    else{
      run(1);
    }
}

void right_corner(){
  if(sensorValues[0] >(sensorValues[3] + 500) && sensorValues[1] > (sensorValues[3] + 100) && lefty == false){
      if(count>7){
        line_right(5);
        righty = true;
        lefty = false;
        count = 0;
      }
      else{
        line_right(2);
      }
    }
    else{
      run(1);
      }
}

void searchBall(){
  while(!foundBall){
     lightDetection();
     Distance_test2();
     if(Distance2<3){
      foundBall = true;
      brake(1);
      closeClaw();
     }
     back(1);
     brake(30);
  } 
}

void checkLine(){
  rLightSide = digitalRead(lightSideR);
  lLightSide = digitalRead(lightSideL);
  rLightSide -= 50;
  
  Serial.println();
  Serial.print("left:");
  Serial.println(lLightSide);
  Serial.print("right:");
  Serial.println(rLightSide );

  if(rLightSide<(rStartingLightSide-100) && lLightSide<(lStartingLightSide-100)){
    line_left(4);
    searchBall();
  }
  else if(rLightSide<(rStartingLightSide-100)){
    line_right(5);
    searchBall();
  }
  else if(lLightSide<(lStartingLightSide-100)){
    line_left(4);
    searchBall();
  }
  else{
    run(1);
  }
}
  
void readLineSensors(){
    position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}

void followLine(){
  
  readLineSensors();
  
  rSensor = digitalRead(rightSensor); //black line gives 1
  lSensor = digitalRead(leftSensor);
  Serial.print("rSensor: ");
  Serial.println(rSensor);
 
  Serial.print("lSensor: ");
  Serial.println(lSensor);
  Serial.println("");

  
  
  count += 1;
  if(count>10){
    lefty = false;
    righty = false;
  }

  if(lSensor==1){
    count = 0;
    lefty = true;
    righty = false;
    line_left(4);
  }
  else if(rSensor == 1){
    count = 0;
    righty = true;
    lefty = false;
    line_right(4);
  }
  else{
    if(position < 6000 && position > 1000){
      

      if(sensorValues[2] > (sensorValues[3]+200) || sensorValues[1] > (sensorValues[3]+200) || sensorValues[0] > (sensorValues[3]+200)){
        line_right(1);
      }
      
      else if(sensorValues[5] > (sensorValues[4]+200) || sensorValues[6] > (sensorValues[4]+200) || sensorValues[7] > (sensorValues[4]+200)){
        line_left(1);
      }
      else{
        run(1);
      }
      
    
    }
    else if(position>6500 || position<100){
      if(sensorValues[7]<200 && sensorValues[0]<200){
        checkLine();
      }
    }
  }
}

  

//light detection functions
void detect(){
  lL = analogRead(lightL);
  rR = analogRead(lightR);
  cC = analogRead(lightC);
  rR -= 50;
  Serial.println();
  Serial.print("left:");
  Serial.println(lL);
  Serial.print("right:");
  Serial.println(rR);
  Serial.print("centre:");
  Serial.println(cC);
  //delay(500);
  /*while(lL <= 100 || rR <= 100){
    lL = analogRead(lightL);
    rR = analogRead(lightR);
    brake(1);
  }*/
}

void lightDetection(){
  detect();

  if(cC < 305 && cC < lL && cC < rR){
    while(cC < 305){
      lightOn = false;
      brake(1);
      detect();}}
   else if(rR < (lL - 50) && rR < 400){
    while(rR < (lL - 50)){
      Serial.println("go right");
      lightOn = true;
      spin_right(1);
      brake(1);
    detect();}
  }
  else if(lL < (rR - 50) && lL < 400){
    while(lL < (rR - 50)){
      Serial.println("go left");
      lightOn = true;
      spin_left(1);
      brake(1);
      detect();
    }}
    //detect();

  }


  //claw functions
  void openClaw(){
    myservo.write(360);
    delay(100);
    Serial.println("servo open");
  }
  void closeClaw(){
    myservo.write(45);
    delay(100);
    Serial.println("close claw");
  }


void loop()
{
  openClaw();
  keysacn();//Press the button to start
 
  while(1)
  {
    
  followLine();
  brake(1);
  //lightDetection();
  
  }
}
