//Justin Almassi 
//Servo things
//===============================================================
#include <Servo.h>
#include <QTRSensors.h>

QTRSensors qtr;
Servo myservo;
int pos = 0;

//===============================================================
int Echo = A1;
int Trig =A0;

int Echo1 = A3;
int Trig1 = A2;

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

unsigned int lL;
unsigned int rR;
unsigned int cC;

unsigned int SL2;
unsigned int SR2;



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
  
  pinMode(Echo, INPUT);    // Set Echo port mode
  pinMode(Trig, OUTPUT);   // Set Trig port mode

  pinMode(Echo1, INPUT);    // Set Echo port mode
  pinMode(Trig1, OUTPUT);   // Set Trig port mode

  pinMode(Echo2, INPUT);    // Set Echo port mode
  pinMode(Trig2, OUTPUT);   // Set Trig port mode


  pinMode(lightL, INPUT); // Set left Line Walking Infrared sensor as input
  pinMode(lightR, INPUT); // Set Right Line Walking Infrared sensor as input
  pinMode(lightC, INPUT); // Set left Line Walking Infrared sensor as input
  
//  myservo.attach(10);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10, A9, A8}, SensorCount);
  qtr.setEmitterPin(21);

  delay(500);
  pinMode(21, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

 for (uint16_t i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(20);
  }
  digitalWrite(21, LOW); // turn off Arduino's LED to indicate we are through with calibration

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
  delay(1000);
  
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
  delay(time * 50);  //Running time can be adjusted  
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
  analogWrite(Right_motor_en,120);  // Right motor enable
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



void Distance_test()   // Measuring front distance
{
  digitalWrite(Trig, LOW);    // set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // set trig port low level
  float Fdistance = pulseIn(Echo, HIGH);  // Read echo port high level time(unit:μs)
  Fdistance= Fdistance/58;       // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******/
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
  Fdistance= Fdistance/58;       // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******/
                                 //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                 // ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  Serial.print("Distance1:");      //Output Distance(cm)
  Serial.println(Fdistance);         //display distance
  Distance1 = Fdistance;
}  

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
//Servo Look
//================================

void servoLookForward() {
  myservo.write(90);
  delay(400);
}
void servoLookRight() {
  myservo.write(60);
  delay(400);
}
void servoLookLeft() {
  myservo.write(120);
  delay(400);
}
void lookDown() {
  Distance_test();//Measuring front distance
  Distance_test2();
  }
void response() {
  if(Distance > 30|| Distance1 > 30 || Distance2 < 30)//The value is the distance that meets the obstacle, and can be set according to the actual situation  
    {
      Distance_test();//Measuring front distance
      Distance_test1();
      Distance_test2();
      while(Distance > 30 || Distance1 > 30 || Distance2 < 30)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       back(1);  
       spin_right(1);//Right rotation for 300ms
       brake(1);//stop
       
       Distance_test();//Measuring front distance
       Distance_test2();
       
         
      }
    }
    else
         run(1);//There is nothing obstacled. Go ahead.
}

void responseBoth()
{  
    
    lookDown();
    if(Distance >15 || Distance2 >15)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      bool right = true;
      lookDown();
      while(Distance > 15 && Distance2 > 15)
      {
       brake(1);//stop
       spin_left(6);//Right rotation for 300ms
       brake(1);
       right = false;
       lookDown();
      }
      lookDown();
      while(Distance > 15)
      {
       brake(1);//stop
       spin_left(6);//Right rotation for 300ms
       brake(1);
       right = false;
       lookDown();
      }
      lookDown();
      while(Distance2 > 15 && right == true)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_right(4);//Right rotation for 300ms
       brake(1);
        
        lookDown();
      }
     
    }
    else 
         run(1);//There is nothing obstacled. Go ahead.
       
  }

void response2()
{  
    Distance_test1();
    lookDown();
    if(Distance1 < 15)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance1 < 15)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_left(1);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
      }
    }
    else 
         run(1);//There is nothing obstacled. Go ahead.
       
  }

 void response3()
{  
    Distance_test1();
    if(Distance1 < 15)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance1 < 15)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_left(2);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
      }
    }
    
       
  }

void response4()
{  
    Distance_test1();
    if(Distance1 < 15)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance1 < 15)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_right(2);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
      }
    }
    
       
  }
void response5()
{  
    Distance_test1();
    if(Distance1 < 15)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance1 < 15)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_left(2);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
      }
    }
    
       
  }
  
  
//  SR = analogRead(SensorRight);//Right Line Walking Infrared sensor against white undersurface,then LED[L2] light illuminates and while against black undersurface,LED[L2] goes off
//  SL = analogRead(SensorLeft);//Left Line Walking Infrared sensor against white undersurface,then LED[L3] light illuminates and while against black undersurface,LED[L3] goes off
//  SR2 = analogRead(SensorRight2);//Right Line Walking Infrared sensor against white undersurface,then LED[L2] light illuminates and while against black undersurface,LED[L2] goes off
//  SL2 = analogRead(SensorLeft2);//Left Line Walking Infrared sensor against white undersurface,then LED[L3] light illuminates and while against black undersurface,LED[L3] goes off
//  Serial.println();
//  Serial.print("SR: ");
//  Serial.println(SR);
//  Serial.print("SR2: ");
//  Serial.println(SR2);
//  Serial.print("SL: ");
//  Serial.println(SL);
//  Serial.print("SL2: ");
//  Serial.println(SL2);
  //Serial.println(SR);
  //Serial.println(SL);

  
int line = false;
void followLine(){

  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
  if(sensorValues[3]<100 && sensorValues[4]<100 && sensorValues[5]<100 && (position>6500 || position<500)){
    brake(1);
    Serial.println('no line');
  }
  else if(position<2000){
    line_right(4);
  }
  else if(position<3000){
    line_right(1);
  }
  else if(position>6000){
    line_left(3);
  }
  else if(position>5000){
    line_left(1);
  }
//  if ((sensorValues[3] > 700 || sensorValues[4] > 700) && sensorValues[5] < 400 && sensorValues[2] < 400 && sensorValues[5] > 700 && sensorValues[6] < 400 && sensorValues[7] < 400)// Black lines were not detected at the same time
//    ;   // go ahead
//  else if (sensorValues[0] > 700 || sensorValues[1] > 700 || sensorValues[2] > 700){// Left sensor against white undersurface and right against black undersurface , the car left off track and need to adjust to the right.
//    line_right(1);
//    Serial.println("TURN RIGHT");}
//  else if (sensorValues[0] > 700 || sensorValues[1] > 700 || sensorValues[2] > 700){ // Rihgt sensor against white undersurface and left against black undersurface , the car right off track and need to adjust to the left.
//    line_left(1);
//    Serial.println("TURN LEFT");}
//  else if (SR > 500 & SL > 500 && SL2 > 300 && SR2 > 300){ //black lines at same time, go left
//    ;}
    
  }
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
  //Serial.println(rR);
  //delay(500);
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
  


void loop()
{
  keysacn();//Press the button to start
 
  while(1)
  {
  run(1);
  brake(1);
  delay(1000);
  
//  responseBoth();
//  brake(1);
//  if(!lightOn){
//    servoLookRight();
//    response5();
//      
//    servoLookForward();
//    response3();
//      
//    servoLookLeft();
//    response4();}
//  delay(1500);
//  delay(500);
  Distance_test1();
  Distance_test();
  Distance_test2();
  followLine();
  brake(1);
  lightDetection();
  }
}
