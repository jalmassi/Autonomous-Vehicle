//Justin Almassi
//Nicholas Bell
//Zixuan Chen 


//============================NICK==========================
//Servo things
//===============================================================
#include <Servo.h>
Servo myservo;
int pos = 0;
//============================KUMAN==========================
// Ultrasonic check obstacle and avoid Experiment
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
//==============================
int Left_motor_go=8;    
int Left_motor_back=9;    

int Right_motor_go=6;  
int Right_motor_back=7;  

int Right_motor_en=5;  
int Left_motor_en=11;  

/*Set Button port*/
int key=13;



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
// 
//  digitalWrite(key,HIGH);//Initialize button
  //digitalWrite(beep,LOW);// set buzzer mute

  
  myservo.attach(12);
}


void run(int time)     // ahead
{
 analogWrite(Left_motor_en,100);
  analogWrite(Right_motor_en,100);  // Right motor enable
  //digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  //digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,200);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back,0); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,137);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,0);
  delay(time * 100); 
}

void brake(int time)         //STOP
{
  digitalWrite(Right_motor_go,LOW);//Stop the right motor
  digitalWrite(Right_motor_back,LOW);
  digitalWrite(Left_motor_go,LOW);//Stop the left motor
  digitalWrite(Left_motor_back,LOW);
  delay(time * 100);  //Running time can be adjusted  
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
  delay(time * 100); 
}
void spin_left(int time)   //Left rotation
{
  analogWrite(Left_motor_en,150);
  analogWrite(Right_motor_en,150);  // Right motor enable
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

void right(int time)      //turn right
{
 analogWrite(Left_motor_en,100);
  analogWrite(Right_motor_en,100);  // Right motor enable
  //digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  //digitalWrite(Right_motor_back,LOW);   
  //analogWrite(Right_motor_go,200);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  //analogWrite(Right_motor_back,0); 
  //digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  //digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,137);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,0);
  delay(time * 100); 
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
  delay(500);
  delay(10);
}
void servoLookRight() {
  myservo.write(135);
  delay(500);
  delay(10);
}
void servoLookLeft() {
  myservo.write(45);
  delay(500);
  delay(10);
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


void response1()
{  
    Distance_test1();
    lookDown();
    if(Distance >15 || Distance2 >15)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance>15 || Distance2 > 15)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_left(5);//Right rotation for 300ms
       brake(1);
        Distance_test1();
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
       spin_left(3);
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
    if(Distance1 < 30)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance1 < 30)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_left(3);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
      }
    }
    
       
  }

void response4()
{  
    Distance_test1();
    if(Distance1 < 30)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance1 < 30)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_right(10);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
      }
    }
    
       
  }
void response5()
{  
    Distance_test1();
    if(Distance1 < 30)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance1 < 30)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
        while(Distance1 < 10){
          brake(1);//stop
       back(1);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
        }
       brake(1);//stop
       spin_left(3);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
      }
    }
    
       
  }


void responseL()
{  
    Distance_test1();
    lookDown();
    if(Distance2 >15)//The value is the distance that meets the obstacle, and can be set according to the actual situation   
    { 
      Distance_test1();
      lookDown();
      while(Distance2 > 15)//Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again.
      {
       brake(1);//stop
       spin_right(1);//Right rotation for 300ms
       brake(1);
        Distance_test1();
        lookDown();
      }
    }
    else 
         run(1);//There is nothing obstacled. Go ahead.
       
  }
 
void loop()
{
  keysacn();//Press the button to start
 
  while(1)
  {

    response1();

    brake(1);
    servoLookRight();
    response5();
    
    
    servoLookForward();
    response3();
    
    servoLookLeft();
    response4();
    
   

  }
}
