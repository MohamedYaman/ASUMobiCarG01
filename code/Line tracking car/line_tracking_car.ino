#include "SoftwareSerial.h"// import the serial library\

int IN1 = 2; // IN1 left
int IN2 = 3; // IN2 left
int IN3 = 4 ; // IN3 right 
int IN4 = 5 ; // IN4 right

//// LINE FOLLOWER ////


int const Right = 8;
int const Center = 7 ;
int const Left =  6;

//int const Rsensor = digitalRead(Right);
//int const Lsensor = digitalRead(Left);
//int const Csensor = digitalRead(Center);




int  SpeedL=64;
int  SpeedR=58;
int Enable = 9 ; 


void setup() {
// Start serial for Bluetooth module
Serial.begin (9600);
// define the H bridge's IN pins as an OUTPUT
pinMode( IN1,OUTPUT); 
pinMode( IN2 ,OUTPUT); 
pinMode( IN3,OUTPUT); 
pinMode( IN4 ,OUTPUT);

pinMode (Enable , OUTPUT ) ;


// line follower 
pinMode(Right,INPUT) ;
pinMode(Center,INPUT) ;
pinMode(Left,INPUT);
}

// left half is hooked up together to rotate either clockwise or anticlockwise  and the same goes for the right half

void FORWARD(int SpeedR){

    analogWrite (Enable,SpeedR) ; 

  digitalWrite(IN1,1);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,1);  
  digitalWrite(IN4,0); 
  
}
void BACKWARD(int SpeedL){
  analogWrite (Enable,SpeedL) ; 

 digitalWrite(IN1,0);  
  digitalWrite(IN2,1); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,1); 
 }
void LEFT(int SpeedR){
    analogWrite (Enable,SpeedR) ; 

  digitalWrite(IN1,0);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,1);  
  digitalWrite(IN4,0); 
}
void RIGHT(int SpeedR){
  analogWrite (Enable,SpeedR) ; 
  digitalWrite(IN1,1);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,0); 
}

void Stop(){

    digitalWrite(IN1,0);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,0); 
}




void loop() {

//analogWrite (Enable,Speed) ; 

int Rsensor = digitalRead(Right);
int Lsensor = digitalRead(Left);
int Csensor = digitalRead(Center);

if (  ((Rsensor == LOW) && (Csensor == HIGH) && (Lsensor == LOW)) )
{ 
  FORWARD (SpeedR); 
 } 

 

else if ( ((Rsensor == HIGH) && (Csensor == LOW) && (Lsensor == LOW)) )
{
  analogWrite (Enable,SpeedL) ; 

 digitalWrite(IN1,1);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,0);  
 }


else if ( ((Rsensor == HIGH) && (Csensor == HIGH) && (Lsensor == LOW)))
{
  
  analogWrite (Enable,SpeedL) ; 

 digitalWrite(IN1,1);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,0);  
 }

 
else if ( ((Rsensor == LOW) && (Csensor == LOW) && (Lsensor == HIGH)) )
 {
   analogWrite (Enable,SpeedL) ; 

 digitalWrite(IN1,0);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,1);  
  digitalWrite(IN4,0);  
}



 else if ( ((Rsensor == LOW) && (Csensor == HIGH) && (Lsensor == HIGH)) ) 

{   analogWrite (Enable,SpeedL) ; 

 digitalWrite(IN1,0);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,1);  
  digitalWrite(IN4,0);  
}
 
else if (  ((Rsensor == LOW) && (Csensor == LOW) && (Lsensor == LOW)) )
{
//BACKWARD(Speed);
}





}
