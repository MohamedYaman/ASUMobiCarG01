  #include "SoftwareSerial.h"// import the serial library\

int LedPinUltraSonic = A0 ;
int LedBT = A1 ;

int IN1 = 2; // IN1 left
int IN2 = 3; // IN2 left
int IN3 = 4 ; // IN3 right 
int IN4 = 5 ; // IN4 right

int BluetoothData; // save data recieved from the bluetooth module
SoftwareSerial HC05 (0,1); // RX, TX
int  Speed=255;

const int trigPin = 13; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 12; // Echo Pin of Ultrasonic Sensor
long duration, distanceCm; // define time duration , distance in inches ,and cm
float minObstacleDistance = 26 ; // min distace of obstacle detection


/// OBSTACLE CHECK FUNCTION ///
int ObstacleCheck ( )
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

 duration = pulseIn(echoPin, HIGH);
 distanceCm = duration/29/2;

return distanceCm;
  
}





void setup() {
// Start serial for Bluetooth module
HC05.begin(9600);
Serial.begin (9600);
// define the H bridge's IN pins as an OUTPUT
pinMode( IN1,OUTPUT); 
pinMode( IN2 ,OUTPUT); 
pinMode( IN3,OUTPUT); 
pinMode( IN4 ,OUTPUT);


pinMode(LedBT, OUTPUT ) ; 

// Ultra sonic stuff
Serial.begin(9600); // Starting Serial Terminal
pinMode (trigPin , OUTPUT ) ;
pinMode (echoPin , INPUT ) ; 
pinMode(LedPinUltraSonic , OUTPUT ) ; 
}

// left half is hooked up together to rotate either clockwise or anticlockwise  and the same goes for the right half

void FORWARD(int Speed){

 
digitalWrite(IN1,1);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,1);  
  digitalWrite(IN4,0); 
  
}
void BACKWARD(int Speed){
   
 digitalWrite(IN1,0);  
  digitalWrite(IN2,1); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,1); 
 }
void LEFT(int Speed){
   
  digitalWrite(IN1,0);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,1);  
  digitalWrite(IN4,0); 
}
void RIGHT(int Speed){
 
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


void FORWARDRIGHT(int Speed){
 
  digitalWrite(IN1,1);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,0); 
}


void FORWARDLEFT(int Speed){
 
  digitalWrite(IN1,0);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,1);  
  digitalWrite(IN4,0); 
}


void BACKWARDRIGHT(int Speed){
   
  digitalWrite(IN1,0);  
  digitalWrite(IN2,1); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,0); 
}


void BACKWARDLEFT(int Speed){
  
 digitalWrite(IN1,0);  
  digitalWrite(IN2,0); 
  digitalWrite(IN3,0);  
  digitalWrite(IN4,1); 
}



void loop() {
  


 if (HC05.available()){
 BluetoothData=HC05.read();
 Serial.println(BluetoothData);

 int check ;

while ( 1 ) {
check = ObstacleCheck () ;
if ( check < minObstacleDistance )
 {check = 1 ;
 analogWrite ( LedPinUltraSonic , 255 ) ;
 RIGHT(Speed) ; }

else 
//break ; 

//}

analogWrite ( LedPinUltraSonic , 0 ) ;

   
/// FORWARD ///

  if(BluetoothData=='F'){   // if Forward direction pressed .... 
   analogWrite ( LedBT , 255 ) ;
   FORWARD(Speed);
   }
   
   

/// BACKWWARD ///
  if(BluetoothData=='B'){   // if Back direction pressed .... 
  analogWrite ( LedBT , 255 ) ;
  BACKWARD(Speed);}


/// LEFT ///   
  if(BluetoothData=='L'){   // if Left direction pressed .... 
 analogWrite ( LedBT , 255 ) ;
  LEFT(Speed);}


 /// RIGHT ///
  if(BluetoothData=='R'){   // if Right direction pressed ....
  analogWrite ( LedBT , 255 ) ;
    RIGHT(Speed);}

 /// forward right ///
  if(BluetoothData=='I'){   // if F L direction pressed ....
   analogWrite ( LedBT , 255 ) ;
   FORWARDRIGHT(Speed);}

  
 /// forward left ///
  if(BluetoothData=='G'){   // if F R direction pressed ....
  analogWrite ( LedBT , 255 ) ;
   FORWARDLEFT(Speed);}

  



 /// back right ///
  if(BluetoothData=='H'){   // if B L direction pressed ....
   analogWrite ( LedBT , 255 ) ;
    BACKWARDLEFT(Speed);}


 /// back right ///
  if(BluetoothData=='J'){   // if B R direction pressed ....
  analogWrite ( LedBT , 255 ) ;
    BACKWARDRIGHT(Speed);}
    
  if(BluetoothData=='S'){   // if nothing pressed ....
  analogWrite ( LedBT , 0 ) ;
  Stop();}


}
}
  
}

}




