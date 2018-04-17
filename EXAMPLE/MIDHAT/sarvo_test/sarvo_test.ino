#include <servo.h>
Servo vert,hori;  // create servo object to control a servo
                // a maximum of eight servo objects can be created 
int pos = 0;    // variable to store the servo position
void setup()
{
  hori.attach(9);
  vert.attach(10);  // attaches the servo on pin 9,10 to the servo objects
  vert.write(0);
  hori.write(0);
}

void loop()
{
  for(pos = 0; pos < 180; pos += 10)  // goes from 0 degrees to 180 degrees   
  {                                  // in steps of 10 degrees     
    vert.write(pos);     
    hori.write(pos);    // tell servo to go to position in variable 'pos'     
    delay(100);                       // waits 100ms for the servo to reach the position   
  }   
  for(pos = 180; pos>=1; pos-=10)     // goes back from 180 degrees to 0 degrees
  {                               
    vert.write(pos);              // tell servo to go to position in variable 'pos'
    hori.write(pos);
    delay(100);                       // waits 100ms for the servo to reach the position
  }
}
