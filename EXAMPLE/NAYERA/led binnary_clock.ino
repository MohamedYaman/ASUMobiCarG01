#include <Wire.h>
#include "RTClib.h"
#include <TimerOne.h>

RTC_DS1307 RTC;
int temp,inc,hours1,minut,add=11;

#define d1 12
#define d2 11
#define d3 10
#define d4 9
#define d5 8
#define d6 7

#define r1 6
#define r2 5
#define r3 4
#define r4 3

int HOUR,MINUT,SECOND;

volatile int count=0;

void Clear(int d)
{
    digitalWrite(d1, HIGH);
    digitalWrite(d2, HIGH);
    digitalWrite(d3, HIGH);
    digitalWrite(d4, HIGH);
    digitalWrite(d5, HIGH);
    digitalWrite(d6, HIGH);
}

void callback()
{
  digitalWrite(13, digitalRead(13) ^ 1);
  count++;
  if(count>=7)
  count=1;
  switch(count%7)
  {
    case 1:
    Clear(d1);
    temp=SECOND%10;
    show(temp);
    digitalWrite(d1, LOW);
    break;

    case 2:
    Clear(d2);
    temp=SECOND/10;
    show(temp);
    digitalWrite(d2, LOW);
    for(int i=0;i<10000;i++)
    {
    }
    break;
    
    case 3:
    Clear(d3);
    temp=MINUT%10;
    show(temp);
    digitalWrite(d3, LOW);
    for(int i=0;i<10000;i++)
    {
    }
    break;  

    case 4:
    Clear(d4);
    temp=MINUT/10;
    show(temp);
    digitalWrite(d4, LOW);
    for(int i=0;i<10000;i++)
    {
    }
    break;

    case 5:
    Clear(d5);
    temp=HOUR%10;
    show(temp);
    digitalWrite(d5, LOW);
    for(int i=0;i<10000;i++)
    {
    }
    break;  

    case 6:
    Clear(d6);
    temp=HOUR/10;
    show(temp);
    digitalWrite(d6, LOW);
    for(int i=0;i<10000;i++)
    {
    }
    break;    
  }
}

void show(int d)
{
    for(int i=0;i<1;i++)
    {
      digitalWrite(r4, !((temp>>0)&1));
      digitalWrite(r3, !((temp>>1)&1));
      digitalWrite(r2, !((temp>>2)&1));
      digitalWrite(r1, !((temp>>3)&1));
     // delay(1);
     for(int i=0;i<1000;i++);
    }
}

 
void setup()
{
 Wire.begin();
 Serial.begin(9600);
 RTC.begin();
 digitalWrite(next, HIGH);
 digitalWrite(set_mad, HIGH);
 digitalWrite(INC, HIGH);
 pinMode(14, OUTPUT);
 for(int i=2;i<=12;i++)
 {
  pinMode(i, OUTPUT);
  digitalWrite(i, HIGH);
 }
    
 if(!RTC.isrunning())
 {
 RTC.adjust(DateTime(__DATE__,__TIME__));
 }

 Timer1.initialize(1000);
 Timer1.attachInterrupt(callback); 
}
 
void loop()
{
 int temp=0,val=1,temp4;
 DateTime now = RTC.now();
 HOUR=now.hour();
 MINUT=now.minute();
 SECOND=now.second();

 Serial.print(HOUR);
 Serial.print(":");
 Serial.print(MINUT);
 Serial.print(":");
 Serial.println(SECOND);
 delay(200);
}
