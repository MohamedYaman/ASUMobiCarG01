const int swf=13;
const int swb=12;
const int swr=9;
const int swl=8;
const int m1=1;
const int m11=2;
const int m2=4;
const int m22=7;

 int swif;
 int swib;
 int swir;
 int swil;

void setup() {
  // put your setup code here, to run once:
pinMode(swf,INPUT);
pinMode(swb,INPUT);
pinMode(swr,INPUT);
pinMode(swl,INPUT);
pinMode(m1,OUTPUT);
pinMode(m11,OUTPUT);
pinMode(m2,OUTPUT);
pinMode(m22,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
swif =digitalRead(swf);
swib =digitalRead(swb);
swir =digitalRead(swr);
swil =digitalRead(swl);


if ( swif==1)
{
  digitalWrite(m1,1);
  digitalWrite(m11,0);
digitalWrite(m2,1);
  digitalWrite(m22,0);
}
 else if  (swib==1)
{
digitalWrite(m1,0);
  digitalWrite(m11,1);
  digitalWrite(m2,0);
  digitalWrite(m22,1);
}
 else if (swir==1)
{
 digitalWrite(m1,1);
  digitalWrite(m11,0);
  digitalWrite(m2,0);
  digitalWrite(m22,1);
}
 else if  (swil==1)

 { digitalWrite(m1,0);
  digitalWrite(m11,1);
  digitalWrite(m2,1);
 digitalWrite(m22,0);
 }
 else
{ digitalWrite(m1,LOW);
  digitalWrite(m11,LOW);
 digitalWrite(m2,LOW);
digitalWrite(m22,LOW);
}
}
