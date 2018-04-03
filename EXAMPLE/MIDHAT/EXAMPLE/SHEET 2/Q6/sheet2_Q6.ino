#define LED0 3
#define LED1 4
#define LED2 5
#define analog0 A0
#define analog1 A1
#define analog2 A2


void setup() {
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED0,OUTPUT);
  
}
int AN0,AN1,AN2;
void loop() {
  AN0=analogRead(analog0);
  AN1=analogRead(analog1);
  AN2=analogRead(analog2);

  if(AN0>AN1)
  {
    if(AN0>AN2)
    {
      digitalWrite(LED0,HIGH);
    }
    else digitalWrite(LED2,HIGH);
  }
  
  else if(AN1>AN2)
  {
      digitalWrite(LED1,HIGH);   
  }
  else digitalWrite(LED2,HIGH);






}
