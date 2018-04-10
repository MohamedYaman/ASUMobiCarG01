float speed;

void setup() {
  for(int i=2;i<12;i++)
  {
    pinMode(i,OUTPUT);
  }
pinMode(A0,INPUT);
}

void loop() {

speed=analogRead(A0);

for(int i=2;i<12;i++)
{
    digitalWrite(i,HIGH);
    delay( (200/speed)*1000 );
    digitalWrite(i,LOW);

    
  
}

  
}
