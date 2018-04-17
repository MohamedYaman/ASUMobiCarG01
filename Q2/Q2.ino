void setup() 
{
  pinMode(4 , OUTPUT);
  pinMode(5 , OUTPUT);
  pinMode(6 , OUTPUT);
  pinMode(7 , OUTPUT);
}

void loop() 
{
  for (int i=4;i<8;i++)
  {
    digitalWrite (i,HIGH);
    delay(1000);
    digitalWrite (i,LOW);
    delay(1000);
  }
  for (int j=6;j>4;j--)
  {
    digitalWrite (j,HIGH);
    delay(1000);
    digitalWrite (j,LOW);
    delay(1000);
  }
                        
}
