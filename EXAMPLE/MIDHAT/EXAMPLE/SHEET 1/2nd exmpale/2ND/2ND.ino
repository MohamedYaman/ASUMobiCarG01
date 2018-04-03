 
void setup() 
{
  for (int j =4 ;j<=7;j++)
  {  
pinMode (j , OUTPUT );


  }
}

void loop() {
 for (int j =4 ;j<=7;j++)
{
  digitalWrite (j,HIGH);
  delay (1000);
  digitalWrite (j,LOW);
  delay (1000);
  
}

  digitalWrite (6,HIGH);
  delay (1000);
  digitalWrite (6,LOW);
  delay (1000);
  digitalWrite (5,HIGH);
  delay (1000);
  digitalWrite (5,LOW);
  delay (1000);

}
