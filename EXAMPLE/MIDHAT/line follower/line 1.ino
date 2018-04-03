
int const Right = A3 ;
int const Center = A2 ;
int const Left = A1 ;

int const FrontL = 5 ;
int const BackL = 6 ;

int const FrontR = 10 ;
int const BackR = 11 ;

int const Rsensor = digitalRead(Right);
int const Lsensor = digitalRead(Left);
int const Csensor = digitalRead(Center);

void setup() {

pinMode (5, OUTPUT) ;
pinMode (6, OUTPUT) ;
pinMode (10, OUTPUT) ;
pinMode (11, OUTPUT) ;


}

void loop() {

  if ((Rsensor == HIGH) && (Csensor == LOW) && (Lsensor == HIGH))
{   
  analogWrite(FrontL, 180);
  digitalWrite(BackL, LOW);

  analogWrite(FrontR, 180);
  digitalWrite(BackR, LOW);
}

else if ( ((Rsensor == LOW) && (Csensor == LOW) && (Lsensor == HIGH)) || ((Rsensor == LOW) && (Csensor == HIGH) && (Lsensor == HIGH)) )
{
  //left
  analogWrite(FrontL, 180);
  digitalWrite(BackL, LOW);

  digitalWrite(FrontR, LOW);
  analogWrite(BackR, 180);
}

else if ( ((Rsensor == HIGH) && (Csensor == LOW) && (Lsensor == LOW)) || ((Rsensor == HIGH) && (Csensor == HIGH) && (Lsensor == LOW)) )
 {
  //right
  digitalWrite(FrontL, LOW);
  analogWrite(BackL, 180);

  analogWrite(FrontR, 180);
  digitalWrite(BackR, LOW);

}

delay(100);
}
