
int const Right = A3 ;
int const Center = A2 ;
int const Left = A1 ;

int const FrontL = 5 ;
int const BackL = 6 ;

int const FrontR = 10 ;
int const BackR = 11 ;

int const Rsensor =  analogRead(Right);
int const Lsensor =  analogRead(Left);
int const Csensor =  analogRead(Center);

void setup() {

pinMode (5, OUTPUT) ;
pinMode (6, OUTPUT) ;
pinMode (10, OUTPUT) ;
pinMode (11, OUTPUT) ;

}

void loop() {

  if ((Rsensor == HIGH) && (Csensor == LOW) && (Lsensor == HIGH))
{   
   analogWrite(FrontL, HIGH);
   digitalWrite(BackL, LOW);

   analogWrite(FrontR, HIGH);
   digitalWrite(BackR, LOW);

 
  
}

else if ( ((Rsensor == LOW) && (Csensor == LOW) && (Lsensor == HIGH)) || ((Rsensor == LOW) && (Csensor == HIGH) && (Lsensor == HIGH)) )
{
  //left
   analogWrite(FrontL, HIGH);
   digitalWrite(BackL, LOW);

   digitalWrite(FrontR, LOW);
   analogWrite(BackR, HIGH);

 
}

else if ( ((Rsensor == HIGH) && (Csensor == LOW) && (Lsensor == LOW)) || ((Rsensor == HIGH) && (Csensor == HIGH) && (Lsensor == LOW)) )
 {
  //right
   digitalWrite(FrontL, LOW);
   analogWrite(BackL, HIGH);

   analogWrite(FrontR, HIGH);
   digitalWrite(BackR, LOW);

  }

delay(100);
}
