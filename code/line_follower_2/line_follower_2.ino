const int in1=13;
const int in2=12;
const int in3=11;
const int in4=10;
const int in5=2;
const int in6=3;
const int in7=5;
 int val2;
int val3;
int val4;
void forward ()
{ digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);}
  void backward(){
    digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
    
  }
  void right(){ digitalWrite(in1,HIGH);
   digitalWrite(in2,LOW);
   digitalWrite(in3,LOW);
   digitalWrite(in4,HIGH);
    
  }
  void left(){ digitalWrite(in1,LOW);
   digitalWrite(in2,HIGH);
   digitalWrite(in3,HIGH);
   digitalWrite(in4,LOW);
  }


void setup() {
  pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);
pinMode(in5,INPUT);
pinMode(in6,INPUT);
pinMode(in7,INPUT);
}

void loop() {
  val2=digitalRead(in5);

val3=digitalRead(in6);
val4=digitalRead(in7);
if(val2==LOW&&val3==HIGH&&val4==HIGH){left();}
else if(val2==LOW&&val3==HIGH&&val4==LOW){forward();}
else if(val2==HIGH&&val3==HIGH&&val4==LOW){right();}
else if(val2==HIGH&&val3==LOW&&val4==HIGH){backward();}
else if(val2==HIGH&&val3==HIGH&&val4==HIGH){left();}
}
