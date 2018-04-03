#define button 3

void setup() {
  pinMode(2,OUTPUT);
  pinMode(button,INPUT);
  
}
int val[6];
int i=0;
void loop() {

 
  val[0]=analogRead(A0);
  val[1]=analogRead(A1);
  val[2]=analogRead(A2);
  val[3]=analogRead(A3);
  val[4]=analogRead(A4);
  val[5]=analogRead(A5);
 
    if(button==1)
        { delay(200);
          i=i+1;
        }
     
     if(i==7)
        i=0;
    
 
  digitalWrite(2,HIGH);
  delay(val[i]);
  digitalWrite(2,LOW);
  delay(1000);
  
 
  
}
