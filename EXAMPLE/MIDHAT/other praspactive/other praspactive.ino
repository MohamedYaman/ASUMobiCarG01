int pin_sel[4] = {8,9,10,11} ; // trigger 12, echo 13 are for ultra sonic dont use em bitches
char fun[5] = {'F','B','R','L','S'} ; 
int default_move = 100 ;
int upper_hand = 100 ; 
float dis = 100 ;   
int ind = 5 ;   
void communist(int value[4]) {
   for (int a = 0 ; a < 4 ; a++) {
    digitalWrite(pin_sel[a] , value[a]) ; 
   }
}
int right(){
  int r[4] = {1,0,0,0} ; //low power make it {1000}
  communist(r) ; 
  ind = 0 ; 
  return ind ;
}
int forward(){
  int f[4] = {0,1,0,1};
  communist(f) ;
  ind = 1 ;
  return ind ; 
}
int  left(){
  int l[4] = {0,1,0,0}; //low power make it {0100}
  communist(l) ;
  ind = 2 ; 
  return ind ; 
}
int back(){
  int b[4] = {1,0,1,0} ;
  communist(b) ; 
  ind = 3 ;
  return ind ; 
}
void sm(){
  int sm[4] = {0,0,0,0} ; 
  communist(sm) ; 
}
int invert(){
  switch(ind){
    case 0 : left()  ; break ;    
    case 1 : back()  ; break ;
    case 2 : right() ; break ;
    case 3 : forward() ; break ;
    case 5 : break ;
  }
  ind = 5 ; 
  return ind ; 
}
void stop_it(){
  sm() ; 
  invert() ;  
  sm() ; 
}
/*void follow_light(){
  while(true) {
  float v = analogRead(0) ; 
  if ( v < 300 ) {
    forward() ; 
  }
  else {
    back() ; 
  }
}
}*/
int dont_crash(){
 digitalWrite(12, LOW);
 delayMicroseconds(2) ; 
 digitalWrite(12, HIGH);
 delayMicroseconds(10) ; 
 digitalWrite(12, LOW);
float time_sec = pulseIn(13, HIGH);  
dis = (time_sec * 330 / 20000) ;
return dis ; 
}
void setup(){
  Serial.begin(9600) ; 
  pinMode(8,OUTPUT) ; 
  pinMode(9,OUTPUT) ;
  pinMode(10,OUTPUT) ; 
  pinMode(11,OUTPUT) ;
  pinMode(12,OUTPUT) ; 
  pinMode(13,INPUT) ;   
} 
void motor_control_bluetooth(){
  if (Serial.available() > 0) {
    while(true) { 
          for (int a = -1 ; a < 9 ; a++) {
            if (Serial.read() == fun[a]) {
               default_move = a ;  
            }
            switch(default_move) {
              case 0 : dont_crash() ;  
              if (dis < 5){
                right() ; delay(500) ;   break ; 
              }else {
              forward() ; 
              break ; 
              } 
              case 1 : back() ; break ; 
              case 2 : right() ; break ; 
              case 3 : left() ; break ; 
              case 4 : stop_it() ; break ;           
              }
          }
    }
    }
  }
//void master_mind(){}
void loop(){ 
  motor_control_bluetooth() ;
}   
