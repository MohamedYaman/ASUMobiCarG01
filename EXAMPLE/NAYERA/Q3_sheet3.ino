int inputpin= 1;
float Y= 0.00;
float X = 0.00;

void setup(){
   pinMode(inputpin, INPUT); //assigning the input port
   Serial.begin(9600); //BaudRate
}
void loop(){
   
   X = analogRead(inputpin);//reads the analog input
   while (X>=0 || X<=5 )//condition 
   {
       Y= (20*X)-50; // formula for calculating voltage out i.e. V+, here 5.00
       Serial.print ("The actual voltage is ");
       Serial.print(Y);
       Serial.print("The given voltage is ");
       Serial.print(X);

   }

Serial.print("the voltage is less-than 0 or greater-than 5v");
}
