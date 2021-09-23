//Dont upload to mega



#include <Arduino.h>
String msg ="";
  void setup() {
    // Begin the Serial at 9600 Baud
    Serial.begin(9600);
  }
  
  void loop() {
    
   if(Serial.available() > 0){
    msg = "";
    while(Serial.available() > 0){
      msg += char(Serial.read());
      delay(20);//maybe decrease
    }
    Serial.print(msg);
  }  
    
  }