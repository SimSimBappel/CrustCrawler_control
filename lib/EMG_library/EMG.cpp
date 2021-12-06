#include "Arduino.h"
#include "EMG.h"
#include "hardwareserial.h"


/*
Format for XBEE packets
    0 0x7e  - start ch
    1 0x00  - length msb
    2 0x14  - length lsb
    3 0x83  - API frame identifier
    4 0x56  - senders address
    5 0x78  - senders address
    6 0x43  - received signal strength - RSI
    7 0x00  - option byte
    8 0x01  - nu,mber of samples
    9 0x3e  - channel indicator mask n/a A5 A4 A3 A2 A1 A0 D8
   10 0xe0  - channel indicatot mask  D7 D6 D5 D4 D3 D2 D1 D0
   11 0x00  - digital sample (msb)
   12 0x40  - digital sample (lsb)

   13 0x??  - Acc Z (msb)
   14 0x??  - Acc Z (lsb)

   15 0x??  - Acc Y (msb)
   16 0x??  - Acc Y (lsb)

   17 0x??  - Acc X (msb)
   18 0x??  - Acc X (lsb)

   19 0x??  - EMG ch1 (msb)
   20 0x??  - EMG ch1 (lsb)

   21 0x??  - EMG ch2 (msb)
   22 0x??  - EMG ch2 (lsb)

   23 0x??  - checksum
   */

EMG::EMG(){
  //intentionally do nothing
}

//Get input from EMG module
void EMG::GetInput(){ 
  //read input and put data into array 
  for(int i = 0; i < 24; i++){
    
    //while(!Serial.available()); //Wait for data to be received
    _str[i] = Serial.read(); //Save incoming byte into string array that is not string but integer.

    //Make sure it reads from start
    if(i == 2){
      if(_str[0] != 0X7E || _str[1] != 0X00 || _str[2] != 0X14){ //make sure that data is proper arranged
      i = -1;
      }
    }
  }
}

//reset values in the input array
void EMG::reset(){
  for(int i = 0; i < 24; i++){
    _str[i] = 0;
  }
}

//Prints out the complete package
void EMG::printInput(){
  for(int i = 0; i < 24; i++){
    Serial.print(_str[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
}

//access the accellerometer x value
int EMG::AccX(){
    return _str[18] + (_str[17] << 8);
}

//access the accellerometer y value
int EMG::AccY(){
    return _str[16] + (_str[15] << 8);
}

//access the accellerometer z value
int EMG::AccZ(){
    return _str[14] + (_str[13] << 8);
}

//access the EMG 1 value
int EMG::EMG1(){
  return _str[20] + _str[19] * 256;
}

//access the EMG 2 value
int EMG::EMG2(){
  return _str[22] + _str[21] * 256;
}
