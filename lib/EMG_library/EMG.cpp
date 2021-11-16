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

void EMG::GetInput(int time){
  for(int i = 0; i < 24; i++){
    while(!Serial.available());
    _str[i] = Serial.read();
    if(_str[i] == 0X7E){
      i = 0;
    }
    //delay(1); Experimental delay.
    if(i == 23 && _str[0] != 0X7E){
      i = -1;
    }
  }
  delay(time);
}

void EMG::reset(){
  for(int i = 0; i < 24; i++){
    _str[i] = 0;
  }
}

void EMG::returnInput(){
  for(int i = 0; i < 24; i++){
    Serial.print(_str[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
}

int EMG::AccX(){
    return _str[18] + (_str[17] << 8);
}

int EMG::AccY(){
    return _str[16] + (_str[15] << 8);
}

int EMG::AccZ(){
    return _str[14] + (_str[13] << 8);
}


int EMG::EMG1(){
  return _str[20] + _str[19] * 256;
  //return _str[19];
}


int EMG::EMG2(){
  return _str[22] + _str[21] * 256;
}
