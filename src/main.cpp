#include <Arduino.h>

#include <DynamixelShield.h>

void home();

 uint8_t DXL_ID = 1;
//int DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  Serial1.begin(9600);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);
  delay(5000);
  home();
}

//gay

void loop() {
  // put your main code here, to run repeatedly:
  
  // Position Control Mode in protocol2.0, Joint Mode in protocol1.0
  // Turn off torque when configuring items in EEPROM area
/*
  Serial1.println("HOME");
  int homePos[] = {2726,2020,933,2172,2112};
  for(int i=2;i<6;i++){
    Serial1.print("moving motor: ");
    Serial1.println(i);
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_POSITION);
    dxl.torqueOn(i);
    if(dxl.setGoalPosition(i, homePos[i-1])){
      delay(5000);
      Serial1.print("Present Position : ");
      Serial1.println(dxl.getPresentPosition(i)); Serial1.println();
    }
  }
  DXL_ID=1;
  dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.torqueOn(DXL_ID);
    if(dxl.setGoalPosition(DXL_ID, 2726)){
      delay(1000);
      Serial1.print("Present Position : ");
      Serial1.println(dxl.getPresentPosition(DXL_ID)); Serial1.println();
    }*/
}

void home(){


  DXL_ID = 3;

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);

  while(1){
    Serial1.println("running");
    if(dxl.setGoalCurrent(DXL_ID, 512)){
    delay(300);
    Serial1.print("Present Current : ");
    Serial1.println(dxl.getPresentCurrent(DXL_ID)); Serial1.println();
    }
  }
}