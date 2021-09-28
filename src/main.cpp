#include <Arduino.h>
#include <krnl.h>
#include <DynamixelShield.h>



const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

// NB only one task must use print if you dont protect the serial port by a critical section???

struct k_t *pt1, *pt2, *pt3;          
 

//find proper stacksize, see void setup comments
char s1[3000]; 
char s2[3000]; 
char s3[3000];
 
void t1(void)
{
  int deviation = 60;
  int DXL_ID = 3;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);
  
  while(1){
    
    //make a Serial handler
    Serial1.print("M3 Current : ");
    Serial1.print(dxl.getPresentCurrent(DXL_ID)); 
    Serial1.print("  Present POS : ");
    Serial1.println(dxl.getPresentPosition(DXL_ID)); 

    if(int(dxl.getPresentPosition(DXL_ID)) < 2030-deviation){
      dxl.setGoalCurrent(DXL_ID, 70);
    }
    else if(int(dxl.getPresentPosition(DXL_ID)) > 2030+deviation){
      dxl.setGoalCurrent(DXL_ID, -70);
    }
    else{
      dxl.setGoalCurrent(DXL_ID, 0);

    }
    k_sleep(50);
  }
  //hej

          
}           

void t2(void)
{
  int deviation = 60;
  int DXL_ID = 2;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);
  while(1){
    Serial1.print("M2 Current : ");
    Serial1.print(dxl.getPresentCurrent(DXL_ID)); 
    Serial1.print("  Present POS : ");
    Serial1.println(int(dxl.getPresentPosition(DXL_ID))); 

    if(int(dxl.getPresentPosition(DXL_ID)) < 2020-deviation){
      
      dxl.setGoalCurrent(DXL_ID, 400);
    }
    else if(int(dxl.getPresentPosition(DXL_ID)) > 2020+deviation){
     
      dxl.setGoalCurrent(DXL_ID, -400);
    }
    else{
      dxl.setGoalCurrent(DXL_ID, 0);
    }
    k_sleep(50);
  }
  
}

void t3(void){
  byte DXL_ID = 4; //test to see if byte works 
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION); //current mode is not supported by the end-effectors motors IDs 4 and 5
  dxl.torqueOn(DXL_ID);

  const byte numChars = 50;
  char receivedChars[numChars];
  boolean newData = false;

  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while(1){
    Serial1.print("t3");
 
    while (Serial1.available() > 0 && newData == false) {
      rc = Serial1.read();

      if (recvInProgress == true) {
        if (rc != endMarker) {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
            ndx = numChars - 1;
          }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            recvInProgress = false;
            ndx = 0;
            newData = true;
        }
      }

      else if (rc == startMarker) {
        recvInProgress = true;
      }
    }


    if (newData) {
      Serial1.print("This just in ... ");
      Serial1.println(receivedChars);
      newData = false;
    }
    /*
    if(Serial1.available() > 0){
      while(Serial1.available() > 0){
        msg += char(Serial1.read());
        k_sleep(10);
      }
      Serial1.print("heard: ");
      Serial1.print(msg);
      
      if(msg == "4")
    }*/
    k_sleep(50);
  }
}



void setup(){
  //Serial definition--
  //Native serial port (switches from USB-port to dynamixel with physical switch)
  dxl.begin(57600);
  //second serial (the one that communicates with UNO)
  Serial1.begin(9600);
  //--

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  //if (manifacturing) info is needed from a specific motor use below
  //dxl.ping(DXL_ID);
  

  // init krnl so you can create x tasks, y semaphores and z message queues
  k_init(3,0,0); 

  //each pt(n) is a pointer to a function t(n), priority, stack size 
  pt1=k_crt_task(t1, 10, 1000); 
  pt2=k_crt_task(t2, 11, 1000);
  pt3=k_crt_task(t2, 12, 3000);

  //stack size----
  //Arudino Mega has 8 kByte RAM
  //stack is used in funcion calls for:
  // return address, registers stakked, local variables in a function
  //
  //!!! to find the unused stacksize call "k_unused_stack" !!!
  
  // start kernel with tick speed 1 milli seconds
  k_start(1); 
}

void loop(){ 
  //Serial1.print("Im not supposed to be here.");
 }
