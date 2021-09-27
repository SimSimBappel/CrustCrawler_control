#include <Arduino.h>
#include <krnl.h>
#include <DynamixelShield.h>



const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;



// A small krnl program with two independent tasks
// They run at same priority so krnl will do timeslicing between them
// Watch LED and Serial TX 

// NB only one task must use print if you dont protect the serial port by a critical section

struct k_t *pt1, *pt2, *pt3;          // to taskdescriptor for t1 and t2  
 
char s1[3000]; // stak for task t1
char s2[3000]; // stak for task t2
char s3[3000];
 
void t1(void)
{
  int deviation = 60;
  int DXL_ID = 3;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);
  
  while(1){
    //Serial1.println(DXL_ID);
    
    Serial1.print("M3 Current : ");
    Serial1.print(dxl.getPresentCurrent(DXL_ID)); //Serial1.println();
    Serial1.print("  Present POS : ");
    Serial1.println(dxl.getPresentPosition(DXL_ID)); //Serial1.println();

    if(int(dxl.getPresentPosition(DXL_ID)) < 2030-deviation){
      //Serial1.print("højre");
      dxl.setGoalCurrent(DXL_ID, 70);
    }
    else if(int(dxl.getPresentPosition(DXL_ID)) > 2030+deviation){
      //Serial1.print("venstre");
      dxl.setGoalCurrent(DXL_ID, -70);
    }
    else{
      dxl.setGoalCurrent(DXL_ID, 0);

    }
    k_sleep(20);
  }
  // a task must have an endless loop
  // if you end and leave the task function - a crash will occur!!
  // so this loop is the code body for task 1

            // lenght of ticks in millisec is specified in
}                 // k_start call called from setup

void t2(void)
{
  int deviation = 60;
  int DXL_ID = 2;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);
  while(1){
    //Serial1.println(DXL_ID);
    
    Serial1.print("M2 Current : ");
    Serial1.print(dxl.getPresentCurrent(DXL_ID)); //Serial1.println();
    Serial1.print("  Present POS : ");
    Serial1.println(int(dxl.getPresentPosition(DXL_ID))); //Serial1.println();

    if(int(dxl.getPresentPosition(DXL_ID)) < 2020-deviation){
      //Serial1.print("højre");
      dxl.setGoalCurrent(DXL_ID, 400);
    }
    else if(int(dxl.getPresentPosition(DXL_ID)) > 2020+deviation){
      //Serial1.print("venstre");
      dxl.setGoalCurrent(DXL_ID, -400);
    }
    else{
      dxl.setGoalCurrent(DXL_ID, 0);
    }
    k_sleep(20);
  }
  
}

void t3(void){
  
}



void setup()
{

    // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  Serial1.begin(9600);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  //dxl.ping(DXL_ID);
  

  // init krnl so you can create 2 tasks, no semaphores and no message queues
  k_init(3,0,0); 

// two task are created
//               |------------ function used for body code for task
//               |  |--------- priority (lower number= higher prio
//               |  |   |--- staksize for array s1

  pt1=k_crt_task(t1,10,3000); 
  pt2=k_crt_task(t2,11,3000);
  pt3=k_crt_task(t2,12, 1000);
  
  
  // NB-1 remember an Arduino has only 2-8 kByte RAM
  // NB-2 remember that stak is used in function calls for
  //  - return address
  //  - registers stakked
  //  - local variabels in a function
  //  So having 200 Bytes of stak excludes a local variable like ...
  //    int arr[400];  
  // krnl call k_unused_stak returns size of unused stak
  // Both task has same priority so krnl will shift between the
  // tasks every 10 milli second (speed set in k_start)

  
    
  k_start(1); // start kernel with tick speed 1 milli seconds
}

void loop(){ /* loop will never be called */ }
