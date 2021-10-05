#include <Arduino.h>
#include <krnl.h>
#include <DynamixelShield.h>
#include <PID_v1.h>

const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

struct k_t *pt1, *pt2, *pt3, *pt4;          
 
//find proper stacksize, see void setup comments
char s1[500]; 
char s2[500]; 
char s3[3000];
char s4[500];
 


void t1(void)
{
  //setup
  int DXL_ID = 2;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalPosition(DXL_ID, 1073);
  //loop
  while(1){
    //make a Serial handler
    k_sleep(100);
  }
}           

void t2(void)
{
  //int homePos = 1073;
  //int deviation = 60;
  int DXL_ID = 1;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalPosition(DXL_ID,2755);
  while(1){
    /*Serial1.print("M2 Current : ");
    Serial1.print(dxl.getPresentCurrent(DXL_ID)); 
    Serial1.print("  Present POS : ");
    Serial1.println(int(dxl.getPresentPosition(DXL_ID))); 
    
    if(int(dxl.getPresentPosition(DXL_ID)) < homePos-deviation){
      
      dxl.setGoalCurrent(DXL_ID, 200);
    }
    else if(int(dxl.getPresentPosition(DXL_ID)) > homePos+deviation){
     
      dxl.setGoalCurrent(DXL_ID, -200);
    }
    else{
      dxl.setGoalCurrent(DXL_ID, 0);
    }*/

    /*Serial1.print("t2 unused:");
    Serial1.println(int(k_unused_stak));*/
    k_sleep(100);
  }
  
}



void t3(void){
  
  byte DXL_ID = 2; //test to see if byte works 
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT); //current mode is not supported by the end-effectors motors IDs 4 and 5
  dxl.torqueOn(DXL_ID);

  byte DXL_ID2 = 3; //test to see if byte works 
  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_CURRENT); //current mode is not supported by the end-effectors motors IDs 4 and 5
  dxl.torqueOn(DXL_ID2);

  const byte numChars = 10;
  char receivedChars[numChars];
  //String inString = "";
  boolean newData = false;
  //int msg = 0;
  //int msg2 = 0;

  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while(1){
    //Serial1.println("im running    ");



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
      Serial1.print(receivedChars);
      /*inString = String(receivedChars);
      if(receivedChars[1] == '1'){
        Serial1.print("char 1 = 1");

        msg = Serial1.readStringUntil(',');
        
      }*/

      //msg = atoi(receivedChars);
      newData = false;
    }
/*
    while(Serial1.available() > 0){
      
      inString = Serial1.readStringUntil(':');
      Serial1.print("a: ");
      Serial1.print(inString);
      if(inString == "M1"){
        Serial1.println("string is M1");
        inString = Serial1.readStringUntil('\n');
        msg = inString.toInt();
      }
      else if(inString == "M2"){
        Serial1.print("string is M2");
        inString = Serial1.readStringUntil('\n');
        msg2 = inString.toInt();
      }
    }*/


/*
  //use analog stick
    msg = analogRead(A8);
    msg = map(msg,0,1023,-200,200);

    msg2 = analogRead(A9);
    msg2 = map(msg2,0,1023,70,-70);
    Serial1.print("msg: ");
    Serial1.print(msg);
    Serial1.print("msg2: ");
    Serial1.print(msg2);

    dxl.setGoalCurrent(DXL_ID, msg);    
    dxl.setGoalCurrent(DXL_ID2, msg2); 
*/


      
      

/*
    if(msg != 0){
      dxl.torqueOn(2);
      dxl.setGoalCurrent(2, msg);  
    }
    else{
      dxl.torqueOff(2);
    }

    if(msg2 != 0){
      dxl.torqueOn(3);
      dxl.setGoalCurrent(3, msg2);  
    }
    else{
      dxl.torqueOff(3);
    }
    */

    k_sleep(100);
  }
}

void t4(void){
  //PID
  //Define Variables we'll be connecting to
  /*
  double Setpoint, Input, Output;

  //Specify the links and initial tuning parameters
  double Kp=0.05, Ki=5, Kd=0.15;
  PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-600, 600);
  char msg = 'a';
  Setpoint = 2736;

  //int homePos = 2736;
  //int deviation = 60;
  //float current = 0;
  //float k_p = 0.1;
  byte DXL_ID = 1;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);


  //pid
  //Input = dxl.getPresentPosition(DXL_ID);
  */
  
  while(1){
    /*
    while(Serial1.available()>0){
      msg=Serial1.read();
    }

    if(msg == '1'){
      Setpoint = 2000;
      //Serial1.print("heard: ");
      
    }
    else if(msg == '2'){
      Setpoint = 3000;
      //Serial1.print("yeeeeet");
    }

    Input = dxl.getPresentPosition(DXL_ID);
    myPID.Compute();

    //Serial1.print("M1 pos: ");
    //Serial1.print(float(dxl.getPresentPosition(DXL_ID)));
    //Serial1.print("  Current: ");
    Serial1.println(Output);
    dxl.setGoalCurrent(DXL_ID, Output);
    else if(Input > Setpoint){
      Serial1.print("M1 pos: ");
      Serial1.print(float(dxl.getPresentPosition(DXL_ID)));
      Serial1.print("  Current: ");
      Serial1.println(-Output);
      dxl.setGoalCurrent(DXL_ID, -Output);
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
  k_init(4,0,0); 

  //each pt(n) is a pointer to a function t(n), priority, stack size 
  pt1=k_crt_task(t1, 10, 500); 
  pt2=k_crt_task(t2, 11, 500);
  pt3=k_crt_task(t3, 12, 3000);
  pt4=k_crt_task(t4, 13, 500);

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




