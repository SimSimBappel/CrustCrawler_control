#include <Arduino.h>
#include <krnl.h>
#include <DynamixelShield.h>
#include <PID_v1.h>

const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

struct k_t *pserialHandler, *pt2, *pgripper, *pt4;          
 



struct k_msg_t *msgQ;

char dataBufForMsgQ[100]; 

struct k_t *mutSem, *gripSem;





//find proper stacksize, see void setup comments
char s1[500]; 
char s2[500]; 
char s3[3000];
char s4[500];
 


void serialHandler(void)
{ 
  //Serial 
  const byte numChars = 20;
  char receivedChars[numChars];
  boolean newData = false;
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  char res;

  //loop
  while(1){
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
      newData = false;
      res = k_send(msgQ, &receivedChars);
      if(receivedChars[0] == 'G'){
        k_signal(gripSem);
        
      }
    }

/*
    k_wait(mutSem, 0);
    if (0 <= res) {
      Serial1.print("1: did deliver "); Serial1.println(receivedChars);
    }
    else {
      Serial1.print("1: no deliver:>>>>>>>>>>>>>>>>>< "); Serial1.println(receivedChars);
    }
    k_signal(mutSem);
*/
    


    k_sleep(100);
  }
}           

void t2(void)
{
  char res;
  char msg2[20];
  int lostMessages;

  byte DXL_ID = 2;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalPosition(DXL_ID, 1073);
  
  DXL_ID = 1;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalPosition(DXL_ID,2755);


  

  while(1){
    /*
    k_wait(mutSem, 0);

    k_signal(mutSem);

    res = k_receive(msgQ, &msg2, 0, &lostMessages);
    
    k_wait(mutSem, 0);
    //Serial.print("2: received "); Serial.print(msg2);
    //Serial.print(" lost: "); Serial.println(lostMessages);
    k_signal(mutSem);*/
    
    k_sleep(100);
  }
  
}



void gripper(void){
  char res;
  char msg2[20];
  int lostMessages;

  byte DXL_ID = 4;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalPosition(DXL_ID, 2149);
  DXL_ID = 5;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalPosition(DXL_ID, 2084);


  while(1){
    k_wait(gripSem, 0);
    k_eat_ticks(2);
    
    Serial1.println("gripper active");
    res = k_receive(msgQ, &msg2, 0, &lostMessages);
   
    Serial1.print("MSG2: ");
    Serial1.println(msg2);
    
      if(msg2[1] == 'O'){
        //OPEN
        DXL_ID = 4;
        dxl.setGoalPosition(DXL_ID, 2685);
        
        DXL_ID = 5;
        dxl.setGoalPosition(DXL_ID, 1637);
      }
      else if(msg2[1] == 'C'){
        //close
        DXL_ID = 4;
        dxl.setGoalPosition(DXL_ID, 2149);
        
        DXL_ID = 5;
        dxl.setGoalPosition(DXL_ID, 2084);
      }
      else{
        Serial1.println("msg[1] is not == to c || o");
      }
    
    
    
    k_sleep(100);
  }
}

void t4(void){

  byte DXL_ID = 2; //test to see if byte works 
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT); //current mode is not supported by the end-effectors motors IDs 4 and 5
  dxl.torqueOn(DXL_ID);

  byte DXL_ID2 = 3; //test to see if byte works 
  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_CURRENT); //current mode is not supported by the end-effectors motors IDs 4 and 5
  dxl.torqueOn(DXL_ID2);
  while(1){

  
    



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
  k_init(4,2,1); 

  msgQ = k_crt_send_Q (1, sizeof(char[20]),  dataBufForMsgQ); 

  //each pt(n) is a pointer to a function t(n), priority, stack size 
  pserialHandler=k_crt_task(serialHandler, 3, 500); 
  pt2=k_crt_task(t2, 3, 500);
  pgripper=k_crt_task(gripper, 4, 3000);
  pt4=k_crt_task(t4, 4, 500);


  mutSem = k_crt_sem(1, 1);
  gripSem = k_crt_sem(0, 1);

  //stack size----
  //Arudino Mega has 8 kByte RAM
  //stack is used in funcion calls for:
  // return address, registers stakked, local variables in a function
  //
  //!!! to find the unused stacksize call "k_unused_stack" !!!
  
  // start kernel with tick speed 1 milli seconds
  k_start(1); 
  Serial1.println("you don goofed up boaaah");
}

void loop(){ 
  //Serial1.print("Im not supposed to be here.");
}




