#include <Arduino.h>
#include <krnl.h>
#include <DynamixelShield.h>
#include <PID_v1.h>

const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

struct k_t *pserialHandler, *pcurrent, *pgripper, *pt4;          


struct k_msg_t *msgQ, *msgQ2;

char dataBufForMsgQ[100]; 

char dataBufForMsgQ2[100];

struct k_t  *gripSem, *curSem;





//find proper stacksize, see void setup comments
char s1[500]; 
char s2[500]; 
char s3[500];
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
  bool motorsOn = true;

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
      Serial1.println(receivedChars);
      newData = false;
    }


    if(receivedChars[0] == 'G'){
      //make sure the message queue is emptied idk how
      res = k_send(msgQ, &receivedChars);
      k_signal(gripSem);
      receivedChars[0] = ' ';
      k_sleep(50);
    }


    if(receivedChars[0] == 'C'){
      //Serial1.println("a");
      //make sure the message queue is emptied idk how
      res = k_send(msgQ2, &receivedChars);
      k_signal(curSem);
      //Serial1.println("b");
      receivedChars[0] = ' ';
      k_sleep(50);
    }
    

    if(receivedChars[0] == 'S' && motorsOn){
      for(int i=0; i<6; i++){ 
        receivedChars[i] = 'A';
        dxl.torqueOff(i);
      }
      Serial1.println("Dynamixel STOP");
      motorsOn = false;
      receivedChars[0] = ' ';
    }
    else if(receivedChars[0] == 'S' && !motorsOn){
      Serial1.println("Dynamixel Start");
      delay(2000);
      for(int i=0; i<6; i++){ 
        receivedChars[i] = 'A';
        dxl.torqueOn(i);
      }
      motorsOn = true;
      receivedChars[0] = ' ';
    }




    k_sleep(100);
  }
}           

void current(void)
{

  uint8_t DXL_ID = 1;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT); //set to OP_CURRENT 
  dxl.torqueOn(DXL_ID);
  //dxl.setGoalPosition(DXL_ID, 2755); //temp
  
  DXL_ID = 2;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT); //set to OP_CURRENT 
  dxl.torqueOn(DXL_ID);
  //dxl.setGoalPosition(DXL_ID,1073);//temp

  DXL_ID = 3; //test to see if byte works 
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT); //set to OP_CURRENT 
  dxl.torqueOn(DXL_ID);
  //dxl.setGoalPosition(DXL_ID,2034);//temp

  DXL_ID = 4;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM); 
  dxl.torqueOn(DXL_ID);
  

  
  DXL_ID = 5;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM); 
  dxl.torqueOn(DXL_ID);

  char res;
  int lostMessages;
  char msg[20];
  char tempMsg[4];
  int tempCurrent;
  

  while(1){
    k_wait(curSem, 0);
    //Serial1.println("c");
    res = k_receive(msgQ2, &msg, 10, &lostMessages);  
    //Serial1.println("d");
    for(int i = 1; i < 4; i++){
      tempMsg[i-1] = msg[i];
    }
    tempCurrent = atoi(tempMsg);
    Serial1.print("tempMsg: ");
    Serial1.println(tempMsg);
    dxl.setGoalCurrent(1, tempCurrent);
    
    for(int i = 4; i < 7; i++){
      tempMsg[i-4] = msg[i];
    }
    tempCurrent = atoi(tempMsg);
    Serial1.print("tempMsg: ");
    Serial1.println(tempMsg);
    dxl.setGoalCurrent(2, tempCurrent);

    for(int i = 7; i<10; i++){
      tempMsg[i-7] = msg[i];
    }
    tempCurrent = atoi(tempMsg);
    Serial1.print("tempMsg: ");
    Serial1.println(tempMsg);
    dxl.setGoalCurrent(3, tempCurrent);
    
    
    
    
    k_sleep(100);
  }
  
}



void gripper(void){
  char res;
  char msg[20];
  int lostMessages;
  float POS = 250;

  while(1){
    k_wait(gripSem, 0);
    
    dxl.torqueOff(4);
    dxl.setOperatingMode(4, OP_PWM); 
    dxl.torqueOn(4);

    dxl.torqueOff(5);
    dxl.setOperatingMode(5, OP_PWM); 
    dxl.torqueOn(5);

    res = k_receive(msgQ, &msg, 10, &lostMessages);
   
    
      if(msg[1] == 'O'){//OPEN
        Serial1.println("gripper open");

        //dxl.setGoalPosition(4, 2685);
        dxl.setGoalPWM(4, POS, UNIT_RAW);
        
        //dxl.setGoalPosition(5, 1600);
        dxl.setGoalPWM(5, -POS, UNIT_RAW);
      }
      else if(msg[1] == 'C'){//close
        Serial1.println("gripper close");
        
        //dxl.setGoalPosition(4, 2149);
        dxl.setGoalPWM(4, -POS, UNIT_RAW);
      
        //dxl.setGoalPosition(5, 2084);
        dxl.setGoalPWM(5, POS, UNIT_RAW);
      }
      else{
        Serial1.println("msg2[1] is not == to c || o");
      }

    k_sleep(100);
  }
}



void t4(void){
  int m4Pos;
  int m5Pos;
  int m4Load;
  int m5Load;
  while(1){
    //Serial spammer
      Serial1.print("M4 load: ");
      m4Load = dxl.readControlTableItem(PRESENT_LOAD, 4);
      Serial1.print(m4Load);
      Serial1.print(" Temp: ");
      Serial1.print(dxl.readControlTableItem(PRESENT_TEMPERATURE, 4));

      Serial1.print(" M5 load: ");
      m5Load = dxl.readControlTableItem(PRESENT_LOAD, 5);
      Serial1.print(m5Load);
      Serial1.print(" Temp: ");
      Serial1.println(dxl.readControlTableItem(PRESENT_TEMPERATURE, 5));

      if(m4Load < -250 && m5Load > 250){
        Serial1.println("i did the thing");
        m4Pos = dxl.getPresentPosition(4);
        m5Pos = dxl.getPresentPosition(5);
        dxl.torqueOff(4);
        dxl.setOperatingMode(4,OP_POSITION);
        dxl.torqueOn(4);
        dxl.torqueOff(5);
        dxl.setOperatingMode(5,OP_POSITION);
        dxl.torqueOn(5);
        dxl.setGoalPosition(4, m4Pos);
        dxl.setGoalPosition(5, m5Pos);
      }
    k_sleep(100);
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
  k_init(4,2,2); 

  msgQ = k_crt_send_Q (1, sizeof(char[20]),  dataBufForMsgQ);
  msgQ2 = k_crt_send_Q (1, sizeof(char[20]),  dataBufForMsgQ2); 

  //each pt(n) is a pointer to a function t(n), priority, stack size 
  pserialHandler=k_crt_task(serialHandler, 15, 500); 
  pcurrent=k_crt_task(current, 13, 500);
  pgripper=k_crt_task(gripper, 1, 3000);
  pt4=k_crt_task(t4, 12, 500);


  
  gripSem = k_crt_sem(0, 1);
  curSem = k_crt_sem(0,1);

  //stack size----
  //Arudino Mega has 8 kByte RAM
  //stack is used in funcion calls for:
  // return address, registers stakked, local variables in a function
  //
  //!!! to find the unused stacksize call "k_unused_stack" !!!
  
  // start kernel with tick speed 1 milli seconds
  int res;
  res = k_start(1); // 1 milli sec tick speed
  // you will never return from k_start
  Serial1.print("ups an error occured: "); Serial.println(res);
}

void loop(){ 
  //Serial1.print("Im not supposed to be here.");
}




