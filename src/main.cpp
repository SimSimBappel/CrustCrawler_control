#include <Arduino.h>
#include <krnl.h>
#include <DynamixelShield.h>
#include <math.h>

//test
int startMillis = millis();

//dxl setup
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
char s1[1000]; 
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
    int t4Millis = millis(); //delay tester
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
            receivedChars[ndx] = '\0'; 
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
      //Serial1.print("This just in ... ");
      //Serial1.print(receivedChars);
      newData = false;
    }


    if(receivedChars[0] == 'G'){
      //make sure the message queue is emptied idk how
      res = k_send(msgQ, &receivedChars);
      k_signal(gripSem);
      receivedChars[0] = ' ';
      k_sleep(10);
    }


    if(receivedChars[0] == 'P'){ //current input given in miliampere
      //make sure the message queue is emptied idk how
      res = k_send(msgQ2, &receivedChars);
      k_signal(curSem);
      receivedChars[0] = ' ';
      k_sleep(10);
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

    //delay tester
    //Serial1.print("t4 delay: ");
    //Serial1.println(t4Millis-startMillis);   
    startMillis = t4Millis;  

    k_sleep(10);
  }
}           

void current(void)
{

  uint8_t DXL_ID = 1;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION); 
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 20);
  dxl.torqueOn(DXL_ID);
  
  DXL_ID = 2;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 20);
  dxl.torqueOn(DXL_ID);
  

  DXL_ID = 3; 
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT); // Skal sættes til current senere hvis control system skal laves
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 50);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 10);
  dxl.torqueOn(DXL_ID);
  

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
  char tempMsg[7];
  int tempPos;
  int lastKomma;

  float m3 = 0.306;
  float theta1;
  float theta2;
  float theta3;
  float goalcurrent;
  int J3Pose;
  float Torque_g;
  float Torque_cs;
  float CurrentKick = 3.6;
  bool in_movement = false;
  //gripper fixed code
  int m4Pos;
  int m5Pos;

  unsigned long prevMillis = 0;
  bool serialDebug = true;

  while(1){

    k_wait(curSem, 0);
    prevMillis = millis();

    res = k_receive(msgQ2, &msg, 10, &lostMessages);  

    //P,roll,pitch

    Serial1.println(msg);
    
    //M1
    
    for(int i = 0; i < 7; i++){
      tempMsg[i]= ' ';
    }
    

    for(int i = 1; i<7; i++){
      if(msg[i] == ','){
        lastKomma = i+1;
        i=100;
      }
      else{
        tempMsg[i-1]= msg[i];
      }
    }
    tempPos = 2700;//-atoi(tempMsg);
    Serial1.print("m1: ");  
    Serial1.print(tempPos);
    dxl.setGoalPosition(1,tempPos);
    theta1 = (-tempPos+2700)*0.088;

    //M2
    
    for(int i = 0; i < 7; i++){
      tempMsg[i]= ' ';
    }
    

    for(int i = lastKomma; i<13; i++){
      if(msg[i] == ','){
        lastKomma = i+1;
        i=100;
      }
      else{
        tempMsg[i-lastKomma]= msg[i];
      }
    }
    tempPos = 1170;//-atoi(tempMsg);
    Serial1.print("m2: ");  
    Serial1.print(tempPos);
    dxl.setGoalPosition(2,tempPos);
    theta2 = (-tempPos+1170)*0.088;

    //M3
    
    for(int i = 0; i < 7; i++){
      tempMsg[i]= ' ';
    }
    

    for(int i = lastKomma; i<14; i++){
      if(msg[i] == ','){
        i=100; // end reading of message
      }
      else{
        tempMsg[i-lastKomma]= msg[i];
      }
    }

    tempPos = atoi(tempMsg); //tempPos could be renamed to tempCurrent
    J3Pose = dxl.getCurPosition(3);

   
    if (tempPos == 1) 
      tempPos = J3Pose + 75;

    if (tempPos == 0) 
      tempPos = J3Pose - 75;

    //Serial1.print("m3: ");
    theta3 = (-J3Pose+2020)*0.088; 
    float error = (tempPos-J3Pose);

    if (abs(error) < 5){
      in_movement = false;}

    else {
      CurrentKick = 0;
      in_movement = true;
    }

    Torque_cs = (tempPos-J3Pose)*1; // Kp
     
    Torque_g = m3*((1.2956*cos(theta1*PI/180)*sin(theta3*PI/180)) - ((1.2956*cos(theta2*PI/180))*cos(theta3*PI/180))*sin(theta1*PI/180));
    
    if (Torque_g+Torque_cs < 0){
    goalcurrent = 0.875*Torque_g - 0.25375 - CurrentKick;}
    else {
    goalcurrent = 0.875*Torque_g + 0.25375 + CurrentKick;}

    dxl.setGoalCurrent(3,goalcurrent);
    Serial1.print("Goal current");
    Serial1.println(goalcurrent);


    /*
   GP 
   PP
   Current



    */

    //gripper fixed code 2
    if(dxl.readControlTableItem(PRESENT_LOAD, 4) < -300 && dxl.readControlTableItem(PRESENT_LOAD, 5) > 300){
      Serial1.println("Gripper Fixed");
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

void gripper(void){
  char res;
  char msg[20];
  int lostMessages;
  float POS = 350;

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
  
  
  while(1){
      
}

    k_sleep(10);
  }  
  
void setup(){
  
  //Serial definition--
  //Native serial port (switches from USB-port to dynamixel with physical switch)
  dxl.begin(57600);
  //second serial (the one that communicates with UNO)
  Serial1.begin(57600);
  //--


  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  //if (manifacturing) info is needed from a specific motor use below
  //dxl.ping(DXL_ID);
  

  // init krnl so you can create x tasks, y semaphores and z message queues
  k_init(4,2,2); 

  msgQ = k_crt_send_Q (1, sizeof(char[20]),  dataBufForMsgQ);
  msgQ2 = k_crt_send_Q (1, sizeof(char[20]),  dataBufForMsgQ2); 

  //each pt(n) is a pointer to a function t(n), priority, stack size 
  pserialHandler=k_crt_task(serialHandler, 1, 500); 
  pcurrent=k_crt_task(current, 2, 500);
  pgripper=k_crt_task(gripper, 3, 3000);
  pt4=k_crt_task(t4, 5, 500);
  
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
/*
bool serialDebug = false; //not in while(1)
  //Serial spammer
   
    for(int i = 1; i <= 5 && serialDebug; i++){
      Serial1.print(" M");
      Serial1.print(i);
      Serial1.print(" load: ");      
      Serial1.print(dxl.readControlTableItem(PRESENT_LOAD, i));
      Serial1.print(" Temp: ");
      Serial1.print(dxl.readControlTableItem(PRESENT_TEMPERATURE, 5));
      Serial1.print(" Angle: ");
      Serial1.print(dxl.getPresentPosition(i));
      Serial1.println();
    }

    for(int i = 1; i <= 5 && serialDebug; i++){
      if(dxl.readControlTableItem(PRESENT_TEMPERATURE, i) == 0){
        Serial1.print("ERROR motor: ");
        Serial1.print(i);
        Serial1.println("is not connected");
      }
    }
    */

  //Serial1.print("Im not supposed to be here.");
}




