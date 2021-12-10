#include <Arduino.h>
#include <krnl.h>
#include <DynamixelShield.h>
#include <math.h>

// test
int startMillis = millis();

// dxl setup
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

// This namespace is required to use Control table item names
using namespace ControlTableItem;

struct k_t *pserialHandler, *pcurrent, *pgripper, *pt4;
struct k_msg_t *msgQ, *msgQ2, *msgQ3;
char dataBufForMsgQ[100];
char dataBufForMsgQ2[100];
char dataBufForMsgQ3[100];
struct k_t *gripSem, *posSem, *curSem;

// find proper stacksize, see void setup comments
char s1[1000];
char s2[1000];
char s3[1000];
char s4[1000];

bool shouldMoveUp(double omega, float torque){
  if(omega == 0.00 && torque > 0.05){
    return true;
  }
  else{
    return false;
  }
}

bool shouldMoveDown(double omega, float torque){
  if(omega == 0.00 && torque < -0.05){
    return true;
  }
  else{
    return false;
  }
}




void turnTorqueOn(bool on)
{
  if(on)
  {
    for(int i = 1; i <= 5; i++)
    {
      dxl.torqueOn(i);
    }
  }
  else
  {
    for(int i = 1; i <= 5; i++)
    {
      dxl.torqueOff(i);
    }
  }
}



void serialHandler(void)
{
  // Serial
  const byte numChars = 20;
  char receivedChars[numChars];
  boolean newData = false;
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  bool motorsOn = true;

  // loop
  while (1)
  {
    unsigned long t1Millis = millis(); // delay tester
    while (Serial1.available() > 0 && newData == false)
    {
      rc = Serial1.read();

      if (recvInProgress == true)
      {
        if (rc != endMarker)
        {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars)
          {
            ndx = numChars - 1;
          }
        }
        else
        {
          receivedChars[ndx] = '\0';
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }

      else if (rc == startMarker)
      {
        recvInProgress = true;
      }
    }

    if (newData)
    {
      // Serial1.print("This just in ... ");
      // Serial1.print(receivedChars);
      newData = false;


      if (receivedChars[0] == 'G')
    {
      // make sure the message queue is emptied idk how
      k_send(msgQ, &receivedChars);
      k_signal(gripSem);
      //receivedChars[0] = ' ';
      k_sleep(10);
    }

    if (receivedChars[0] == 'P')
    { // current input given in miliampere
      // make sure the message queue is emptied idk how
      Serial1.println("got a P");
      k_send(msgQ2, &receivedChars);
      k_signal(posSem);
      //receivedChars[0] = ' ';
      k_sleep(10);
    }

    if(receivedChars[0] == 'C'){
      Serial1.println("got a C");
      k_send(msgQ3, &receivedChars);
      //k_signal(curSem);
    }

    if (receivedChars[0] == 'S' && motorsOn)
    {
      turnTorqueOn(false);
      Serial1.println("Dynamixel STOP");
      motorsOn = false;
      //receivedChars[0] = ' ';
    }
    else if (receivedChars[0] == 'S' && !motorsOn)
    {
      turnTorqueOn(true);
      Serial1.println("Dynamixel Start");
      motorsOn = true;
      //receivedChars[0] = ' ';
    }
    }

    

    // delay tester
    //Serial1.print("t1msec: ");
    //Serial1.println(millis()-t1Millis);
    k_sleep(30);
  }
}

void current(void) //maybe current should be called computed torque kernel instead?
{

  uint8_t DXL_ID = 3;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT); 
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 10);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 5);
  dxl.torqueOn(DXL_ID);

  DXL_ID = 4;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);

  DXL_ID = 5;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);
/*
  char res;
  int lostMessages;
  char msg[20];
  char tempMsg[7];
  int tempPos;
  int lastKomma;

  unsigned long prevMillis = 0;
  bool serialDebug = true;
*/
  float theta1;
  float theta2;
  float theta3;

  bool ref = false;


  float Torque_g;

  float m3 = 0.306; // weight of 3rd link in kg

  unsigned long t = 0;
  //unsigned long ts = millis();
  int torque = 10;
  double omega;
  float torque_Nm = 0.07;
  float theta_ref = 0, theta_ref_90 = -50, theta_ref_10 = 45; // ref 90 og 10 er test
  float Jt = 0.0001627;
  float omegac = 7, zeta = 0.35, kp = omegac * omegac, kv = 2 * zeta * omegac;
  int goalPose;
  int lostMessages;
  char msg[20];
  float currentKick = 275;

  while (1)
  {
    unsigned long t2Millis = millis();

    
    if(k_receive(msgQ3, &msg, -1, &lostMessages) == 1)
    {
      Serial.println("ass");
      if (msg[1] == '1')
      {
        goalPose = goalPose + 5;
      }
      else if (msg[1] == '0')
      {
        goalPose = goalPose - 5;
      }
    }

    theta_ref = goalPose*0.088;

    Serial1.println(theta_ref);

    //tempPos = atoi(tempMsg); // tempPos could be renamed to tempCurrent
    
    //Serial1.println(theta_ref);
      
     


    // goalPose = 1500;

    theta1 = (-dxl.getCurPosition(1) + 2700) * 0.088; // converts posistions to degrees and puts 0 downwards
    theta2 = (dxl.getCurPosition(2) - 1038) * 0.088;
    theta3 = (dxl.getCurPosition(3) - 1535) * 0.088;

    omega = dxl.getPresentVelocity(3, UNIT_RPM) * 6; // gets speed and convert it to angles a second.
/*
    if (millis() - t > 5000)
    {
      t = millis();
      if (ref == false){
        theta_ref = theta_ref_10;
        ref = true;
       
      }

      else if (ref == true)
        {
          theta_ref = theta_ref_90;
          ref = false;
        }
    }
   */

    float gconst = 0.45; // 1.2956;
    Torque_g = m3 * ((gconst * cos(theta1 * PI / 180) * sin(theta3 * PI / 180)) - (gconst * cos(theta2 * PI / 180) * sin(theta1 * PI / 180))); // New idea, maybe we should use R30 instead of R03
                    //m3*(gconst*cos(theta1 * PI / 180)*cos(theta2)*sin(theta3 * PI / 180) - gconst*sin(theta1 * PI / 180)*sin(theta3 * PI / 180)*cos(theta3 * PI / 180));
    torque_Nm = Jt * (kp * (theta_ref - theta3) - kv * omega);
    torque = (torque_Nm + Torque_g)*875; //to milli amps the minus the beacuse we dumb and need to flip the orientation of the new motor

    if(shouldMoveUp(omega, torque_Nm)){ //make if so that if it is approaching the gravitational vector the dampening should be larger
      torque += currentKick;
    }
    if(shouldMoveDown(omega, torque_Nm)){ //make if so that if it is approaching the gravitational vector the dampening should be larger
      torque -= currentKick; 
    }
    
    
    dxl.setGoalCurrent(3, torque, UNIT_MILLI_AMPERE);
    //dxl.setGoalPWM(3, torque);
    
    //__asm__ volatile ("cli");
/*
    Serial1.print("A:");
    Serial1.print(theta3);
    Serial1.print(",A_ref:");
    Serial1.print(theta_ref);
    Serial1.print(",Torque_Nm:");
    Serial1.print(torque_Nm);
    Serial1.print(",Torque_g:");
    Serial1.print(Torque_g);
    Serial1.print("Torque:");
    Serial1.println(torque);
*/
    //__asm__ volatile ("sei");


    

    //Serial1.print("t2msec: ");
    //Serial1.println(millis()-t2Millis);
    k_sleep(40);
  }
}

void gripper(void)
{
  char msg[20];
  int lostMessages;
  float POS = 350;
  int m4Pos;
  int m5Pos;
  bool moving = false;

  while (1)
  {
    /*
    unsigned long t3Millis = millis();
    
    if(!moving)
    {
      k_wait(gripSem, 0);
    }
    

    dxl.torqueOff(4);
    dxl.setOperatingMode(4, OP_PWM);
    dxl.torqueOn(4);

    dxl.torqueOff(5);
    dxl.setOperatingMode(5, OP_PWM);
    dxl.torqueOn(5);

    k_receive(msgQ, &msg, 10, &lostMessages);

    if (msg[1] == 'O')
    {
      moving = true;
      dxl.setGoalPWM(4, POS, UNIT_RAW);
      dxl.setGoalPWM(5, -POS, UNIT_RAW);
    }
    else if (msg[1] == 'C')
    { 
      moving = true;
      dxl.setGoalPWM(4, -POS, UNIT_RAW);
      dxl.setGoalPWM(5, POS, UNIT_RAW);
    }
    else
    {
      Serial1.println("msg2[1] is not == to c || o");
    }

    // gripper fixed code 2
    if (dxl.readControlTableItem(PRESENT_LOAD, 4) < -300 && dxl.readControlTableItem(PRESENT_LOAD, 5) > 300 && moving)
    {
      moving = false;
      m4Pos = dxl.getPresentPosition(4);
      m5Pos = dxl.getPresentPosition(5);
      dxl.torqueOff(4);
      dxl.setOperatingMode(4, OP_POSITION);
      dxl.torqueOn(4);
      dxl.torqueOff(5);
      dxl.setOperatingMode(5, OP_POSITION);
      dxl.torqueOn(5);
      dxl.setGoalPosition(4, m4Pos);
      dxl.setGoalPosition(5, m5Pos);
    }


    //__asm__ volatile ("cli");
    

      for(int i = 1; i <= 5; i++)
      {
        if(dxl.readControlTableItem(PRESENT_TEMPERATURE, i))
        {
          
        }
        else if(dxl.readControlTableItem(PRESENT_TEMPERATURE, i) == 0)
        {
          Serial1.print("ERROR motor: ");
          Serial1.print(i);
          Serial1.println("is not connected");
        }
      }
    //__asm__ volatile ("sei");
    Serial1.print("t3msec: ");
    Serial1.println(millis()-t3Millis);
    
    */
    k_sleep(100);
  }
}

void t4(void){

  uint8_t DXL_ID = 1;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 20);
  dxl.torqueOn(DXL_ID);

  DXL_ID = 2;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 20);
  dxl.torqueOn(DXL_ID);

  int lostMessages;
  char msg[20];
  char tempMsg[7];
  int tempPos;
  int lastKomma;

  bool serialDebug = false;


  float theta1;
  float theta2;
  float theta3;

  float ddtheta1;

  float mObj = 0;
  

  while (1)
  {
    k_wait(posSem, 0);

    unsigned long t4Millis = millis();

    k_receive(msgQ2, &msg, 10, &lostMessages);

    theta1 = (-dxl.getCurPosition(1) + 2700) * 0.088;
    theta2 = (dxl.getCurPosition(2) - 1038) * 0.088;
    theta3 = (dxl.getCurPosition(3) - 1535) * 0.088;
    //calculatedd(1);

    for (int i = 0; i < 7; i++)
    {
      tempMsg[i] = ' ';
    }

    for (int i = 1; i < 7; i++)
    {
      if (msg[i] == ',')
      {
        lastKomma = i + 1;
        i = 100;
      }
      else
      {
        tempMsg[i - 1] = msg[i];
      }
    }
    tempPos = 2700 - atoi(tempMsg);





    //I13_3*ddtheta1 + 0.555*sin(theta2)*mObj*(sin(theta1)*cos(theta2)*cos(theta3) - 1.*cos(theta1)*sin(theta3)) - 0.555*sin(theta2)*cos(theta2)*mObj*(cos(theta1)*cos(theta2)*cos(theta3) + 1.*sin(theta1)*sin(theta3));



    dxl.setGoalPosition(1, tempPos);
    





    //M2

    for (int i = 0; i < 7; i++)
    {
      tempMsg[i] = ' ';
    }

    for (int i = lastKomma; i < 13; i++)
    {
      if (msg[i] == ',')
      {
        lastKomma = i + 1;
        i = 100;
      }
      else
      {
        tempMsg[i - lastKomma] = msg[i];
      }
    }
    tempPos = 1170 - atoi(tempMsg);
    dxl.setGoalPosition(2, tempPos);


    for(int i = 1; i <= 5 && serialDebug; i++){
      Serial1.print(" M");
      Serial1.print(i);
      Serial1.print(" load: ");
      Serial1.print(dxl.readControlTableItem(PRESENT_LOAD, i));
      Serial1.print(" Temp: ");
      Serial1.print(dxl.readControlTableItem(PRESENT_TEMPERATURE, i));
      Serial1.print(" Angle: ");
      Serial1.println(dxl.getPresentPosition(i));
    }


    //Serial1.print("t4msec: ");
    //Serial1.println(millis()-t4Millis);
    k_sleep(80);

  }
}




void setup()
{
  // Serial definition--
  // Native serial port (switches from USB-port to dynamixel with physical switch)
  dxl.begin(115200);
  // second serial (the one that communicates with UNO)
  Serial1.begin(57600);
  //--

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // if (manifacturing) info is needed from a specific motor use below
  // dxl.ping(DXL_ID);

  // init krnl so you can create x tasks, y semaphores and z message queues
  k_init(4, 3, 3);

  msgQ = k_crt_send_Q(1, sizeof(char[20]), dataBufForMsgQ);
  msgQ2 = k_crt_send_Q(1, sizeof(char[20]), dataBufForMsgQ2);
  msgQ3 = k_crt_send_Q(1, sizeof(char[20]), dataBufForMsgQ3);

  // each pt(n) is a pointer to a function t(n), priority, stack size
  pserialHandler = k_crt_task(serialHandler, 1, 1000);
  pcurrent = k_crt_task(current, 2, 1000);
  pgripper = k_crt_task(gripper, 4, 1000);
  pt4 = k_crt_task(t4, 3, 1000);

  gripSem = k_crt_sem(0, 1);
  posSem = k_crt_sem(0, 1);
  curSem = k_crt_sem(0, 1);

  // stack size----
  // Arudino Mega has 8 kByte RAM
  // stack is used in funcion calls for:
  //  return address, registers stakked, local variables in a function
  //
  //!!! to find the unused stacksize call "k_unused_stack" !!!
  Serial1.println("succ me ass");
  // start kernel with tick speed 1 milli seconds
  int res;
  res = k_start(1); // 1 milli sec tick speed
  // you will never return from k_start
  Serial1.print("ups an error occured: ");
  Serial1.println(res);
}

void loop()
{
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

  // Serial1.print("Im not supposed to be here.");
}
