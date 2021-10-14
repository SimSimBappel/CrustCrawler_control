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




//----------------------------------------------------
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





    /*
    char res;
  int lostMessages;
  char msg[20];
  
  int tempCurrent;
  

  unsigned long prevMillis = 0;
  bool serialDebug = true;

  while(1){
    k_wait(curSem, 0);
    prevMillis = millis();

    res = k_receive(msgQ2, &msg, 10, &lostMessages);  
    
    for(int x = 1; x < 4; x++){
      char tempMsg[5]= {' ',' ',' ',' ',' '};
      int startVal = 5*x-4;
      int endVal = 5*x+1;
      Serial1.print("start: ");
      Serial1.print(startVal);
      Serial1.print(" endval: ");
      Serial1.println(endVal);
      
      for(int i = startVal; i < endVal; i++){
        tempMsg[i-startVal+1] = msg[i];
      }
      tempCurrent = atoi(tempMsg);
      
      if(serialDebug){
        Serial1.print("tempMsg: ");
        Serial1.println(tempMsg);
        Serial1.print("tempCurrent: ");
        Serial1.println(tempCurrent);
      }
      
      tempCurrent = tempCurrent * 10; // input from milli- to centiampere
      dxl.setGoalCurrent(x, tempCurrent, UNIT_MILLI_AMPERE);
      delay(10);
    }
*/
      