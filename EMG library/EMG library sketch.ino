#include "EMG.h"

EMG emg;
int x;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  emg.GetInput(100);
  /*
  Serial.print("AccX : ");
  Serial.println(emg.AccX());

  Serial.print("AccY : ");
  Serial.println(emg.AccY());

  Serial.print("AccZ : ");
  Serial.println(emg.AccZ());
  */

  Serial.print("EMG1 : ");
  Serial.println(emg.EMG1());
/*
  Serial.print("EMG2 : ");
  Serial.println(emg.EMG2());
*/
}