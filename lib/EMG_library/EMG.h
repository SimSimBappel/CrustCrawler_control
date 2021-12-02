#ifndef EMG_h
#define EMG_h

#include "Arduino.h"

class EMG 
{
  public:
    EMG();
    void GetInput();
    void printInput();
    void reset();
    int AccX();
    int AccY();
    int AccZ();
    int EMG1();
    int EMG2();

  private:
    int _str[24];
};

#endif