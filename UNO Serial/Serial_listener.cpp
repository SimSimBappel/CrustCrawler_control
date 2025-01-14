#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EMG.h>

//
EMG emg;

unsigned long startMillis;
unsigned long currentMillis;
const int period = 2000;

unsigned long prevmillis=0;

int Direction = 2;

bool gripperOpen = false;

SoftwareSerial mySerial(10, 11); // RX, TX

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ;
double aX_angle, aY_angle, aZ_angle; 

const int filterOrder = 3;

float aX_filtered[filterOrder];
float aY_filtered[filterOrder];
float aZ_filtered[filterOrder];
int last = 0;

float aX_show = 0;
float aY_show = 0;
float aZ_show = 0;

int sendroll;
int sendpitch;

float roll,pitch; // roll == M1, pitch == M2 bacts

int BaseEMG1 = 1030;
int BaseEMG2 = 1030;
const int EMGdev = 25;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  mySerial.begin(57600);

  //startMillis = millis();

  bme.begin();
  mySensor.beginAccel();
}

void loop() {
  
  /*
  int current2Millis = millis();
  Serial.print("delay: ");
  Serial.println(current2Millis-startMillis);
  startMillis = current2Millis;
  */
  if(millis()> prevmillis + 100){
    prevmillis = millis();
    if (mySensor.accelUpdate() == 0) {
      aX = mySensor.accelX();
      aY = mySensor.accelY();
      aZ = mySensor.accelZ();
      aZ = aZ + 0.43;
  
      //filter 
      aX_filtered[last] = aX;
      aY_filtered[last] = aY;
      aZ_filtered[last] = aZ;
  
      if(last<filterOrder-1){
      last++;  
                }
      else{
      last = 0;  
          }
      }
      for(int i = 0; i < filterOrder ; i++){
        aX_show += aX_filtered[i];
        aY_show += aY_filtered[i];
        aZ_show += aZ_filtered[i];
      }
      
      aX_show=aX_show/filterOrder;
      aY_show=aY_show/filterOrder;
      aZ_show=aZ_show/filterOrder;
  
      roll = atan(aY_show / sqrt(pow(aX_show, 2) + pow(aZ_show, 2))) * 180 / 3.14;
      pitch = atan(-1 * aX_show / sqrt(pow(aY_show, 2) + pow(aZ_show, 2))) * 180 / 3.14;
  
      if( aZ_show < 0 && aY_show < 0){
         pitch = -180-pitch;
      }
      else if( aZ_show < 0 && aY_show > 0){
        // probably nothing
        }
  
      //conversion to dxl units
      roll = roll / 0.088;
      pitch = pitch /0.088;
  
      sendroll = int(roll);
      sendpitch = int(pitch);
/*
      Serial.print("aX:");
      Serial.print(aX);
      Serial.print(",");
      Serial.print("aX_show:");
      Serial.println(aX_show);
      
          */
      mySerial.print("<P");
      mySerial.print(String(sendroll));
      mySerial.print(",");
      mySerial.print(String(sendpitch));
      mySerial.print(",>");
      
/*
      Serial.print("<P");
      Serial.print(String(sendroll));
      Serial.print(",");
      Serial.print(String(sendpitch));
      Serial.print(",>");
      */

    }

    
    //main delay issue, delay is 5ms without, 115 with
    
    //emg.reset();
    
    emg.GetInput();
    int EMG1 = emg.EMG1();
    int EMG2 = emg.EMG2();
/*
    Serial.print(EMG1);
    Serial.print("    ");
    Serial.println(EMG2);
*/
    /*
    Serial.print("EMG Signals: ");
    Serial.print(EMG1);
    Serial.print("    ");
    Serial.println(EMG2);
   */
    
    currentMillis = millis();
    if( EMG1 > BaseEMG1 + 2*EMGdev && EMG2 > BaseEMG2 + EMGdev){ //Opens the gripper 
      if(gripperOpen == false && currentMillis - startMillis >= period){
      mySerial.print("<GO>");
      Serial.print("GO");
      gripperOpen = true;
      startMillis = currentMillis;
      }
      else if(currentMillis - startMillis >= period) {
      mySerial.print("<GC>");
      gripperOpen = false;
      startMillis = currentMillis;
      Serial.println("GC");
      } 
     }

    else if (EMG1 > BaseEMG1 + 2*EMGdev){ //J3 positive direction
      Direction = 1;
      BaseEMG2 = makeBaseline(EMG2, BaseEMG2);
      }
    else if (EMG2 > BaseEMG2 + 0.5*EMGdev){ //J3 negative direction
      Direction = 0;
      BaseEMG1 = makeBaseline(EMG1, BaseEMG1);
      }
    else{
      Direction = 2;
        BaseEMG1 = makeBaseline(EMG1, BaseEMG1);  
        BaseEMG2 = makeBaseline(EMG2, BaseEMG2);
        }

   


    mySerial.print("<C");
    mySerial.print(String(Direction));
    mySerial.print(">\n");

    /*Serial.print("<C");
    Serial.print(String(Direction));
    Serial.print(">\n");
    */
    
    /*
    Serial.print("baseline: ");
    Serial.print(BaseEMG1);
    Serial.print(" , ");
    Serial.println(BaseEMG2);
    */

  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
  
  delay(100); //might be able to delete it
}



int makeBaseline(int sEMG, int curBase){

return int(0.4*sEMG + 0.6*curBase);
}