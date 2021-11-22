#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EMG.h>

//
EMG emg;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 2000;

int sign = 2;

bool gripperOpen = false;

SoftwareSerial mySerial(10, 11); // RX, TX

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ;
double aX_angle, aY_angle, aZ_angle; 

const int filterOrder = 2;

float aX_filtered[filterOrder];
float aY_filtered[filterOrder];
float aZ_filtered[filterOrder];
int last = 0;

double aX_show = 0;
double aY_show = 0;
double aZ_show = 0;

int sendroll;
int sendpitch;

float roll,pitch; // roll == M1, pitch == M2 bacts

int BaseEMG1 = 1000;
int BaseEMG2 = 1000;
const int EMGdev = 25;

int J3goalPose = 2020;
float tau3 = 0;
float goalcurrent = 0;
const float m3 = 0.306; // kg 
const float [3] I3 = {11.4, 4.5, 162.7};


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

    theta1 = atan(aY_show / sqrt(pow(aX_show, 2) + pow(aZ_show, 2))) * 180 / 3.14;
    theta2 = atan(-1 * aX_show / sqrt(pow(aY_show, 2) + pow(aZ_show, 2))) * 180 / 3.14;


    if( aZ_show < 0 && aY_show < 0){
       theta2 = -180-pitch;
    }
    /*else if (aZ_show < 0){
      pitch = -180-pitch; 
      }*/
    else if( aZ_show < 0 && aY_show > 0){
      // probably nothing
      }
    

    //conversion to dxl units
    roll = theta1 / 0.088;
    pitch = theta2 /0.088;

    sendroll = int(roll);
    sendpitch = int(pitch);

    //main delay issue, delay is 5ms without, 115 with
    emg.GetInput(1);
    int EMG1 = emg.EMG1();
    int EMG2 = emg.EMG2();

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
      gripperOpen = true;
      startMillis = currentMillis;
      }
      else if(currentMillis - startMillis >= period) {
      mySerial.print("<GC>");
      gripperOpen = false;
      startMillis = currentMillis;
      } 
     }

    else if (EMG1 > BaseEMG1 + 2*EMGdev){ //J3 positive direction
      J3goalPose =+ 75;
      BaseEMG2 = makeBaseline(EMG2, BaseEMG2);
      }
    else if (EMG2 > BaseEMG2 + 0.5*EMGdev){ //J3 negative direction
      J3goalPose =- 75;
      BaseEMG1 = makeBaseline(EMG1, BaseEMG1);
      }
    else{
        BaseEMG1 = makeBaseline(EMG1, BaseEMG1);  
        BaseEMG2 = makeBaseline(EMG2, BaseEMG2);
        }
    
        
    float gcrossSc3 = -(1963*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2))*((33*cos(theta3))/250 + 2279/10000))/200 - (1963*cos(theta1)*cos(theta2)((33*sin(theta3))/250 + 113/20))/200
    tau3 = angACC*I3[1]+angACC*I3[2]+angACC*I3[3]+m3*gcrossSc3;

    mySerial.print("<P");
    mySerial.print(String(sendroll));
    mySerial.print(",");
    mySerial.print(String(sendpitch));
    mySerial.print(",");
    mySerial.print(String(sign));
    mySerial.print(">\n");
    
    
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
  delay(10); //might be able to delete it
}

int makeBaseline(int sEMG, int curBase){

return 0.4*sEMG + 0.6*curBase;
}