#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EMG.h>

EMG emg;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 2000;

int sign = 2;

bool gripperToggle = false;

SoftwareSerial mySerial(10, 11); // RX, TX

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ;
double aX_angle, aY_angle, aZ_angle; 

float aX_filtered[10];
float aY_filtered[10];
float aZ_filtered[10];
int last = 0;

double aX_show = 0;
double aY_show = 0;
double aZ_show = 0;

int sendroll;
int sendpitch;

float roll,pitch; // roll == M1, pitch == M2 bacts

int BaseEMG1 = 500;
int BaseEMG2 = 500;
const int EMGdev = 5;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  mySerial.begin(57600);

  startMillis = millis();

  bme.begin();
  mySensor.beginAccel();
}

void loop() {

  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aZ = aZ + 0.43;

    //filter 
    aX_filtered[last] = aX;
    aY_filtered[last] = aY;
    aZ_filtered[last] = aZ;

    if(last<9){
    last++;  
              }
    else{
    last = 0;  
        }
    }
    for(int i = 0; i < 10 ; i++){
      aX_show += aX_filtered[i];
      aY_show += aY_filtered[i];
      aZ_show += aZ_filtered[i];
    
    }
    aX_show=aX_show/10;
    aY_show=aY_show/10;
    aZ_show=aZ_show/10;

     

    roll = atan(aY_show / sqrt(pow(aX_show, 2) + pow(aZ_show, 2))) * 180 / 3.14;
    pitch = atan(-1 * aX_show / sqrt(pow(aY_show, 2) + pow(aZ_show, 2))) * 180 / 3.14;

      /*
    if(aZ_show < 0 && aX_show > 0){ // et forsøg på at fixe joint 2's udregninger over vandret, virker ikke tho.
      pitch = - 180 - pitch;

      }*/

    if(aZ_show < 0){
      roll = 180 - roll;
      }

    //conversion to dxl units
    roll = roll / 0.088;
    pitch = pitch /0.088;

    sendroll = int(roll);
    sendpitch = int(pitch);

    emg.GetInput(100);
    int EMG1 = emg.EMG1();
    int EMG2 = emg.EMG2();

    Serial.print(EMG1);
    Serial.print("    ");
    Serial.println(EMG2);
    
    currentMillis = millis();

    if (EMG1 > BaseEMG1 +EMGdev){ //&& EMG2 < BaseEMG2 + EMGdev//J3 positive direction
      sign  = 1;
      }
     
   else if (EMG2 > BaseEMG2+ EMGdev){ // EMG1 < BaseEMG1 +EMGdev && //J3 Negative direction
      sign  = 0;
      }
   else{
        sign=2;
        }

    if( EMG1 > BaseEMG1 + EMGdev && EMG2 > BaseEMG2 + EMGdev && gripperToggle == false && currentMillis - startMillis >= period){ //Opens the gripper
      mySerial.print("<GO>");
      gripperToggle = true;
      startMillis = currentMillis;
      }
    else BaseEMG1 = makeBaseline(EMG1, BaseEMG1);

    if(EMG1 > BaseEMG1 + EMGdev && EMG2 > BaseEMG2 + EMGdev && gripperToggle == true && currentMillis - startMillis >= period){ //Closes the gripper
      mySerial.print("<GC>");
      gripperToggle = false;
      startMillis = currentMillis;
      }
    else  BaseEMG2 = makeBaseline(EMG2, BaseEMG2);

    mySerial.print("<P");
    mySerial.print(String(sendroll));
    mySerial.print(",");
    mySerial.print(String(sendpitch));
    mySerial.print(",");
    mySerial.print(String(sign));
    mySerial.print(">\n");

    Serial.print("baseline: ");
    Serial.print(BaseEMG1);
    Serial.print(" , ");
    Serial.println(BaseEMG2);


    delay(10);
    while (mySerial.available()) {
    Serial.write(mySerial.read());
    delay(5);
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}

int makeBaseline(int sEMG, int curBase){

return 0.4*sEMG + 0.6*curBase;
}