    /*
  by MohammedDamirchi
  Home
*/
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <SoftwareSerial.h>

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

int fjollet = 0;
void setup() {
  Serial.begin(57600);
  while (!Serial);
  mySerial.begin(57600);


  bme.begin();
  mySensor.beginAccel();
  //mySensor.beginGyro();
  //mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

void loop() {

  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aZ = aZ + 0.43;
    //aY = aY - 0.11;

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

    //conversion to dxl units
    roll = roll / 0.088;
    pitch = pitch /0.088;

    sendroll = int(roll);
    sendpitch = int(pitch);

    mySerial.print("<P");
    mySerial.print(String(sendroll));
    mySerial.print(",");
    mySerial.print(String(sendpitch));
    mySerial.print(">\n");

/*
    //Serial.print("<P");
    Serial.print(int(roll));
    Serial.print(",");
    Serial.println(int(pitch));
    //Serial.print(">\n");
    */
    delay(100);
    while (mySerial.available()) {
    Serial.write(mySerial.read());
    delay(5);
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}