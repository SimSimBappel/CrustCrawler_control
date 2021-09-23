#include <Arduino.h>

/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/** Please refer to each DYNAMIXEL eManual(http://emanual.robotis.com) for supported Operating Mode
 * Operating Mode
 *  1. OP_POSITION                (Position Control Mode in protocol2.0, Joint Mode in protocol1.0)
 *  2. OP_VELOCITY                (Velocity Control Mode in protocol2.0, Speed Mode in protocol1.0)
 *  3. OP_PWM                     (PWM Control Mode in protocol2.0)
 *  4. OP_EXTENDED_POSITION       (Extended Position Control Mode in protocol2.0, Multi-turn Mode(only MX series) in protocol1.0)
 *  5. OP_CURRENT                 (Current Control Mode in protocol2.0, Torque Mode(only MX64,MX106) in protocol1.0)
 *  6. OP_CURRENT_BASED_POSITION  (Current Based Postion Control Mode in protocol2.0 (except MX28, XL430))
 */

#include <DynamixelShield.h>


const uint8_t DXL_ID = 4;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  Serial1.begin(9600);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Position Control Mode in protocol2.0, Joint Mode in protocol1.0
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 512)){
    delay(1000);
    Serial1.print("Present Position : ");
    Serial1.println(dxl.getPresentPosition(DXL_ID)); Serial1.println();
  }

  // Velocity Contorl Mode in protocol2.0, Speed Mode in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalVelocity(DXL_ID, 128)){
    delay(1000);
    Serial1.print("Present Velocity : ");
    Serial1.println(dxl.getPresentVelocity(DXL_ID)); Serial1.println();
  }

  // Extended Position Contorl Mode in protocol2.0, Multi-turn Mode(only MX series) in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 4096)){
    delay(1000);
    Serial1.print("Present Extended Position : ");
    Serial1.println(dxl.getPresentPosition(DXL_ID)); Serial1.println();
  }

  // PWM Contorl Mode in protocol2.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPWM(DXL_ID, 200)){
    delay(1000);
    Serial1.print("Present PWM : ");
    Serial1.println(dxl.getPresentPWM(DXL_ID)); Serial1.println();
  }
  
  // Current Contorl Mode in protocol2.0, Torque Mode(only MX64,MX106) in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalCurrent(DXL_ID, 256)){
    delay(1000);
    Serial1.print("Present Current : ");
    Serial1.println(dxl.getPresentCurrent(DXL_ID)); Serial1.println();
  }
  
  // Current Based Postion Contorl Mode in protocol2.0 (except MX28, XL430)
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 8192)){
    delay(1000);
    Serial1.print("Present Current Based Position : ");
    Serial1.println(dxl.getPresentPosition(DXL_ID)); Serial1.println();
  }
}