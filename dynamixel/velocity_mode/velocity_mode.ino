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

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const float DXL_PROTOCOL_VERSION = 1.0;

DynamixelShield dxl_1;
DynamixelShield dxl_2;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl_1.begin(57600);
  dxl_2.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_1.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_2.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information
  dxl_1.ping(DXL_ID_1);
  dxl_2.ping(DXL_ID_2);

  // Turn off torque when configuring items in EEPROM area
  dxl_1.torqueOff(DXL_ID_1);
  dxl_1.setOperatingMode(DXL_ID_1, OP_EXTENDED_POSITION);
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, OP_EXTENDED_POSITION);
  dxl_2.torqueOn(DXL_ID_2);
}

void loop() {
//  // put your main code here, to run repeatedly:
//  
//  // Please refer to e-Manual(http://emanual.robotis.com) for available range of value. 
//  // Set Goal Velocity using RAW unit
//  dxl_1.setGoalVelocity(DXL_ID_1, 200);
//  dxl_2.setGoalVelocity(DXL_ID_2, 200);
//  delay(1000);
//  // Print present velocity
//  DEBUG_SERIAL.print("Present Velocity(raw) : ");
//  DEBUG_SERIAL.println(dxl_1.getPresentVelocity(DXL_ID_1));
//
//  DEBUG_SERIAL.print("Present Velocity(raw) : ");
//  DEBUG_SERIAL.println(dxl_2.getPresentVelocity(DXL_ID_2));
//  delay(1000);
//
//  // Set Goal Velocity using RPM
//  dxl_1.setGoalVelocity(DXL_ID_1, 25.8, UNIT_RPM);
//  dxl_2.setGoalVelocity(DXL_ID_2, 25.8, UNIT_RPM);
//  delay(1000);
//  DEBUG_SERIAL.print("Present Velocity(rpm) : ");
//  DEBUG_SERIAL.println(dxl_1.getPresentVelocity(DXL_ID_1, UNIT_RPM));
//  delay(1000);
//
//  // Set Goal Velocity using percentage (-100.0 [%] ~ 100.0 [%])
//  dxl_1.setGoalVelocity(DXL_ID_1, -10.2, UNIT_PERCENT);
//  dxl_2.setGoalVelocity(DXL_ID_2, -10.2, UNIT_PERCENT);
//  delay(1000);
//  DEBUG_SERIAL.print("Present Velocity(ratio) : ");
//  DEBUG_SERIAL.println(dxl_1.getPresentVelocity(DXL_ID_1, UNIT_PERCENT));
//
//  DEBUG_SERIAL.print("Present Velocity(ratio) : ");
//  DEBUG_SERIAL.println(dxl_2.getPresentVelocity(DXL_ID_2, UNIT_PERCENT));
//  delay(1000);
  motor();
}

void motor(){
  dxl_1.setGoalPosition(DXL_ID_1, 512);
  delay(1000);
  // Print present position in raw value
  DEBUG_SERIAL.print("Present Position(raw) : ");
  DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1));
  delay(1000);

  // Set Goal Position in DEGREE value
  dxl_1.setGoalPosition(DXL_ID_1, 5.7, UNIT_DEGREE);
  delay(1000);
  // Print present position in degree value
  DEBUG_SERIAL.print("Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_DEGREE));
  delay(1000);
}
