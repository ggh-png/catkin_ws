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
int position = 0;
DynamixelShield dxl_1;
DynamixelShield dxl_2;

//--------------------------------------------------------------

int num_1 = 0;
int num_2 = 0;
#include <FreeRTOS_AVR.h>
SemaphoreHandle_t sem;

//
//static void Thread1(void* arg);
//static void Thread2(void* arg);
//
//void motor_1(int n_1);
//void motor_2(int n_2);

void setup() {

  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl_1.begin(2000000);
  dxl_2.begin(2000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_1.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_2.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information
  dxl_1.ping(DXL_ID_1);
  dxl_2.ping(DXL_ID_2);

  // Turn off torque when configuring items in EEPROM area
  dxl_1.torqueOff(DXL_ID_1);
  dxl_1.setOperatingMode(DXL_ID_1,  OP_VELOCITY);
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2,  OP_VELOCITY);
  dxl_2.torqueOn(DXL_ID_2);

//-------------------------------------------------------------
//  portBASE_TYPE s1, s2;
//
//  sem = xSemaphoreCreateCounting(1, 0);
//  s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL); 
//  s2 = xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL); 
//  vTaskStartScheduler(); 
//  while(1) {}
//  



//
//static void Thread1(void* arg) {
//  while (1) {
//  motor_1(3);
//  }
//}
//
//
//static void Thread2(void* arg) {
//  while (1) {
//  motor_2(3);
//  }
//}


void motor_1(int n_1, int n_2){
  if(n_1 != num_1){
    dxl_1.setGoalVelocity(DXL_ID_1, 100);
    dxl_2.setGoalVelocity(DXL_ID_2, 100);
    
    int enc_1 = dxl_1.getPresentPosition(DXL_ID_1) - 4060;
    int avg_1 = 0;
    int sum_1 = 0;
    
    if(enc_1 > 25) {
      for(int i = 0; i < 2; i++){  
        sum_1 = sum_1 + enc_1;
      }
  
      if(sum_1 > 50){
        num_1++;
        delay(100);
        Serial.print("Average : ");
        Serial.println(num_1);
        Serial.println(num_1);
        Serial.println(num_1); 
      }
    }
  }
  else{
    dxl_1.setGoalVelocity(DXL_ID_1, 0);
  }
}


void motor_2(int n_2){
  if(n_2 != num_2){
    dxl_2.setGoalVelocity(DXL_ID_2, 100);
    int enc_2 = dxl_2.getPresentPosition(DXL_ID_2) - 4060;
    int avg_2 = 0;
    int sum_2 = 0;
    
    if(enc_2 > 25) {
      for(int i = 0; i < 2; i++){  
        sum_2 = sum_2 + enc_2;
      }
  
      if(sum_2 > 50){
        num_2++;
        delay(100);
        Serial.print("Average : ");
        Serial.println(num_2);
        Serial.println(num_2);
        Serial.println(num_2); 
      }
    }
  }
  else{
    dxl_2.setGoalVelocity(DXL_ID_2, 0);
  }
}
  
void loop() {
  motor_1(3);
  motor_2(3);
}
