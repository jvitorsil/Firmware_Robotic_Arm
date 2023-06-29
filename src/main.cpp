/**
**************************************************************************************************************
* @file Arm_main.cpp
* @author João Vitor Silva <joaovitor_s2015@ufu.br>
* @version V0.1.0
* @date 26-Jun-2023
* @brief code for PET project - HOOK 4º ed.
*************************************************************************************************************
*/

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Servo.h>


/* Constants ---------------------------------------------------------------------------------------------------------*/


/* Pin numbers -------------------------------------------------------------------------------------------------------*/
#define Servo_Joint_Left_1_PIN 18  // Ombro
#define Servo_Joint_Right_1_PIN 19 // Ombro
#define Servo_Joint_3_PIN 21 // Cotovelo 
#define Servo_Joint_4_PIN 22 // Punho
#define Servo_Joint_5_PIN 23 // Punho

#define DC_Gripper_Upper_PIN 35
#define DC_Gripper_Lower_PIN 32

/* Private variables -------------------------------------------------------------------------------------------------*/
Servo servoJointLeft;
Servo servoJointRight; 
Servo servoJoint3;
Servo servoJoint4;
Servo servoJoint5;

uint8_t posJointLeft = 90, posJointRight = 90, posJoint3 = 90, posJoint4 = 90, posJoint5 = 90;


typedef struct struct_message{
  float_t pitch;
  float_t roll;
  uint16_t flexValue;
  uint16_t goniometerValue;
  uint8_t buttonState;

} struct_message;

struct_message myData;

/* Private functions -------------------------------------------------------------------------------------------------*/
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);


/* Main Application --------------------------------------------------------------------------------------------------*/
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
    return;

  esp_now_register_recv_cb(OnDataRecv);

  pinMode(DC_Gripper_Upper_PIN, OUTPUT);
  pinMode(DC_Gripper_Lower_PIN, OUTPUT);


  servoJointLeft.attach(Servo_Joint_Left_1_PIN);
  servoJointLeft.write(posJointLeft);

  servoJointRight.attach(Servo_Joint_Right_1_PIN);
  servoJointRight.write(posJointRight);

  servoJoint3.attach(Servo_Joint_3_PIN);
  servoJoint3.write(posJoint3);

  servoJoint4.attach(Servo_Joint_4_PIN);
  servoJoint4.write(posJoint4);

  servoJoint5.attach(Servo_Joint_5_PIN);
  servoJoint5.write(posJoint5);
}

void loop(){

  uint16_t angle = map(myData.goniometerValue, 0, 4095, 0, 180);
  int targetPosJoint3 = angle;
  int diffJoint3 = targetPosJoint3 - posJoint3;

  if (diffJoint3 != 0) {
    int incrementJoint3 = diffJoint3 / abs(diffJoint3);
    posJoint3 += incrementJoint3;
    servoJoint3.write(posJoint3);
    delay(15);
    return;
  }

    // put your main code here, to run repeatedly:
  digitalWrite(DC_Gripper_Upper_PIN, HIGH);
  digitalWrite(DC_Gripper_Lower_PIN, LOW);
  delay(200);


  Serial.println("Pitch: " + String(myData.pitch) + " | Roll: " + String(myData.roll) + " | Goniometer R: " + String(myData.goniometerValue) + " | Push Button: " + String(myData.buttonState) + " | Flex Sensor: " + String(myData.flexValue) + " | Angle: " + String(angle));


  if (myData.pitch > 15 && myData.buttonState == HIGH) {
    posJointLeft = min(180, posJointLeft + 1);
    posJointRight = max(0, posJointRight - 1);
    servoJointLeft.write(posJointLeft);
    servoJointRight.write(posJointRight);
    delay(15);
  } else if (myData.pitch < -15 && myData.buttonState == HIGH) {
    posJointLeft = max(0, posJointLeft - 1);
    posJointRight = min(180, posJointRight + 1);
    servoJointLeft.write(posJointLeft);
    servoJointRight.write(posJointRight);
    delay(15);
  } else if (myData.roll > 15 && myData.buttonState == HIGH) {
    // Código Stepper motor
  } else if (myData.roll < 15 && myData.buttonState == HIGH) {
    // Código Stepper motor
  } else if (myData.pitch > 15 && myData.buttonState == LOW) {
    posJoint4 = min(180, posJoint4 + 1);
    servoJoint4.write(posJoint4);
    delay(15);
  } else if (myData.pitch < -15 && myData.buttonState == LOW) {
    posJoint4 = max(0, posJoint4 - 1);
    servoJoint4.write(posJoint4);
    delay(15);
  } else if (myData.roll > 15 && myData.buttonState == LOW) {
    posJoint5 = min(180, posJoint5 + 1);
    servoJoint5.write(posJoint5);
    delay(15);
  } else if (myData.roll < -15 && myData.buttonState == LOW) {
    posJoint5 = max(0, posJoint5 - 1);
    servoJoint5.write(posJoint5);
    delay(15);
  } 
}


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println("Pitch: " + String(myData.pitch) + " | Roll: " + String(myData.roll) + " | Goniometer R: " + String(myData.goniometerValue) + " | Push Button: " + String(myData.buttonState) + " | Flex Sensor: " + String(myData.flexValue));
}

