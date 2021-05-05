/*
 * Generated using BlocklyDuino:
 *
 * https://github.com/MediaTek-Labs/BlocklyDuino-for-LinkIt
 *
 * Date: Wed, 05 May 2021 09:17:49 GMT
 */

/*

 * 部份程式碼由吉哥積木產生
 * https://sites.google.com/jes.mlc.edu.tw/ljj/linkit7697

*/

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

int target;

int motor_L;

int motor_R;

HUSKYLENS huskylens;
SoftwareSerial mySerial(2, 3);
void printResult(HUSKYLENSResult result);

int readData[5] = {};
byte dataType = 0;
byte idCount = 0;

void motor_LR(int ML, int MR)
{
  if (ML > 0) {
    analogWrite(17, ML);
    analogWrite(12, 0);
  }else {
    analogWrite(17, 0);
    analogWrite(12, ML*(-1));
  }
  if (MR > 0) {
    analogWrite(16, MR);
    analogWrite(13, 0);
  }else {
    analogWrite(16, 0);
    analogWrite(13, MR*(-1));
  }
}

void setup()
{
  mySerial.begin(9600);
  while (!huskylens.begin(mySerial)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }

  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
  pinMode(17, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(13, OUTPUT);

}


void loop()
{
  if (!huskylens.request()) {
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  }
  else {
    if (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      idCount = huskylens.countLearned();
      if (result.command == COMMAND_RETURN_BLOCK){
        dataType = 0;
        readData[0] = result.xCenter;
        readData[1] = result.yCenter;
        readData[2] = result.width;
        readData[3] = result.height;
        readData[4] = result.ID;
      }
      else if (result.command == COMMAND_RETURN_ARROW){
        dataType = 1;
        readData[0] = result.xOrigin;
        readData[1] = result.yOrigin;
        readData[2] = result.xTarget;
        readData[3] = result.yTarget;
        readData[4] = result.ID;
      }
      else {
        for (byte i=0; i<5; i++) {
          readData[i] = 0;
        }
      }
    }
  }
  target = readData[2] - 160;
  motor_L = (constrain(255 + (map(target,-160,160,-255,255)), 0,255));
  motor_R = (constrain(255 - (map(target,-160,160,-255,255)), 0,255));
  motor_LR(motor_L, motor_R);
}