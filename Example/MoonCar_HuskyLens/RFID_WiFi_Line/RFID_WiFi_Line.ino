/*
 * Generated using BlocklyDuino:
 *
 * https://github.com/MediaTek-Labs/BlocklyDuino-for-LinkIt
 *
 * Date: Wed, 05 May 2021 10:39:56 GMT
 */

/*

 * 部份程式碼由吉哥積木產生
 * https://sites.google.com/jes.mlc.edu.tw/ljj/linkit7697

*/

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

#include <LWiFi.h>
#include <Wire.h>
#include "MFRC522_I2C.h"

int target;

int motor_L;

int motor_R;

String rfid;

HUSKYLENS huskylens;
SoftwareSerial mySerial(2, 3);
void printResult(HUSKYLENSResult result);

char _lwifi_ssid[] = "iCShop_1";
char _lwifi_pass[] = "075564686";
const char* asId="AKfycbyR-Yp-uu4nIvnjvnkILaQ5AX8yFxp-UpBO-Sqs0su3ai1N_BvQsz_Q";
String sheetId="";
String sheetTag="";

String URLEncode(const char* msg)
{
  const char *hex = "0123456789abcdef";
  String encodedMsg = "";
  while (*msg!='\0'){
      if( ('a' <= *msg && *msg <= 'z')
              || ('A' <= *msg && *msg <= 'Z')
              || ('0' <= *msg && *msg <= '9') ) {
          encodedMsg += *msg;
      } else {
          encodedMsg += '%';
          encodedMsg += hex[*msg >> 4];
          encodedMsg += hex[*msg & 15];
      }
      msg++;
  }
  return encodedMsg;
}

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

MFRC522 mfrc522(0x28);

String readRFID() {
  String mfrc522ReadCode = "";
  if ( ! mfrc522.PICC_IsNewCardPresent() || ! mfrc522.PICC_ReadCardSerial() ) {}
  else {
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      mfrc522ReadCode += String(mfrc522.uid.uidByte[i], HEX);
    }
  }
  return mfrc522ReadCode;
}

void  sendToGoogleSheets(const String& dateInclude,const String& data)
{
  static TLSClient sheetClient;
  const char* host="script.google.com";
  if (sheetClient.connect(host, 443)) {
      const String url = String() +"https://"+host+"/macros/s/"+asId+"/exec?type=insert&dateInclude="+dateInclude+"&sheetId="+sheetId+"&sheetTag="+sheetTag+"&data="+data;
      sheetClient.println("GET " + url + " HTTP/1.1");
      sheetClient.println(String()+"Host: "+host);
      sheetClient.println("Accept: */*");
      sheetClient.println("Connection: close");
      sheetClient.println();
      sheetClient.println();
      sheetClient.stop();
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
  while (WiFi.begin(_lwifi_ssid, _lwifi_pass) != WL_CONNECTED) { delay(1000); }
  sheetId="1L1v3Wch0S7yPmfk-pO82ByfS_868aONRgyby0o5tKAE";
  sheetTag=URLEncode("Sheet1");
  pinMode(17, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(13, OUTPUT);

  Wire.begin();
  mfrc522.PCD_Init();

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
  rfid = readRFID();
  if (rfid != "") {
    motor_LR(0, 0);
    sendToGoogleSheets("1",URLEncode((String() + "RFID: " + "," + rfid).c_str()));
    motor_LR(100, 100);
    delay(500);

  }
}