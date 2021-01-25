/*
 * Generated using BlocklyDuino:
 *
 * https://github.com/MediaTek-Labs/BlocklyDuino-for-LinkIt
 *
 * Date: Fri, 30 Oct 2020 09:47:28 GMT
 */

/*

 * 部份程式碼由吉哥積木產生
 * https://sites.google.com/jes.mlc.edu.tw/ljj/linkit7697

*/

#include "Wire.h"
#include "U8g2lib.h"
#include <SoftwareSerial.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
SoftwareSerial Serial2_(2, 3);

String Data_Main_Receive() {
  char result_buf[256] = "";
  if (Serial2_.available() > 0) {
    int recv_len = Serial2_.readBytesUntil('\r', result_buf, 256);
    Serial2_.flush();
    String message_ = "";
    for (int i=0; i<256; i++) {
      message_ += String(result_buf[i]);
    }
    return message_;
  }
  else {
    return "None";
  }
}

bool a_button()
{
  if (digitalRead(0) == 0 && digitalRead(7) == 1) {
    return true;
  } else {
    return false;
  }
}

bool b_button()
{
  if (digitalRead(0) == 1 && digitalRead(7) == 0) {
    return true;
  } else {
    return false;
  }
}

bool c_button()
{
  if (digitalRead(0) == 0 && digitalRead(7) == 0) {
    return true;
  } else {
    return false;
  }
}

void Motion_Detect_mode_Send(int mode_) {
  String TT;
  char input_[100] = "";
  if (mode_ == 0) {
    TT = "{\"MOTION DETECT\": 1.0, \"mode\": \"COMPUTE_MODE_STATIC\"}";
    TT.toCharArray(input_, 100);
    Serial2_.print(input_);
  }
  else {
    TT = "{\"MOTION DETECT\": 1.0, \"mode\": \"COMPUTE_MODE_DYNAMIC\"}";
    TT.toCharArray(input_, 100);
    Serial2_.print(input_);
  }
}

const unsigned char F3[] U8X8_PROGMEM= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0xe0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0xe0,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0xff,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0xff,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0xff,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0x03,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0x03,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0xff,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0xff,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0xff,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0xe0,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0xe0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1c,0x00,0x00,0x80,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7e,0x00,0x00,0xe0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x01,0x00,0xf8,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x03,0x00,0xfe,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0x0f,0x80,0xff,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x7f,0xf0,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xff,0xff,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xff,0xff,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xff,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
String Motion_Detect_Main_(int data_) {
  String new_Text_ = Data_Main_Receive();
  if (new_Text_ == "None") {
    return "None";
  }
  else {
    int diff_total = new_Text_.indexOf(",\"DIFF TOTAL\":");
    int diff_max = new_Text_.indexOf(",\"DIFF MAX\":");
    int total = new_Text_.indexOf(",\"TOTAL\":");
    if (data_ == 0) {
      String diff_total_;
      for (int i=diff_total+14; i<diff_max; i++) {
        diff_total_ += new_Text_[i];
      }
      return diff_total_;
    }
    else if (data_ == 1) {
      String diff_max_;
      for (int i=diff_max+12; i<total; i++) {
        diff_max_ += new_Text_[i];
      }
      return diff_max_;
    }
    else {
      String total_ = String(new_Text_[total+9]);
      return total_;
    }
  }
}

const unsigned char F9[] U8X8_PROGMEM= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0xff,0xff,0x3f,0x00,0x00,0x00,0xe0,0xff,0xff,0xff,0xff,0x01,0x00,0x00,0xfe,0xff,0xff,0xff,0x7f,0x00,0x00,0x00,0xf0,0xff,0xff,0xff,0xff,0x03,0x00,0x00,0xfe,0xff,0xff,0xff,0x7f,0x00,0x00,0x00,0xf0,0xff,0xff,0xff,0xff,0x03,0x00,0x00,0xfe,0xff,0xff,0xff,0x7f,0x00,0x00,0x00,0xf0,0xff,0xff,0xff,0xff,0x03,0x00,0x00,0xfc,0xff,0xff,0xff,0x3f,0x00,0x00,0x00,0xe0,0xff,0xff,0xff,0xff,0x01,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x1c,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x00,0xc0,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xe0,0xff,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0xff,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xe7,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xff,0x00,0xff,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x3f,0x00,0xf8,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xe0,0x1f,0x00,0xf0,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x0f,0x00,0xc0,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0x03,0x00,0x00,0xff,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0x01,0x00,0x00,0xfc,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0x00,0x00,0x00,0xf0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0xe0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x80,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void setup()
{
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);

  Serial.begin(115200);

  Serial2_.begin(115200);

  pinMode(0, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  while (!a_button()) {
  }
  for(int i=0;i<3;i++){Motion_Detect_mode_Send(1);delay(50);}
  u8g2.clearBuffer();
  u8g2.drawXBMP(0, 0, 128, 64, F3);
  u8g2.sendBuffer();
  pinMode(17, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(13, OUTPUT);

}


void loop()
{
  if (Motion_Detect_Main_(1).toInt() > 100) {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, F9);
    u8g2.sendBuffer();
    analogWrite(17, 0);
    analogWrite(12, 255);
    analogWrite(16, 0);
    analogWrite(13, 255);
    delay(100);
    analogWrite(17, 50);
    analogWrite(12, 0);
    analogWrite(16, 50);
    analogWrite(13, 0);
    delay(300);
    analogWrite(17, 0);
    analogWrite(12, 0);
    analogWrite(16, 0);
    analogWrite(13, 0);
    delay(1000);
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, F3);
    u8g2.sendBuffer();

  }
}