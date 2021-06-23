/*
 * Generated using BlocklyDuino:
 *
 * https://github.com/MediaTek-Labs/BlocklyDuino-for-LinkIt
 *
 * Date: Wed, 23 Jun 2021 09:00:08 GMT
 */

/*

 * 部份程式碼由吉哥積木產生
 * https://sites.google.com/jes.mlc.edu.tw/ljj/linkit7697

*/

#include <SoftwareSerial.h>

#define PXT_PACKET_START   0xFD
#define PXT_PACKET_END    0xFE

#define PXT_CMD_STREAMON  0x79
#define PXT_CMD_STREAMOFF 0x7A
#define PXT_CMD_ENABLEFUNC  0x7D
#define PXT_CMD_DETMODE   0x7E

#define PXT_RET_CAM_SUCCESS 0xE0
#define PXT_RET_CAM_ERROR 0xE1

#define PXT_RET_OBJNUM    0x46

#define PXT_BUF_SIZE    40

#define MAX_OPENCAM_ERROR   2
#define MAX_HEX_ERROR     30

enum EFunc {
  FUNC_COLOR_DETECTION		= 1,
  FUNC_COLOR_CODE_DETECTION	= 2,
  FUNC_SHAPE_DETECTION		= 3,
  FUNC_SPHERE_DETECTION		= 4,
  FUNC_TEMPLATE_MATCHING		= 6,
  FUNC_KEYPOINTS				= 8,
  FUNC_NEURAL_NETWORK			= 9,
  FUNC_APRILTAG				= 10,
  FUNC_FACE_DETECTION			= 11,
  FUNC_TRAFFIC_SIGN_DETECTION	= 12,
  FUNC_HANDWRITTEN_DIGITS_DETECTION	= 13,
  FUNC_HANDWRITTEN_LETTERS_DETECTION	= 14,
  FUNC_CLOUD_DETECTION		= 15,
  FUNC_LANES_DETECTION		= 16,
  FUNC_EQUATION_DETECTION		= 17,
  FUNC_SIMPLE_CLASSIFIER		= 18,
  FUNC_VOICE_COMMAND			= 19
};
enum EColor {
  COLOR_RED = 1,
  COLOR_YELLOW,
  COLOR_GREEN,
  COLOR_BLUE,
  COLOR_PURPLE,  COLOR_BLACK
};
enum EShape {
  SHAPE_ROUND = 1,
  SHAPE_RECTANGLE,
  SHAPE_TRIANGLE,
  SHAPE_PENTAGON
};
enum ETrafficSign {
  SIGN_NO_ENTRE = 0,
  SIGN_NO_LEFT_TURN,
  SIGN_NO_RIGHT_TURN,
  SIGN_WRONG_WAY,
  SIGN_NO_U_TURN,
  SIGN_MAX_SPEED,  SIGN_ONEWAY_TRAFFIC,
  SIGN_LEFT_TURN,
  SIGN_RIGHT_TURN,
  SIGN_MIN_SPEED,
  SIGN_U_TURN,
  SIGN_TUNNEL_AHEAD,
  SIGN_BEWARE_OF_CHILDREN,
  SIGN_ROUNDABOUT,
  SIGN_YIELD_TO_PEDESTRIAN,
  SIGN_RED_LIGHT,
  SIGN_GREEN_LIGHT
};
enum ELetters {
  LETTER_A=0,
  LETTER_B,
  LETTER_C,
  LETTER_D,
  LETTER_E,
  LETTER_F,
  LETTER_G,
  LETTER_H,
  LETTER_I,
  LETTER_J,
  LETTER_K,
  LETTER_L,
  LETTER_M,
  LETTER_N,
  LETTER_O,
  LETTER_P,
  LETTER_Q,
  LETTER_R,
  LETTER_S,
  LETTER_T,
  LETTER_U,
  LETTER_V,
  LETTER_W,
  LETTER_X,
  LETTER_Y,
  LETTER_Z
};
enum EVoiceCommand {
  VOICE_Hello = 1,
  VOICE_Thanks,
  VOICE_Bye,
  VOICE_WhatsThis,
  VOICE_WhatTime,
  VOICE_HowOld,
  VOICE_WhatDay,
  VOICE_TellStory,
  VOICE_TellJoke,
  VOICE_ReadPoem,
  VOICE_TurnOnLight,
  VOICE_TurnOffLight,
  VOICE_TurnLeft,
  VOICE_TurnRight,
  VOICE_GoAhead,
  VOICE_MoveBack,
  VOICE_Stop,
  VOICE_Open,
  VOICE_Close,
  VOICE_OpenEyes1,
  VOICE_OpenEyes2,
  VOICE_CloseEyes1,
  VOICE_CloseEyes2,
  VOICE_Jump,
  VOICE_StandUp,
  VOICE_SquatDown
};
enum EApriltagField {
  APRILTAG_POS_X=1,
  APRILTAG_POS_Y,
  APRILTAG_POS_Z,
  APRILTAG_ROT_X,
  APRILTAG_ROT_Y,
  APRILTAG_ROT_Z,
  APRILTAG_CENTER_X,
  APRILTAG_CENTER_Y
};
enum ELanesField {
    LANES_LX1=1,
    LANES_LY1,
    LANES_LX2,
    LANES_LY2,
    LANES_RX1,
    LANES_RY1,
    LANES_RX2,
    LANES_RY2
};
bool bEnableUVC = false;
bool bDetMode = false;
bool isCamOpened;
bool bSendStreamOn;
bool hasDelayed;
int  nOpenCamFailCount;
int  nHexErrCount;
bool m_bDetModeDone;
bool m_bFuncDone = true;
int  m_nFuncID = 0;
int m_id = 0;
int m_type = 0;
int m_x = 0;
int m_y = 0;
int m_h = 0;
int m_w = 0;
int m_objnum;
float m_eqAnswer;
int m_eqLen = 0;
int m_dataLen = 0;
unsigned long nTime4ObjNum = 0;
float m_posx;
float m_posy;
float m_posz;
int m_rotx;
int m_roty;
int m_rotz;
int m_centerx;
int m_centery;
uint8_t m_inbuf[PXT_BUF_SIZE];
int m_points[8];
char m_eqExpr[17];

int MaxSpeed;

int Kp;

int error;

int LeftMotor;

int RightMotor;

SoftwareSerial myPixettoSerial(2, 3);

void begin() {
  myPixettoSerial.begin(38400);
  myPixettoSerial.setTimeout(50);
  hasDelayed = false;
  isCamOpened = false;
  bSendStreamOn = false;
  nOpenCamFailCount = 0;
  nHexErrCount = 0;
}
void end() {
  myPixettoSerial.end();
}
void clearDetectedData() {
  m_id = 0; m_type = 0;
  m_x = 0; m_y = 0; m_h = 0; m_w = 0;
  m_eqAnswer = 0; m_eqLen = 0;
  m_posx = 0.0; m_posy = 0.0; m_posz = 0.0; m_rotx = 0; m_roty = 0; m_rotz = 0;
  m_centerx = 0; m_centery = 0;
  memset(m_inbuf,  0, sizeof(m_inbuf));
  memset(m_points, 0, sizeof(m_points));
  memset(m_eqExpr, 0, sizeof(m_eqExpr));
}
int getFuncID() {
	return m_id;
}
int getTypeID() {
	return m_type;
}
int getPosX() {
	return m_x;
}
int getPosY() {
	return m_y;
}
int getH() {
	return m_h;
}
int getW() {
	return m_w;
}
int numObjects() {
	if ((millis() - nTime4ObjNum) > 500)
		m_objnum = 0;
	return m_objnum;
}
void getEquationExpr(char *buf, int len) {
	 strncpy(buf, m_eqExpr, m_eqLen);
	 buf[m_eqLen] = '\0';
}

float getEquationAnswer() {
	return m_eqAnswer;
}
void getApriltagInfo(float* px, float* py, float* pz, int* rx, int* ry, int* rz, int* cx, int* cy) {
	if (!px || !py || !px || !rx || !ry || !rz || !cx || !cy)
		return;
	*px = m_posx;
	*py = m_posy;
	*pz = m_posz;
	*rx = m_rotx;
	*ry = m_roty;
	*rz = m_rotz;
	*cx = m_centerx;
	*cy = m_centery;
}
float getApriltagField(EApriltagField field) {
	switch(field) {
		case APRILTAG_POS_X:
			return m_posx;
		case APRILTAG_POS_Y:
			return m_posy;
		case APRILTAG_POS_Z:
			return m_posz;
		case APRILTAG_ROT_X:
			return m_rotx;
		case APRILTAG_ROT_Y:
			return m_roty;
		case APRILTAG_ROT_Z:
			return m_rotz;
		case APRILTAG_CENTER_X:
			return m_centerx;
		case APRILTAG_CENTER_Y:
			return m_centery;
		default:
			return 0;
	}
}
void getLanePoints(int* lx1, int* ly1, int* lx2, int* ly2, int* rx1, int* ry1, int* rx2, int* ry2) {
  if (!lx1 || !ly1 || !lx2 || !ly2 || !rx1 || !ry1 || !rx2 || !ry2)
    return;
  *lx1 = m_points[0];
  *ly1 = m_points[1];
  *lx2 = m_points[2];
  *ly2 = m_points[3];
  *rx1 = m_points[4];
  *ry1 = m_points[5];
  *rx2 = m_points[6];
  *ry2 = m_points[7];
}
float getLanesField(ELanesField field) {
	switch(field) {
		case LANES_LX1:
			return m_points[0];
		case LANES_LY1:
			return m_points[1];
		case LANES_LX2:
			return m_points[2];
		case LANES_LY2:
			return m_points[3];
		case LANES_RX1:
			return m_points[4];
		case LANES_RY1:
			return m_points[5];
		case LANES_RX2:
			return m_points[6];
		case LANES_RY2:
			return m_points[7];
		default:
			return 0;
	}
}
void flush() {
  while (myPixettoSerial.available() > 0)
    char t = myPixettoSerial.read();
}
void calcDataChecksum(uint8_t *buf, int len) {
  uint8_t sum = 0;
  for (int i=1; i<len-2; i++)
    sum += buf[i];
  sum %= 256;
  buf[len-2] = sum;
}
void sendQueryCommand() {
  uint8_t SENSOR_CMD[] = {PXT_PACKET_START, 0x06, PXT_CMD_ENABLEFUNC, m_nFuncID, 0, PXT_PACKET_END};
  calcDataChecksum(SENSOR_CMD, 6);
  myPixettoSerial.write(SENSOR_CMD, sizeof(SENSOR_CMD)/sizeof(uint8_t));
}
void sendDetModeCommand() {
  uint8_t SENSOR_CMD[] =  {PXT_PACKET_START, 0x06, PXT_CMD_DETMODE, bDetMode?1:0, 0, PXT_PACKET_END};
  calcDataChecksum(SENSOR_CMD, 6);
  myPixettoSerial.write(SENSOR_CMD, sizeof(SENSOR_CMD)/sizeof(uint8_t));
}

void resetUboot() {
  Serial.println("resetUboot");
  end();
  delay(50);
  myPixettoSerial.begin(115200);
  myPixettoSerial.print("reset\n");
  myPixettoSerial.print("reset\n");
  myPixettoSerial.print("reset\n");
  myPixettoSerial.flush();
  delay(50);
  flush();
  myPixettoSerial.end();
  delay(2000);
  begin();
}
bool openCam() {
  if (isCamOpened)
    return true;
  if (!hasDelayed) {
    delay(3000);
    hasDelayed = true;
  }
  else {
    delay(1000);
  }
  if (nOpenCamFailCount > MAX_OPENCAM_ERROR) {
    resetUboot();
    bSendStreamOn = false;
    nOpenCamFailCount = 0;
    delay(2000);
  }
  if (!bSendStreamOn) {
    flush();
    uint8_t SENSOR_CMD[] =  {PXT_PACKET_START, 0x05, PXT_CMD_STREAMOFF, 0, PXT_PACKET_END};
    calcDataChecksum(SENSOR_CMD, 5);
    myPixettoSerial.write(SENSOR_CMD, sizeof(SENSOR_CMD)/sizeof(uint8_t));
    delay(500);
    flush();
    SENSOR_CMD[2] = PXT_CMD_STREAMON;
    calcDataChecksum(SENSOR_CMD, 5);
    myPixettoSerial.write(SENSOR_CMD, sizeof(SENSOR_CMD)/sizeof(uint8_t));
    bSendStreamOn = true;
  }  if (myPixettoSerial.available() > 0) {
    bSendStreamOn = false;
    uint8_t buffer[10];
    uint8_t input;
    int i=0;
    int nodata=0;
    while ((input = myPixettoSerial.read()) != PXT_PACKET_START) {
      if (input == 0xFF) {
        nOpenCamFailCount++;
        return false;
      }
      continue;
    }
    buffer[i++] = input;
    while ((input = myPixettoSerial.read()) != PXT_PACKET_END) {
      if (input == 0xFF) {
        delay(1);
        nodata++;
        if (nodata > 10) {
          nOpenCamFailCount++;
          return false;
        }
        else
          continue;
      }
      if (input == PXT_PACKET_START)
        i = 0;
      if (i >= 4) {
        nOpenCamFailCount++;
        return false;
      }
      buffer[i++] = input;
      nodata = 0;
    }
    buffer[i] = input;
    if (buffer[2] == PXT_RET_CAM_SUCCESS) {
      isCamOpened = true;
      if (!m_bDetModeDone) {
        sendDetModeCommand();
        m_bDetModeDone = true;
      }
      return true;
    }
    else {
      nOpenCamFailCount++;
    }
  }
  else {
    nOpenCamFailCount++;
  }
  return false;
}

bool verifyDataChecksum(uint8_t *buf, int len) {
  uint8_t sum = 0;
  for (uint8_t i=1; i<len-2; i++)
    sum += buf[i];
  sum %= 256;
  return (sum == buf[len-2]);
}
void parse_Lanes(uint8_t *buf) {
  m_x = buf[3];
  m_y = buf[4];
  for (int aa=0; aa<8; aa++)
    m_points[aa] = buf[aa+5];
}
void parse_Equation(uint8_t *buf, int len) {
  m_x = buf[3];
  m_y = buf[4];
  m_w = buf[5];
  m_h = buf[6];
  m_eqAnswer = 0;
  for (int i=8; i<=14; i++)
    m_eqAnswer = m_eqAnswer * 10 + buf[i];
  m_eqAnswer /= 100;
  if (buf[7] == 0) m_eqAnswer = 0 - m_eqAnswer;
  memset(m_eqExpr, 0, sizeof(m_eqExpr));
  m_eqLen = len - 17;
  for (int aa=0; aa<m_eqLen; aa++)
    m_eqExpr[aa] = (char)buf[aa+15];
}
void parse_Apriltag(uint8_t *buf) {
  m_type = buf[3];
  m_x = buf[4];
  m_y = buf[5];
  m_w = buf[6];
  m_h = buf[7];
  int value = 0;
  value = (short)(buf[8] * 256 + buf[9]);
  m_posx = (float)value / 100.0;
  value = (short)(buf[10] * 256 + buf[11]);
  m_posy = (float)value / 100.0;
  value = (short)(buf[12] * 256 + buf[13]);
  m_posz = (float)value / 100.0;
  m_rotx = (short)(buf[14] * 256 + buf[15]);
  m_roty = (short)(buf[16] * 256 + buf[17]);
  m_rotz = (short)(buf[18] * 256 + buf[19]);
  m_centerx = (short)(buf[20] * 256 + buf[21]);
  m_centery = (short)(buf[22] * 256 + buf[23]);
}
void parse_SimpleClassifier(uint8_t *buf) {
  m_type = buf[3] * 256 + buf[4];
  m_x = buf[5];
  m_y = buf[6];
  m_w = buf[7];
  m_h = buf[8];
}

bool isDetected() {
  if (bEnableUVC) {
    sendDetModeCommand();
  }
  else {
    bool ret = openCam();
    if (!ret) {
      return false;
    }
  }
  if (!bDetMode && !m_bFuncDone) {
    flush();
    sendQueryCommand();
    m_bFuncDone = true;
  }
  if (bDetMode == true) {
    flush();
    sendQueryCommand();
  }
  clearDetectedData();
  if (readFromSerial()) {
    m_id = m_inbuf[2];
    if (m_id <= 0) {
      return false;
    }
    if (m_id == FUNC_LANES_DETECTION) {
			parse_Lanes(m_inbuf);
			m_objnum = 1;
		}
		else if (m_id == FUNC_EQUATION_DETECTION) {
		 	parse_Equation(m_inbuf, m_dataLen);
		 	m_objnum = 1;
		}
		else if (m_id == FUNC_APRILTAG) {
			parse_Apriltag(m_inbuf);
		}
		else if (m_id == FUNC_SIMPLE_CLASSIFIER) {
			parse_SimpleClassifier(m_inbuf);
		}
		else {
			m_type = m_inbuf[3];
			if (m_id == PXT_RET_OBJNUM) {
				if (m_type > 0) {
				    m_objnum  = m_type;
				    nTime4ObjNum = millis();
				}
				return isDetected();
			}
      m_x = m_inbuf[4];
      m_y = m_inbuf[5];
      m_w = m_inbuf[6];
      m_h = m_inbuf[7];
    }
    return true;
  }
  else {
    if (nHexErrCount > MAX_HEX_ERROR) {
      nHexErrCount = 0;
      resetUboot();
    }
    return false;
  }
  return false;
}

bool readFromSerial() {
  uint8_t tmpbuf[PXT_BUF_SIZE];
  int readnum = 0;
  if (bDetMode == true) {
    int loop=0;
    while (myPixettoSerial.available() <= 0 && loop < 100000) loop++;
  }
  delay(150);
  if (myPixettoSerial.available() > 0) {
    memset(tmpbuf, 0 ,sizeof(tmpbuf));
    if ((readnum = myPixettoSerial.readBytes(tmpbuf, PXT_BUF_SIZE)) != 0) {
    }
  }
  if (readnum == 0)
    return false;
  int i = 0;
  while (i < readnum) {
    if (tmpbuf[i] != PXT_PACKET_START) {
      i++;
      continue;
    }
    if (i == readnum - 1) {
      nHexErrCount++;
      return false;
    }
    int len = tmpbuf[i+1];
    if (len < 0 || len > PXT_BUF_SIZE || len > readnum - i) {
      nHexErrCount++;
      return false;
    }
    memset(m_inbuf, 0, sizeof(m_inbuf));
    memcpy(m_inbuf, tmpbuf+i, len);
    if (verifyDataChecksum(m_inbuf, len)) {
      nHexErrCount = 0;
      m_dataLen = len;
      if (m_inbuf[2] == PXT_RET_OBJNUM) {
        if (m_inbuf[3] > 0) {
            m_objnum  = m_inbuf[3];
            nTime4ObjNum = millis();
        }
        i += m_dataLen;
        continue;
      }
      return true;
    }
    else {
      memset(m_inbuf, 0, sizeof(m_inbuf));
      i++;
    }
  }
    nHexErrCount++;
  return false;
}

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
  begin();

  MaxSpeed = 70;
  Kp = MaxSpeed / 50;
  pinMode(17, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(13, OUTPUT);

}


void loop()
{
  if (isDetected()) {
    error = getPosX() - 50;
    LeftMotor = (constrain(MaxSpeed + error * Kp, 0,MaxSpeed));
    RightMotor = (constrain(MaxSpeed - error * Kp, 0,MaxSpeed));
    motor_LR(LeftMotor, RightMotor);

  }
}