/*
기본 debug serial: UART Serial 
<communicate packet form>
0000: 통신 시작
    관절 하나
        0001
        *** ** **** *********
        다리, 관절, 각도(지수부), 각도(가수부)
    전체 관절
        0010
        (**** *********) x 18
    wifi 연결
        0000 0011 : 시작
        * x ? ssid
        0000 0110
        * x ? password
        0000 1100 : 끝
    버튼
        0100
        ******
    IMU
        0101
        어케 하지
    체크섬 (관절 하나, 전체 관절, wifi 연결)
        패킷은 0000 **** 로 시작해 바이트 경계에 맞춰 보내고, 남는 비트는 0으로 채운다.
        마지막 바이트: 시작 코드 바이트부터 패딩까지 모든 바이트의 XOR
모터 제어
    순서 : tibia, femur, hip

*/
#include <Adafruit_PWMServoDriver.h>
#include <functional>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include "BluetoothSerial.h"

using namespace std;

#define CMD_TYPE_JOINT 0
#define CMD_TYPE_JOINTS 1
#define CMD_TYPE_WIFI 2

Adafruit_PWMServoDriver pwmU1(0x41);
Adafruit_PWMServoDriver pwmU2(0x40);

#define SERVO_FREQ 50
#define PWM_I2C_HZ 400000
#define SERVO_START_GAP_MS 100
// ServoCalib 측정값 {최소, 최대}, 인덱스: leg*3 + joint
const uint16_t MOTOR_TICK[18][2] = {};
// MOTOR_TICK의 최소/최대 틱에 해당하는 서보 각도
const uint16_t RC9_MOTORmM_DEG[2] = {};
const uint16_t MG996R_MOTORmM_DEG[2] = {};
const uint16_t OFFSET_DEG[3] = {120, 180, 180};
bool motorStarted[18] = {};
bool calibWarned = false;

const uint8_t legButtonGPIO[6] = {4, 16, 17, 5, 18, 19};

const uint8_t BT_START_CODE = 0x0;
const uint8_t BT_ONE_JOINT_START_CODE = 0x1;
const uint8_t BT_ALL_JOINTS_START_CODE = 0x2;
const uint8_t BT_WIFI_CONNECT_START_CODE = 0x03; 
const uint8_t BT_WIFI_CONNECT_MIDDLE_CODE = 0x6;
const uint8_t BT_WIFI_CONNECT_END_CODE = 0x0C;
const char* bluetoothSsid = "Hexapodv0.2";
String ssid, password;
bool wifiConnecting = false;
unsigned long wifiConnectingDelay = 0;
int connectCount = 0;
unsigned long dataUpdateTime = 0;

WiFiServer wifiServer(3000);
BluetoothSerial SerialBT;
WiFiClient client;

#define DEBUG 1
#define DEBUG_JOINT 0 // 켜면 18관절 명령마다 약 58ms가 걸려 50Hz 제어가 불가능

struct CMD {
    int type;
    String ssid;
    String password;
    int leg;
    int joint;
    float joints[6][3];
    int success;
    String errorMsg;
};

CMD command;


// --- communicate ---
void logMsg(String contents, int logType=0) {
    /*
    type: 0 -> log
          1 -> warning
          2 -> notice
    */
    if (!DEBUG) return;
    if (logType == 0) Serial.print("[log] ");
    else if (logType == 1) Serial.print("[warning] ");
    else if (logType == 2) Serial.print("[notice] ");
    Serial.println(contents);
}

bool readByte(function<char()>& read, function<bool()>& available, int timeout, uint8_t& out) {
    unsigned long curT = millis();
    while (millis() - curT <= timeout) {
        if (available()) {
            out = (uint8_t)read();
            return true;
        }
        if (millis() - curT > 2) delay(1);
    }
    return false;
}

CMD cmdRead(function<char()> read, function<bool()> available, int timeout=70, int Bdeadline=1000) {
    CMD res = {};
    res.type = -1;
    res.success = true;
    int bufS = 0, curS = 0, MbufS = 4, cntLeg=0, cntJoint=0;
    bool ssid_input = true, btStart = false;
    uint32_t buf = 0;
    uint16_t curV = 0;
    uint8_t rawV = 0, checksum = 0;

    while (true) {
        while (bufS<MbufS) {
            if (curS==0) {
                if (!readByte(read, available, timeout, rawV)) {
                    res.errorMsg = "Timeout";
                    res.success = false;
                    return res;
                }
                curV = rawV;
                checksum ^= rawV;
                curS = 8;
            }
            bufS++; buf<<=1;
            buf |= (curV & 0x80)>>7; curS--; 
            curV <<= 1;
            if (Bdeadline-- < 0) { res.success = false; res.errorMsg = "Too long messages"; return res;};
        }
        
        if (res.type < 0) {
            uint8_t typeV = (uint8_t)(buf&0x000F);
            if (!btStart) {
                if (typeV == BT_START_CODE) btStart = true;
            }
            else if (curS != 0) { // 시작 코드는 항상 바이트 경계에 맞춰 온다
                btStart = (typeV == BT_START_CODE);
            }
            else if (BT_ALL_JOINTS_START_CODE == typeV) {
                res.type = 1;
                MbufS = 13;
            }else if (BT_ONE_JOINT_START_CODE == typeV) {
                res.type = 0;
                MbufS = 18;
            }else if (BT_WIFI_CONNECT_START_CODE == typeV) {
                res.type = 2;
                MbufS = 8;
            }else if (typeV != BT_START_CODE) {
                btStart = false;
            }
            if (res.type >= 0) checksum = typeV; // 패킷 앞의 쓰레기 바이트 제외
            bufS = buf = 0;
            continue;
        }
        
        if (res.type == CMD_TYPE_JOINT) {
            res.leg = (buf & 0x38000) >> 15;
            res.joint = (buf & 0x06000) >> 13;
            if (res.leg >= 6 or res.joint >= 3) { res.success = false; res.errorMsg = "Invalid index"; return res; }
            uint32_t e = (buf & 0x1E00) >> 9;
            uint32_t tmp = e ? ((e + 120) << 23) | ((buf & 0x01FF) << 14) : 0;
            memcpy(&res.joints[res.leg][res.joint], &tmp, sizeof(float));
            break;
        }else if (res.type == CMD_TYPE_JOINTS) {
            uint32_t e = (buf & 0x1E00) >> 9;
            uint32_t tmp = e ? ((e + 120) << 23) | ((buf & 0x01FF) << 14) : 0;
            memcpy(&res.joints[cntLeg][cntJoint], &tmp, sizeof(float));
            if (++cntJoint >= 3) {
                if (cntLeg == 5) break;
                cntJoint=0;
                cntLeg++;
            }
            bufS = buf = 0;
        }else if (res.type == CMD_TYPE_WIFI) {
            if (buf == BT_WIFI_CONNECT_END_CODE) {
                break;
            }else if (buf == BT_WIFI_CONNECT_MIDDLE_CODE) {
                ssid_input = false;
            }else if (ssid_input) {
                res.ssid += (char)(uint8_t)buf;
            }
            else {
                res.password += (char)(uint8_t)buf;
            }
            bufS = buf = 0;
        }
    }

    if (!readByte(read, available, timeout, rawV)) {
        res.errorMsg = "Timeout";
        res.success = false;
    }else if (rawV != checksum) {
        res.errorMsg = "Checksum mismatch";
        res.success = false;
    }

    return res;
}

void dataSent(Print& out, const uint8_t *data, size_t len) {
    for (int i=0; i<len; i++) out.write(data[i]);
}

// --- sensor --- 
Adafruit_BNO08x bno(-1);
const uint32_t IMU_REPORT_US = 5000;
const uint8_t IMU_SDA = 32;
const uint8_t IMU_SCL = 33;
const uint32_t IMU_I2C_HZ = 100'000;
const uint8_t  IMU_ADDRS[2] = { 0x4B, 0x4A };

bool imuBegin() {
  Wire1.begin(IMU_SDA, IMU_SCL, IMU_I2C_HZ);
  for (uint8_t a : IMU_ADDRS) {
    Wire1.beginTransmission(a);
    if (Wire1.endTransmission() != 0) continue; 
    if (bno.begin_I2C(a, &Wire1) && bno.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_US)) {
      return true;
    }
  }
  return false;
}

uint8_t buttonRead() {
    uint8_t x = 0;
    for (int i=0; i<6; i++) {
        x <<= 1;
        x |= digitalRead(legButtonGPIO[i]);
    }
    return x;
}

// --- motor ---
void moveJoint(int leg, int joint, float deg) {
    /* 
    관절 각도 -> 서보 각도 -> tick
    tibia, femur, hip (서보 각도 90도 기준)
    120, 180, 180
    */
#if DEBUG_JOINT
    logMsg("leg: "+(String)leg+", joint: "+(String)joint+", deg: "+(String)deg);
#endif
    float tmp = 90 + deg - OFFSET_DEG[joint];
    int motor = leg*3 + joint;
    const uint16_t *mM = MOTOR_TICK[motor], *mMDeg = RC9_MOTORmM_DEG;
    if (joint > 1) mMDeg = MG996R_MOTORmM_DEG;
    if (mM[1] <= mM[0] || mMDeg[1] <= mMDeg[0]) {
        if (!calibWarned) logMsg("Motor not calibrated: "+(String)motor, 1);
        calibWarned = true;
        return;
    }
    float tmp2 = mM[0] + (tmp - mMDeg[0]) * (mM[1] - mM[0]) / (mMDeg[1] - mMDeg[0]);
    uint16_t tick = constrain(tmp2, mM[0], mM[1]);
    if (leg < 5) pwmU1.setPWM(joint+leg*3, 0, tick);
    else pwmU2.setPWM(joint, 0, tick);
    motorStarted[motor] = true;
}

void moveJoints(float joints[][3]) {
    for (int i=0; i<6; i++) for (int j=0; j<3; j++) {
        // 서보는 현재 각도를 알 수 없으므로 첫 명령만 순서대로 켜서 돌입 전류를 분산한다
        bool first = !motorStarted[i*3+j];
        moveJoint(i, j, joints[i][j]);
        if (first && motorStarted[i*3+j]) delay(SERVO_START_GAP_MS);
    }
}


// --- main ---
void setup() {
    Serial.begin(115200);   
    SerialBT.begin(bluetoothSsid);

    for (uint8_t p : legButtonGPIO) {
        pinMode(p, INPUT_PULLUP);
    }
    
    for (auto* p : {&pwmU1, &pwmU2}) {
        p->begin();
        p->setOscillatorFrequency(27000000);
        p->setPWMFreq(SERVO_FREQ);
    }
    Wire.setClock(PWM_I2C_HZ);

    dataUpdateTime = millis();
}

void loop() {
    uint8_t buttonV = buttonRead();

    if (Serial.available() > 0) {
        command = cmdRead([]() { return (char)Serial.read(); }, []() { return Serial.available() > 0; });

        if (command.success) {
            if (command.type == CMD_TYPE_JOINT) moveJoint(command.leg, command.joint, command.joints[command.leg][command.joint]);
            else if (command.type == CMD_TYPE_JOINTS) moveJoints(command.joints);
        }else {
            logMsg("UART error message: "+command.errorMsg, 1);
        }
    }

    if (SerialBT.available() > 0) {
        command = cmdRead([]() { return (char)SerialBT.read(); }, []() { return SerialBT.available() > 0; });

        if (command.success) {
            if (command.type == CMD_TYPE_JOINT) moveJoint(command.leg, command.joint, command.joints[command.leg][command.joint]);
            else if (command.type == CMD_TYPE_JOINTS) moveJoints(command.joints);
            else if (command.type == CMD_TYPE_WIFI) {
                WiFi.disconnect();
                ssid = command.ssid;
                password = command.password;
                WiFi.begin(ssid, password);
                wifiConnecting = true;
                wifiConnectingDelay = millis();
                connectCount=0;
            }
        }else {
            logMsg("Bluetooth error message: "+command.errorMsg, 1);
        }
    }

    if (wifiConnecting) {
        if (WiFi.status() == WL_CONNECTED) {
            wifiConnecting = false;
            logMsg("WiFi Connected, IP Add:"+WiFi.localIP().toString());
            wifiServer.begin();
            return; // 안전을 위해서
        }else if (millis() - wifiConnectingDelay >= 500) {
            wifiConnectingDelay = millis();
            logMsg("WiFi Connecting to ssid:"+ssid);
            connectCount++;
        }else if (connectCount >= 15*2) {
            logMsg("WiFi error message: "+command.errorMsg, 1);
            wifiConnecting = false;
            connectCount=0;
            WiFi.disconnect();
        }
    }

    if (WiFi.status() == WL_CONNECTED && client && client.available() > 0) {
        command = cmdRead([]() { return client.read(); }, []() { return client.available() > 0 and client.connected(); } );
        if (command.success) {
            if (command.type == CMD_TYPE_JOINT) moveJoint(command.leg, command.joint, command.joints[command.leg][command.joint]);
            else if (command.type == CMD_TYPE_JOINTS) moveJoints(command.joints);
        }else {
            logMsg("WiFi error message: "+command.errorMsg, 1);
        }
    }

    if (WiFi.status() == WL_CONNECTED && (!client || !client.connected())) client = wifiServer.accept();

    uint8_t pkt[2] = {0x04, buttonV};

    if (millis()-dataUpdateTime >= 500) {
        if (WiFi.status() == WL_CONNECTED && client) dataSent(client, pkt, 2);
        dataSent(SerialBT, pkt, 2);
        dataSent(Serial, pkt, 2);
        dataUpdateTime = millis();
    }
}
