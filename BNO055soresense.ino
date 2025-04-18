#include <Arduino.h>
#include <HardwareSerial.h>

// -----------------------------------------------------------------------------
//  BNO055 UART demo for Raspberry Pi Pico W – 6軸IMUモード (磁気センサ除外)
//  • UART0 GP0(TX) / GP1(RX) 115200 bps (Serial2)
//  • CONFIG → IMU (6軸) の最小初期化 + 外部クリスタル + 単位設定
//  • 拡張タイムアウト、リトライ付き書き込み (bnoWrite8) + 簡易読み取り (bnoRead)
// -----------------------------------------------------------------------------

// -------- UART protocol constants --------
const uint8_t BNO_START         = 0xAA;
const uint8_t BNO_WRITE         = 0x00;
const uint8_t BNO_READ          = 0x01;
const uint8_t BNO_RESP_OK_READ  = 0xBB;
const uint8_t BNO_RESP_OK_WRITE = 0xEE;

// -------- Registers & modes -------------
const uint8_t REG_OPR_MODE      = 0x3D;
const uint8_t REG_UNIT_SEL      = 0x3B;
const uint8_t REG_SYS_TRIGGER   = 0x3F;
const uint8_t REG_EULER_H_LSB   = 0x1A;

const uint8_t MODE_CONFIG       = 0x00;
const uint8_t MODE_IMU          = 0x08;  // 6軸 (加速度 + ジャイロ)

// -------- Timeout & retry parameters ---
const uint32_t BYTE_TIMEOUT_MS  = 200;  // 1バイトあたり最大待ち時間 (ms)
const int      WRITE_RETRIES     = 3;    // 書き込みリトライ回数

// -------- Helpers -----------------------
static bool waitBytes(HardwareSerial &s, int n, uint32_t timeout = BYTE_TIMEOUT_MS) {
  uint32_t start = millis();
  while (s.available() < n) {
    if (millis() - start > timeout) return false;
  }
  return true;
}

static bool bnoWrite8(uint8_t reg, uint8_t value) {
  for (int attempt = 0; attempt < WRITE_RETRIES; ++attempt) {
    while (Serial2.available()) Serial2.read();
    uint8_t frame[5] = { BNO_START, BNO_WRITE, reg, 1, value };
    Serial2.write(frame, 5);
    if (!waitBytes(Serial2, 2)) continue;
    uint8_t resp[2];
    Serial2.readBytes(resp, 2);
    if (resp[0] == BNO_RESP_OK_WRITE && resp[1] == 0x01) return true;
  }
  return false;
}

static bool bnoRead(uint8_t reg, uint8_t len, uint8_t* buf) {
  for (int retry = 0; retry < 2; ++retry) {
    while (Serial2.available()) Serial2.read();
    uint8_t cmd[4] = { BNO_START, BNO_READ, reg, len };
    Serial2.write(cmd, 4);
    if (!waitBytes(Serial2, 2)) continue;
    uint8_t hdr[2];
    Serial2.readBytes(hdr, 2);
    if (hdr[0] != BNO_RESP_OK_READ || hdr[1] != len) continue;
    if (!waitBytes(Serial2, len)) continue;
    if (Serial2.readBytes(buf, len) == len) return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(650);

  // CONFIGモード
  if (!bnoWrite8(REG_OPR_MODE, MODE_CONFIG)) {
    Serial.print("CONFIG fail\n\r");
  }
  delay(30);

  // 外部クリスタル有効化
  if (!bnoWrite8(REG_SYS_TRIGGER, 0x80)) {
    Serial.print("XT fail\n\r");
  } else {
    Serial.print("XT success\n\r");
  }
  delay(500);

  // 単位系: 度/秒, 度
  if (!bnoWrite8(REG_UNIT_SEL, 0x04)) {
    Serial.print("UNIT fail\n\r");
  } else {
    Serial.print("UNIT success\n\r");
  } 
  delay(10);

  // IMUモード (6軸)
  if (!bnoWrite8(REG_OPR_MODE, MODE_IMU)) {
    Serial.print("IMU mode fail\n\r");
  } else {
    Serial.print("IMU mode success\n\r");
  } 
  delay(600);

  Serial.print("Initialization complete (6-axis IMU)\n\r");
}

void loop() {
  uint8_t raw[6];
  if (bnoRead(REG_EULER_H_LSB, 6, raw)) {
    int16_t h = (int16_t)(raw[0] | (raw[1] << 8));
    int16_t r = (int16_t)(raw[2] | (raw[3] << 8));
    int16_t p = (int16_t)(raw[4] | (raw[5] << 8));
    float Hf = h / 16.0f;
    float Rf = r / 16.0f;
    float Pf = p / 16.0f;
    char buf[64];
    sprintf(buf, "H:%.4f R:%.4f P:%.4f\n\r", Hf, Rf, Pf);
    Serial.print(buf);
  } else {
    Serial.print("read timeout\n\r");
  }
  delay(10);
}
