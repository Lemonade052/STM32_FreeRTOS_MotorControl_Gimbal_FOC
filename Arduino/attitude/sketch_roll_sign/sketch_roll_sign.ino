#include <Wire.h>
#include <math.h>

// 符号检查：只读 AS5600 + MPU，不出力（不输出 PWM，电机不接线也行）
// 目的：确认「横滚电机的 mech 增大方向」和「IMU 的 rollF 增大方向」是不是一致
// 用法：烧录后手转横滚轴 ±20~30°，看 mech 增大时 rollF 是同增还是反减

const float RAD2DEG = 57.29578f;
const float GYRO_LSB = 131.0f;   // ±250°/s

const float PITCH_SIGN      = -1.0f;  // 仰头 = 正（云台上已复核）
const float ROLL_SIGN       = +1.0f;  // 右侧下沉 = 正（云台上已复核）
const float PITCH_RATE_SIGN = -1.0f;  // d(pitch)/dt = -gx
const float ROLL_RATE_SIGN  = +1.0f;  // d(roll)/dt  = +gy

const float ALPHA = 0.98f;

float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
float rollF = 0.0f, pitchF = 0.0f;
bool initialized = false;
unsigned long last_us = 0;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRaw(int16_t a[3], int16_t g[3]) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 14) != 14) return false;

  uint8_t h, l;

  h = Wire.read(); l = Wire.read(); a[0] = (int16_t)((h << 8) | l);
  h = Wire.read(); l = Wire.read(); a[1] = (int16_t)((h << 8) | l);
  h = Wire.read(); l = Wire.read(); a[2] = (int16_t)((h << 8) | l);
  h = Wire.read(); l = Wire.read();                                  // temp
  h = Wire.read(); l = Wire.read(); g[0] = (int16_t)((h << 8) | l);
  h = Wire.read(); l = Wire.read(); g[1] = (int16_t)((h << 8) | l);
  h = Wire.read(); l = Wire.read(); g[2] = (int16_t)((h << 8) | l);
  return true;
}

float readMech() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0) return -1.0f;
  if (Wire.requestFrom(0x36, 2) != 2) return -1.0f;
  int high = Wire.read();
  int low  = Wire.read();
  int raw = ((high & 0x0F) << 8) | low;
  return raw * 360.0f / 4096.0f;
}

void calibrateGyro() {
  const int N = 500;
  long sx = 0, sy = 0, sz = 0;
  int16_t a[3], g[3];

  Serial.println("Keep still, calibrating gyro...");
  for (int i = 0; i < N; i++) {
    if (readRaw(a, g)) {
      sx += g[0];
      sy += g[1];
      sz += g[2];
    }
    delay(2);
  }
  gx_bias = (float)sx / N;
  gy_bias = (float)sy / N;
  gz_bias = (float)sz / N;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(300);

  if (!writeReg(0x6B, 0x00)) { Serial.println("wake: NO ACK"); return; }
  writeReg(0x1B, 0x00);   // 陀螺 ±250°/s
  writeReg(0x1C, 0x00);   // 加速度 ±2g

  calibrateGyro();
  Serial.println("Hand-rotate the ROLL axis +-20~30 deg, watch mech vs rollF");
}

void loop() {
  int16_t a[3], g[3];

  unsigned long now = micros();
  float dt = (now - last_us) * 0.000001f;
  last_us = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  if (!readRaw(a, g)) {
    Serial.println("READ FAIL");
    delay(100);
    return;
  }

  float ax = a[0] / 16384.0f;
  float ay = a[1] / 16384.0f;
  float az = a[2] / 16384.0f;

  float rollA  = ROLL_SIGN  * atan2f(-ax, az) * RAD2DEG;
  float pitchA = PITCH_SIGN * atan2f(ay, sqrtf(ax * ax + az * az)) * RAD2DEG;

  float pitch_rate = PITCH_RATE_SIGN * (g[0] - gx_bias) / GYRO_LSB;
  float roll_rate  = ROLL_RATE_SIGN  * (g[1] - gy_bias) / GYRO_LSB;

  if (!initialized) {
    rollF = rollA;
    pitchF = pitchA;
    initialized = true;
  }

  rollF  = ALPHA * (rollF  + roll_rate  * dt) + (1.0f - ALPHA) * rollA;
  pitchF = ALPHA * (pitchF + pitch_rate * dt) + (1.0f - ALPHA) * pitchA;

  float mech = readMech();

  static unsigned long last_print = 0;
  if (now - last_print >= 100000UL) {
    last_print = now;
    Serial.print("mech=");  Serial.print(mech, 1);
    Serial.print(" rollF="); Serial.print(rollF, 1);
    Serial.print(" rollA="); Serial.print(rollA, 1);
    Serial.print(" pitchF="); Serial.println(pitchF, 1);
  }

  delay(2);
}
