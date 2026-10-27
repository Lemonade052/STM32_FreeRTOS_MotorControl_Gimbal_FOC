#include <Wire.h>
#include <math.h>

// 互补滤波：加速度计管长期，陀螺仪管短期
// 输出 rollF / pitchF 为融合角；rollA / pitchA 为纯加速度角（对照用）
// 用法：上电静止约 1 秒标定陀螺零偏；然后慢慢倾斜、快速晃动观察

const float RAD2DEG = 57.29578f;
const float GYRO_LSB = 131.0f;   // ±250°/s

const float PITCH_SIGN      = -1.0f;  // 仰头 = 正
const float ROLL_SIGN       = +1.0f;  // 右侧下沉 = 正
const float PITCH_RATE_SIGN = -1.0f;  // d(pitch)/dt = -gx
const float ROLL_RATE_SIGN  = +1.0f;  // d(roll)/dt  = +gy

const float ALPHA = 0.98f;       // 陀螺权重；越大越信陀螺

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
  Serial.println("Now tilt slowly, then shake, then let it settle");
}

void loop() {
  int16_t a[3], g[3];

  unsigned long now = micros();
  float dt = (now - last_us) * 0.000001f;
  last_us = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;   // 首次循环/异常保护

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

  // 互补滤波：陀螺积分 + 加速度慢慢拉回
  rollF  = ALPHA * (rollF  + roll_rate  * dt) + (1.0f - ALPHA) * rollA;
  pitchF = ALPHA * (pitchF + pitch_rate * dt) + (1.0f - ALPHA) * pitchA;

  // 20Hz 打印，避免串口拖慢循环
  static int cnt = 0;
  if (++cnt >= 5) {
    cnt = 0;
    Serial.print("rollA=");  Serial.print(rollA, 1);
    Serial.print(" pitchA="); Serial.print(pitchA, 1);
    Serial.print(" | rollF="); Serial.print(rollF, 1);
    Serial.print(" pitchF="); Serial.print(pitchF, 1);
    Serial.print(" | dt="); Serial.println(dt * 1000.0f, 1);
  }

  delay(5);
}
