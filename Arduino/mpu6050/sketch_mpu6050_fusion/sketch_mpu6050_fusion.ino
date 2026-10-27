#include <Wire.h>
#include <math.h>

// 加速度角 + 陀螺角速度 联合测试
// 目的：用加速度角的变化方向，去对齐陀螺的符号（不需要人工标注动作）
// 用法：上电保持静止约 1 秒；然后慢慢做俯仰、横滚，观察同一时刻 pitchA/rollA 与 gx/gy

const float RAD2DEG = 57.29578f;
const float GYRO_LSB = 131.0f;   // ±250°/s

const float PITCH_SIGN = -1.0f;  // 仰头 = 正
const float ROLL_SIGN  = +1.0f;  // 右侧下沉 = 正

float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;

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

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(300);

  if (!writeReg(0x6B, 0x00)) { Serial.println("wake: NO ACK"); return; }
  writeReg(0x1B, 0x00);   // 陀螺 ±250°/s
  writeReg(0x1C, 0x00);   // 加速度 ±2g

  Serial.println("Keep still, calibrating gyro...");

  const int N = 500;
  long sx = 0, sy = 0, sz = 0;
  int16_t a[3], g[3];

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

  Serial.println("Now move slowly: pitch (front up/down), then roll (side down/up)");
}

void loop() {
  int16_t a[3], g[3];

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

  float gx = (g[0] - gx_bias) / GYRO_LSB;
  float gy = (g[1] - gy_bias) / GYRO_LSB;
  float gz = (g[2] - gz_bias) / GYRO_LSB;

  Serial.print("rollA=");  Serial.print(rollA, 1);
  Serial.print(" pitchA="); Serial.print(pitchA, 1);
  Serial.print(" | gx=");   Serial.print(gx, 1);
  Serial.print(" gy=");     Serial.print(gy, 1);
  Serial.print(" gz=");     Serial.println(gz, 1);

  delay(50);
}
