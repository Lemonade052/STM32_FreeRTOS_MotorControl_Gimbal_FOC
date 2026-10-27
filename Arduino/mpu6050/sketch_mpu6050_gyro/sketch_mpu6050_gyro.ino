#include <Wire.h>

// 陀螺零偏标定 + 角速度轴/符号测试
// 用法：上电后保持模块完全静止约 1 秒（自动标定）；然后做动作看符号
// 验证：静止时三轴 ≈ 0.0 °/s；仰头看 gx；右侧下沉看 gy；水平转看 gz

const float GYRO_LSB = 131.0f;   // ±250°/s 量程：131 LSB/(°/s)

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

  Serial.print("bias raw: ");
  Serial.print(gx_bias, 0); Serial.print(", ");
  Serial.print(gy_bias, 0); Serial.print(", ");
  Serial.println(gz_bias, 0);
  Serial.println("Now rotate: pitch=look up, roll=right side down, yaw=turn flat");
}

void loop() {
  int16_t a[3], g[3];

  if (!readRaw(a, g)) {
    Serial.println("READ FAIL");
    delay(200);
    return;
  }

  float gx = (g[0] - gx_bias) / GYRO_LSB;
  float gy = (g[1] - gy_bias) / GYRO_LSB;
  float gz = (g[2] - gz_bias) / GYRO_LSB;

  Serial.print("gx="); Serial.print(gx, 1);
  Serial.print(" gy="); Serial.print(gy, 1);
  Serial.print(" gz="); Serial.println(gz, 1);

  delay(100);
}
