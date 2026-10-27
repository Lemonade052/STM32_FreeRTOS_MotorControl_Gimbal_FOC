#include <Wire.h>
#include <math.h>

// 只用加速度计算 roll / pitch（还没有陀螺积分、没有互补滤波）
// 用法：载板插 AS5600（U4）+ MPU（U5），串口 115200，先开串口再复位
// 坐标定义（与云台一致）：X = 云台 X（右侧，俯仰轴）；Y = +Y（镜头方向，横滚轴）；Z = 向上
// 元件面朝上平放时 az 应为正（≈ +1g）

const float RAD2DEG = 57.29578f;

// 符号约定：仰头 pitch 为正、右侧下沉 roll 为正
// 当前实测：仰头时 ay 为负 → PITCH_SIGN = -1
// 装到云台负载上以后，按最终安装方向再复核一次这两个符号
const float PITCH_SIGN = -1.0f;
const float ROLL_SIGN  = +1.0f;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRaw(int16_t a[3], int16_t g[3]) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                      // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 14) != 14) return false;

  uint8_t h, l;

  h = Wire.read(); l = Wire.read(); a[0] = (int16_t)((h << 8) | l);  // ax
  h = Wire.read(); l = Wire.read(); a[1] = (int16_t)((h << 8) | l);  // ay
  h = Wire.read(); l = Wire.read(); a[2] = (int16_t)((h << 8) | l);  // az
  h = Wire.read(); l = Wire.read();                                  // temp（跳过）
  h = Wire.read(); l = Wire.read(); g[0] = (int16_t)((h << 8) | l);  // gx
  h = Wire.read(); l = Wire.read(); g[1] = (int16_t)((h << 8) | l);  // gy
  h = Wire.read(); l = Wire.read(); g[2] = (int16_t)((h << 8) | l);  // gz
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(300);

  if (writeReg(0x6B, 0x00)) Serial.println("wake: OK");
  else                       Serial.println("wake: NO ACK");

  writeReg(0x1B, 0x00);   // 陀螺 ±250°/s
  writeReg(0x1C, 0x00);   // 加速度 ±2g
  Serial.println("tilt the board: roll/pitch should follow");
}

void loop() {
  int16_t a[3], g[3];

  if (!readRaw(a, g)) {
    Serial.println("READ FAIL");
    delay(200);
    return;
  }

  // ±2g 量程：1g = 16384
  float ax = a[0] / 16384.0f;
  float ay = a[1] / 16384.0f;
  float az = a[2] / 16384.0f;

  // 本机是 X 右 / Y 前（不是教科书默认的 X 前 / Y 右），所以 roll 与 pitch 换位
  // roll：绕 Y（镜头轴），右侧下沉为正
  // pitch：绕 X（俯仰轴），仰头为正（当前实物 ay 符号相反，乘 -1）
  float roll  = ROLL_SIGN  * atan2f(-ax, az) * RAD2DEG;
  float pitch = PITCH_SIGN * atan2f(ay, sqrtf(ax * ax + az * az)) * RAD2DEG;

  Serial.print("ax="); Serial.print(ax, 2);
  Serial.print(" ay="); Serial.print(ay, 2);
  Serial.print(" az="); Serial.print(az, 2);
  Serial.print(" | roll="); Serial.print(roll, 1);
  Serial.print("  pitch="); Serial.println(pitch, 1);

  delay(100);
}
