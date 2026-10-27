#include <Wire.h>

// MPU6050 / MPU6500 原始数据测试
// 用法：载板插 AS5600（U4）+ MPU（U5），USB 供电，串口 115200
// 先打开串口，再按板子复位，才能看到 setup 里的扫描结果
// 期望（平放静止）：az ≈ 16384，ax/ay ≈ 0，gx/gy/gz 只有小噪声

void scanI2C() {
  Serial.println("I2C scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
    }
  }
}

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRaw(int16_t a[3], int16_t g[3], int16_t* temp) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                      // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom(0x68, 14) != 14) return false;

  // 高字节在前
  uint8_t h, l;

  h = Wire.read(); l = Wire.read(); a[0] = (int16_t)((h << 8) | l);  // ax
  h = Wire.read(); l = Wire.read(); a[1] = (int16_t)((h << 8) | l);  // ay
  h = Wire.read(); l = Wire.read(); a[2] = (int16_t)((h << 8) | l);  // az
  h = Wire.read(); l = Wire.read(); *temp = (int16_t)((h << 8) | l); // temp
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

  scanI2C();

  // 唤醒：PWR_MGMT_1 = 0
  if (writeReg(0x6B, 0x00)) Serial.println("wake: OK");
  else                       Serial.println("wake: NO ACK");

  // 量程显式设成默认：陀螺 ±250°/s，加速度 ±2g
  writeReg(0x1B, 0x00);
  writeReg(0x1C, 0x00);

  // WHO_AM_I
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission(false);
  if (Wire.requestFrom(0x68, 1) == 1) {
    uint8_t who = Wire.read();
    Serial.print("WHO_AM_I = 0x");
    if (who < 16) Serial.print('0');
    Serial.println(who, HEX);
  } else {
    Serial.println("WHO_AM_I read FAIL");
  }
}

void loop() {
  int16_t a[3], g[3], t;
  if (!readRaw(a, g, &t)) {
    Serial.println("READ FAIL");
  } else {
    Serial.print("ax="); Serial.print(a[0]);
    Serial.print(" ay="); Serial.print(a[1]);
    Serial.print(" az="); Serial.print(a[2]);
    Serial.print(" | gx="); Serial.print(g[0]);
    Serial.print(" gy="); Serial.print(g[1]);
    Serial.print(" gz="); Serial.print(g[2]);
    Serial.print(" | temp="); Serial.println(t);
  }
  delay(200);
}
