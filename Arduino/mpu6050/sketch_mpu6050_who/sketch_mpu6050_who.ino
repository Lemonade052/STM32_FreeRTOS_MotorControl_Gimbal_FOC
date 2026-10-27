#include <Wire.h>

// MPU6050 上电检测：I2C 扫描 + WHO_AM_I
// 用法：载板插 AS5600（U4）+ MPU6050（U5），USB 供电，串口 115200
// 期望：扫描到 0x36 和 0x68；WHO_AM_I = 0x68

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

uint8_t readWhoAmI() {
  Wire.beginTransmission(0x68);
  Wire.write(0x75);                 // WHO_AM_I 寄存器
  if (Wire.endTransmission(false) != 0) return 0xFF;  // repeated start
  if (Wire.requestFrom(0x68, 1) != 1) return 0xFF;
  return Wire.read();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(200);

  scanI2C();

  // 唤醒 MPU6050：PWR_MGMT_1(0x6B) = 0x00
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 wake: NO ACK");
  } else {
    Serial.println("MPU6050 wake: OK");
  }

  uint8_t who = readWhoAmI();
  Serial.print("WHO_AM_I = 0x");
  if (who < 16) Serial.print('0');
  Serial.println(who, HEX);
}

void loop() {
  // 每秒复读一次，方便热插拔/复位后观察
  uint8_t who = readWhoAmI();
  Serial.print("WHO_AM_I = 0x");
  if (who < 16) Serial.print('0');
  Serial.println(who, HEX);
  delay(1000);
}
