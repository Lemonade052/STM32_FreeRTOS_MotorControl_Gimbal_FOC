#include <Wire.h>

// AS5600 单独读角度测试：只读 I2C，不驱动电机、不输出 PWM
// 用法：载板只插 AS5600（U4），USB 供电，串口 115200
// 期望：setup 打印 found 0x36；loop 打印 mech，手转电机轴角度平滑变化

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

float readMech() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0) return -1.0f;    // 芯片没 ACK
  if (Wire.requestFrom(0x36, 2) != 2) return -1.0f; // 没收到 2 字节
  int high = Wire.read();
  int low  = Wire.read();
  int raw = ((high & 0x0F) << 8) | low;
  return raw * 360.0f / 4096.0f;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);   // 100kHz，抗干扰更强
  delay(200);
  scanI2C();
}

void loop() {
  float mech = readMech();
  if (mech < 0.0f) {
    Serial.println("READ FAIL");
  } else {
    Serial.print("mech=");
    Serial.println(mech, 2);
  }
  delay(200);
}
