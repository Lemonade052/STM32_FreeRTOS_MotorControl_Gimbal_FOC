#include <Wire.h>
#include <math.h>

// Vq 对齐（载板版，24V 供电）
// 固定电角度 elec=0 施加 Vq=1.5V，转子被拉到固定位置；
// 读机械角 M，OFFSET = M - 90/P（P=7）。
// 和旧 offset 321.30 比：差值若是 360/7=51.43° 的整数倍 = 等价（对齐落在了另一个同电角度的齿位）

const int IN1 = PA0, IN2 = PA1, IN3 = PA2;
const float UDC = 24.0f;          // 24V 适配器
const float P   = 7.0f;           // 极对数
const float VQ_ALIGN = 1.5f;      // 对齐电压：够拉起就行，别给大（防堵转发热）
const float OLD_OFFSET = 321.30f;

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

// 按固定电角度 elec 施加 Vq（反 Park → 反 Clark → SVPWM）
void applyVoltage(float Vq, float elec) {
  float valpha = -Vq * sinf(elec);
  float vbeta  =  Vq * cosf(elec);

  float ua = valpha;
  float ub = -0.5f * valpha + 0.8660254f * vbeta;
  float uc = -0.5f * valpha - 0.8660254f * vbeta;

  float mid = 0.5f * (fminf(ua, fminf(ub, uc)) + fmaxf(ua, fmaxf(ub, uc)));
  float da = (ua - mid) / UDC + 0.5f;
  float db = (ub - mid) / UDC + 0.5f;
  float dc = (uc - mid) / UDC + 0.5f;

  da = constrain(da, 0.0f, 1.0f);
  db = constrain(db, 0.0f, 1.0f);
  dc = constrain(dc, 0.0f, 1.0f);

  analogWrite(IN1, (int)(da * 255.0f));
  analogWrite(IN2, (int)(db * 255.0f));
  analogWrite(IN3, (int)(dc * 255.0f));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT);
  analogWriteFrequency(20000);
  delay(300);

  Serial.println("Vq align: elec=0, Vq=1.5V, hold 2s...");

  applyVoltage(VQ_ALIGN, 0.0f);
  delay(2000);

  // 静止后读 10 次取平均
  float M = 0.0f;
  int n = 0;
  for (int i = 0; i < 10; i++) {
    float m = readMech();
    if (m >= 0.0f) {
      M += m;
      n++;
      Serial.print("  sample mech="); Serial.println(m, 2);
    }
    delay(50);
  }

  applyVoltage(0.0f, 0.0f);   // 松力

  if (n == 0) {
    Serial.println("READ FAIL: check AS5600");
    return;
  }
  M /= n;

  float offset = M - 90.0f / P;

  Serial.print("M = "); Serial.println(M, 2);
  Serial.print("NEW OFFSET = "); Serial.println(offset, 2);
  Serial.print("OLD OFFSET = "); Serial.println(OLD_OFFSET, 2);
  Serial.print("DIFF = "); Serial.println(offset - OLD_OFFSET, 2);
  Serial.println("(DIFF is multiple of 51.43 => equivalent)");
}

void loop() {
  delay(1000);
}
