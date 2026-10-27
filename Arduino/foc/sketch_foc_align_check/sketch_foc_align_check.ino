#include <Wire.h>
#include <math.h>

// 对齐复核 + 方向自检（载板版 / 24V）
// 1) 先读起始角 M0
// 2) Vq=3V 强对齐，读 M1（对比 M0，看转子到底动没动）
// 3) 用新 offset 发一个「M1 + 10°」的静态目标，读 M2
// 期望：
//   M1 - M0 明显非 0        → 转子真的被对齐拉动了
//   M2 - M1 ≈ +10°          → offset/方向正确
//   M2 - M1 ≈ -15.7°        → offset 差 180° 电角（≈25.71° 机械），闭环方向会反

const int IN1 = PA0, IN2 = PA1, IN3 = PA2;
const float UDC = 24.0f;
const float P   = 7.0f;
const float VQ_ALIGN = 3.0f;    // 强对齐（短时间，约 1.2A）
const float VQ_TEST  = 1.5f;    // 静态位置测试
const float DEG2RAD  = 0.0174533f;

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

float readAvg(int n) {
  float sum = 0.0f;
  int k = 0;
  for (int i = 0; i < n; i++) {
    float m = readMech();
    if (m >= 0.0f) { sum += m; k++; }
    delay(50);
  }
  return (k > 0) ? (sum / k) : -1.0f;
}

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
  applyVoltage(0.0f, 0.0f);

  Serial.println("== align check ==");

  float M0 = readAvg(5);
  Serial.print("M0 (start) = "); Serial.println(M0, 2);

  // 强对齐：固定 elec=0，Vq=3V，2 秒
  applyVoltage(VQ_ALIGN, 0.0f);
  delay(2000);
  float M1 = readAvg(10);
  applyVoltage(0.0f, 0.0f);
  delay(300);

  if (M1 < 0.0f) {
    Serial.println("READ FAIL at M1");
    return;
  }

  float offset = M1 - 90.0f / P;
  Serial.print("M1 (aligned) = "); Serial.println(M1, 2);
  Serial.print("M1 - M0 = "); Serial.println(M1 - M0, 2);
  Serial.print("NEW OFFSET = "); Serial.println(offset, 2);

  // 静态位置测试：命令 M1 + 10°
  // 注意：applyVoltage 的电压矢量比 elec 超前 90°（Vq），静态保持要减掉 90°，
  // 否则转子会停在 target + 90/P（多 12.86°）
  float target = M1 + 10.0f;
  float elec = (target - offset) * P * DEG2RAD - 1.5707963f;
  applyVoltage(VQ_TEST, elec);
  delay(1000);
  float M2 = readAvg(10);
  applyVoltage(0.0f, 0.0f);

  float d = M2 - M1;
  if (d < -180.0f) d += 360.0f;
  if (d >  180.0f) d -= 360.0f;

  Serial.print("M2 (test) = "); Serial.println(M2, 2);
  Serial.print("M2 - M1 = "); Serial.println(d, 2);
  Serial.println("EXPECT +10 if OK; -15.7 if 180deg off");
}

void loop() {
  delay(1000);
}
