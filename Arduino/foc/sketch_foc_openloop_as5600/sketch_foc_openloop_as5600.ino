#include <math.h>
#include <Wire.h>

// 开环电压 FOC + 边转边读 AS5600
// 目的：验证载板在 20kHz PWM 下 I2C 不冻、不漂（面包板曾出现的干扰）
// 载板 U1：IN1=PA0, IN2=PA1, IN3=PA2；AS5600 = U4，地址 0x36
// 期望：mech 平滑增长（约 51°/s，8.6 rpm），fail=0；到 360° 回 0 正常

#define IN1 PA0
#define IN2 PA1
#define IN3 PA2

const float UDC     = 24.0f;
const float VQ      = 1.0f;     // 装配后 1.0 比 1.5 更丝滑（齿槽/粘滑，不是力矩不够）
const float ELEC_HZ = 1.0f;     // 1Hz 电角度 ≈ 8.6 rpm 机械（7 对极）

float theta = 0.0f;
unsigned long last_us = 0;
unsigned long last_print = 0;
unsigned long last_mech_ms = 0;
float last_mech = 0.0f;
int fail_cnt = 0;

float readMech() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0) return -1.0f;     // 没 ACK
  if (Wire.requestFrom(0x36, 2) != 2) return -1.0f;  // 没收全
  int high = Wire.read();
  int low  = Wire.read();
  int raw = ((high & 0x0F) << 8) | low;
  return raw * 360.0f / 4096.0f;
}

void setDuty(float da, float db, float dc) {
  da = constrain(da, 0.0f, 1.0f);
  db = constrain(db, 0.0f, 1.0f);
  dc = constrain(dc, 0.0f, 1.0f);
  analogWrite(IN1, (int)(da * 255.0f));
  analogWrite(IN2, (int)(db * 255.0f));
  analogWrite(IN3, (int)(dc * 255.0f));
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);

  analogWriteFrequency(20000);      // 单参数版；报错就改两参数
  setDuty(0.5f, 0.5f, 0.5f);        // 先 50%，等效 0V 线电压

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);            // 100kHz，抗干扰强
  delay(300);

  Serial.println("Open-loop FOC + AS5600 read");
  Serial.println("Expect: mech smooth ~51 deg/s, fail=0");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_us) * 0.000001f;
  last_us = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;

  // ① 电角度斜坡
  theta += TWO_PI * ELEC_HZ * dt;
  if (theta >= TWO_PI) theta -= TWO_PI;

  // ② 反 Park（Vd=0）
  float c = cosf(theta);
  float s = sinf(theta);
  float valpha = -VQ * s;
  float vbeta  =  VQ * c;

  // ③ 反 Clark
  float va = valpha;
  float vb = -0.5f * valpha + 0.8660254f * vbeta;
  float vc = -0.5f * valpha - 0.8660254f * vbeta;

  // ④ SVPWM
  float vmax = fmaxf(va, fmaxf(vb, vc));
  float vmin = fminf(va, fminf(vb, vc));
  float mid  = 0.5f * (vmax + vmin);
  setDuty((va - mid) / UDC + 0.5f,
          (vb - mid) / UDC + 0.5f,
          (vc - mid) / UDC + 0.5f);

  // ⑤ 读 AS5600（和 PWM 同时跑，这才是测试点）
  float mech = readMech();
  if (mech < 0.0f) {
    fail_cnt++;
  }

  // 每 200ms 打印：角度 + 由 mech 算出的转速
  if (millis() - last_print >= 200) {
    last_print = millis();

    float rpm = 0.0f;
    unsigned long dm = millis() - last_mech_ms;
    if (mech >= 0.0f && dm > 0) {
      float d = mech - last_mech;
      if (d < -180.0f) d += 360.0f;     // 过 0/360 回绕
      if (d >  180.0f) d -= 360.0f;
      rpm = d / 360.0f * (60000.0f / (float)dm);
      last_mech = mech;
      last_mech_ms = millis();
    }

    Serial.print("mech=");
    Serial.print(mech, 2);
    Serial.print("  rpm=");
    Serial.print(rpm, 1);
    Serial.print("  theta=");
    Serial.print(theta * 57.29578f, 0);
    Serial.print("  fail=");
    Serial.println(fail_cnt);
  }

  delay(1);
}
