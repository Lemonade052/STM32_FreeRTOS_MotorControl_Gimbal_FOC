#include <math.h>

// 开环电压 FOC 首转：不用 AS5600，θ 是自己斜坡生成的「假想角」
// 24V 供电，Vq 先给 1.0V，电角度 1Hz ≈ 机械 8.6 rpm（约 7 秒一圈）
// 载板 U1：IN1=PA0, IN2=PA1, IN3=PA2
// 期望：电机平滑慢转、声音轻；只抖不转就把 VQ 改 1.5 再试

#define IN1 PA0
#define IN2 PA1
#define IN3 PA2

const float UDC      = 24.0f;   // 电源电压（24V 适配器）
const float VQ       = 1.0f;    // 装配后 1.0 比 1.5 更丝滑（齿槽/粘滑，不是力矩不够）
const float ELEC_HZ  = 1.0f;    // 电角度频率；1Hz ≈ 8.6 rpm 机械（7 对极）
// TWO_PI 用 core 自带的宏（wiring_constants.h），不要再定义

float theta = 0.0f;             // 电角度（弧度）
unsigned long last_us = 0;
unsigned long last_print = 0;

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

  // 此版本 core 是单参数；若编译报错改成 analogWriteFrequency(IN1, 20000)
  analogWriteFrequency(20000);

  // 先把三相设成 50%（等效 0V 线电压）
  setDuty(0.5f, 0.5f, 0.5f);

  Serial.begin(115200);
  delay(300);
  Serial.print("Open-loop FOC: Vq=");
  Serial.print(VQ, 2);
  Serial.println("V, 1Hz elec (~8.6 rpm)");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_us) * 0.000001f;
  last_us = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;   // 首次/异常保护

  // ① 电角度斜坡
  theta += TWO_PI * ELEC_HZ * dt;
  if (theta >= TWO_PI) theta -= TWO_PI;

  // ② 反 Park（Vd = 0）
  float c = cosf(theta);
  float s = sinf(theta);
  float valpha = -VQ * s;
  float vbeta  =  VQ * c;

  // ③ 反 Clark（等幅值变换）
  float va = valpha;
  float vb = -0.5f * valpha + 0.8660254f * vbeta;
  float vc = -0.5f * valpha - 0.8660254f * vbeta;

  // ④ SVPWM：减中点（零序注入），再加 1/2 偏置
  float vmax = fmaxf(va, fmaxf(vb, vc));
  float vmin = fminf(va, fminf(vb, vc));
  float mid  = 0.5f * (vmax + vmin);

  float da = (va - mid) / UDC + 0.5f;
  float db = (vb - mid) / UDC + 0.5f;
  float dc = (vc - mid) / UDC + 0.5f;
  setDuty(da, db, dc);

  // 每 200ms 打印一次，确认程序在跑
  if (millis() - last_print >= 200) {
    last_print = millis();
    Serial.print("theta=");
    Serial.print(theta * 57.29578f, 1);
    Serial.print(" deg  Vq=");
    Serial.print(VQ, 1);
    Serial.print("V  duty=(");
    Serial.print(da, 3); Serial.print(",");
    Serial.print(db, 3); Serial.print(",");
    Serial.print(dc, 3); Serial.println(")");
  }

  delay(1);   // ~1kHz 更新
}
