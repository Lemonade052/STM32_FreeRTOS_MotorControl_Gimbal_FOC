#include <Wire.h>
#include <math.h>

// ============ 无刷 FOC 手动模式：串口控制 正转/反转/停/调速 ============
// 发命令，电机立刻响应：
//   f / F → 正转
//   b / B → 反转
//   s / S → 停
//   1~5   → 正转调速（1→1.0V ... 5→3.0V）

const int IN1 = PA0, IN2 = PA1, IN3 = PA2;
const float UDC = 12.0f;       // 母线电压
const float P = 7.0f;          // 极对数
const float OFFSET = 321.30f;  // 电角度偏移（你校准的）

float dir = 0;       // 方向：1=正转 -1=反转 0=停
float speed = 2.0f;  // 速度幅度（V），默认 2V

float read_mech() {
  // 连读 3 次取中值，滤掉 PWM 干扰造成的偶发错误读值
  float v[3];
  int n = 0;
  for (int i = 0; i < 3; i++) {
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    if (Wire.endTransmission() != 0) continue;    // 没 ACK，跳过这次
    if (Wire.requestFrom(0x36, 2) != 2) continue; // 没读到 2 字节，跳过
    int high = Wire.read();
    int low  = Wire.read();
    int raw = ((high & 0x0F) << 8) | low;
    v[n++] = raw * 360.0f / 4096.0f;
  }
  if (n == 0) return -1.0f;
  // 排序取中值
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (v[i] > v[j]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
  return v[n / 2];
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT);
  analogWriteFrequency(10000);   // 降到 10kHz，减少 PWM 对 I2C 的干扰（体验用，啸叫无所谓）
  Serial.println("手动模式：f=正转 b=反转 s=停 1~5=调速");
}

void loop() {
  // ① 处理串口命令
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '5') {
      speed = 1.0f + 0.5f * (c - '1');   // 1→1.0V ... 5→3.0V
      dir = 1;                            // 按数字默认正转（别让 dir 停在 0）
      Serial.print("正转 速度 "); Serial.println(speed);
    }
    else if (c == 'b' || c == 'B') { dir = -1; Serial.println("反转"); }
    else if (c == 's' || c == 'S') { dir = 0;  Serial.println("停"); }
    else if (c == 'f' || c == 'F') { dir = 1;  Serial.println("正转"); }
  }

  // ② 读角度 → 电角度（真实 θ）
  float mech = read_mech();
  if (mech < 0) { delay(5); return; }   // 读失败跳过本轮
  float elec = (mech - OFFSET) * P * PI / 180.0f;

  float Vq = dir * speed;   // 最终指令电压（带符号）

  // ③ 反 Park
  float valpha = -Vq * sin(elec);
  float vbeta  =  Vq * cos(elec);

  // ④ 反 Clark + SVPWM（鞍形波）
  float ua = valpha;
  float ub = -0.5f*valpha + 0.8660254f*vbeta;
  float uc = -0.5f*valpha - 0.8660254f*vbeta;
  float mid = (fmin(ua, fmin(ub, uc)) + fmax(ua, fmax(ub, uc))) * 0.5f;
  ua += mid;  ub += mid;  uc += mid;

  // ⑤ 占空比 → PWM
  analogWrite(IN1, (int)(255.0f * (ua + UDC*0.5f) / UDC));
  analogWrite(IN2, (int)(255.0f * (ub + UDC*0.5f) / UDC));
  analogWrite(IN3, (int)(255.0f * (uc + UDC*0.5f) / UDC));

  // 每 500ms 打印一次当前状态（不刷屏）
  static unsigned long last = 0;
  if (millis() - last > 500) {
    last = millis();
    Serial.print("Vq="); Serial.print(dir * speed);
    Serial.print("  mech="); Serial.println(mech);
  }

  delay(1);   // 循环 ~1ms（FOC 要快，否则电机转快了 θ 过期 → 原地震荡）
}
