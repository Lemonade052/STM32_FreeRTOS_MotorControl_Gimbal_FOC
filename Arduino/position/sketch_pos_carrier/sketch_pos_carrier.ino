#include <Wire.h>
#include <math.h>

// 位置环（载板版 / 24V / 新 offset=233.76）
// 上电不动作；串口发 0~360 设目标（有/无换行都可以），电机才开始走
// P + D + I：D 压 stick-slip，I 消死区静态误差；输出饱和时停止积分（anti-windup）

#define DEBUG_RX 1   // 1=回显收到的每个字符，确认 RX 线通不通；调试完改 0

const int IN1 = PA0, IN2 = PA1, IN3 = PA2;
const float UDC    = 24.0f;
const float P      = 7.0f;
// 2026-10-27 重装 + 交换相线后重测：M1=274.38 → OFFSET = M1 - 90/7 = 261.52
// （开环已验证 mech 随 theta 增大，方向为正；+22.27 的自检已扣除 90° 偏置）
const float OFFSET = 261.52f;

float target = 0.0f;
bool  enabled = false;            // 上电不动作，收到串口目标才开始

// 2026-10-27 装配后重调：旧增益（0.06/0.20）会大幅过冲震荡，
// 先纯 P 低增益验证方向/稳定性，再加 D、加 I
// 装配后的轴有明显静摩擦：I 太大会形成 stick-slip 极限环（±10° 慢摆）
// 2026-10-27 试小 I：KI=0.0005 + 限幅 0.06V，刚够顶静摩擦
const float KP = 0.03f;
const float KD = 0.05f;
const float KI = 0.0008f;         // I 扫描：0.0008 + 0.072V（低摩擦角不再极限环）
const float VQ_LIMIT  = 1.0f;     // 保守限幅
const float INT_LIMIT = 90.0f;    // ×KI = 0.072V

float integral = 0.0f;
float last_error = 0.0f;
bool first_run = true;
int read_fail = 0;

// 安全停：三相全低 = 下桥全开 = 刹车
void brake() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

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
  Serial.println("Pos loop P+D+I. idle: send a target (0~360) + Enter");
}

void loop() {
  // ① 串口设目标（兼容没有行尾：300ms 内没有新字符就自动提交）
  static float input_val = 0;
  static bool  input_has = false;
  static unsigned long last_rx_ms = 0;
  bool commit = false;

  while (Serial.available()) {
    char c = Serial.read();
    last_rx_ms = millis();
#if DEBUG_RX
    Serial.print("[rx:");
    Serial.write(c);
    Serial.println("]");
#endif
    if (c >= '0' && c <= '9') {
      input_val = input_val * 10 + (c - '0');
      input_has = true;
    } else {
      commit = true;
    }
  }

  if (input_has && (commit || millis() - last_rx_ms > 300)) {
    if (input_val > 360) input_val = 360;
    target = input_val;
    enabled = true;
    integral = 0.0f;
    first_run = true;          // 下一个循环重新记录 last_error，避免 D 踢
    Serial.print("target -> "); Serial.println(target, 0);
    input_val = 0;
    input_has = false;
  }

  // ② 读角度：连续 3 次失败就刹车并退出控制，不能保留上一次 PWM
  float mech = readMech();
  if (mech < 0.0f) {
    if (++read_fail >= 3) {
      brake();
      enabled = false;
      Serial.println("AS5600 READ FAIL -> brake, send target to retry");
    }
    delay(5);
    return;
  }
  read_fail = 0;

  // 没收到目标：刹车，只报当前角度
  if (!enabled) {
    brake();
    static unsigned long t_idle = 0;
    if (millis() - t_idle >= 500) {
      t_idle = millis();
      Serial.print("idle mech="); Serial.print(mech, 2);
      Serial.println("  send target + Enter (e.g. mech+10)");
    }
    delay(20);
    return;
  }

  // ③ 位置 P+D+I（误差取最短角，跨 0/360 不会绕远路）
  float error = target - mech;
  if (error >  180.0f) error -= 360.0f;
  if (error < -180.0f) error += 360.0f;
  if (first_run) { last_error = error; first_run = false; }
  float derror = error - last_error;
  last_error = error;

  // Anti-windup：输出未饱和 + 误差不大时才积分
  // - |error|>10° 不积分：大步进/大摆动时不攒偏置
  // - 输出饱和不积分
  // - INT_LIMIT=100（×0.002=0.2V），够顶静摩擦，但不会推着电机大幅摆
  float Vq = KP * error + KD * derror + KI * integral;
  if (fabsf(Vq) < VQ_LIMIT && fabsf(error) < 10.0f) {
    integral += error;
    integral = constrain(integral, -INT_LIMIT, INT_LIMIT);
    Vq = KP * error + KD * derror + KI * integral;
  }
  Vq = constrain(Vq, -VQ_LIMIT, VQ_LIMIT);

  // ④ 电角度 + 输出电压
  float elec = (mech - OFFSET) * P * 0.0174533f;
  applyVoltage(Vq, elec);

  // ⑤ 每 100ms 打印
  static int cnt = 0;
  if (++cnt >= 20) {
    cnt = 0;
    Serial.print("mech="); Serial.print(mech, 2);
    Serial.print("  err="); Serial.print(error, 2);
    Serial.print("  Vq="); Serial.print(Vq, 2);
    Serial.print("  tgt="); Serial.println(target, 0);
  }

  delay(5);
}
