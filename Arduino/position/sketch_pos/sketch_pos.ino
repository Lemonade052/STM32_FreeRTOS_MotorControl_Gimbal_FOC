#include <Wire.h>
#include <math.h>

// ============ 位置环 + 串口设目标角度 ============
// 串口发一个数字（0~360）然后回车，电机平滑转到该角度。
// 目标比当前大 = 正转，比当前小 = 反转。

const int IN1 = PA0, IN2 = PA1, IN3 = PA2;
const float UDC = 12.0f;
const float P = 7.0f;
const float OFFSET = 321.30f;

float target = 60.0f;        // 目标角度（串口可设）
const float KP = 0.03f;      // 位置环 P
const float KI = 0.002f;     // 积分（条件积分）
const float KD = 0.1f;       // D 刹车
const float VQ_LIMIT = 4.0f;
float integral = 0.0f;

float read_mech() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0) return -1.0f;
  if (Wire.requestFrom(0x36, 2) != 2) return -1.0f;
  int high = Wire.read();
  int low  = Wire.read();
  int raw = ((high & 0x0F) << 8) | low;
  return raw * 360.0f / 4096.0f;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT);
  analogWriteFrequency(20000);
  Serial.println("位置环：串口发角度(0~360)+回车，电机转到该角度");
}

void loop() {
  // ① 串口设目标角度
  static float input_val = 0;
  static bool  input_has = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '9') {
      input_val = input_val * 10 + (c - '0');
      input_has = true;
    } else if (c == '\n' || c == '\r' || c == ' ') {
      if (input_has) {
        if (input_val > 360) input_val = 360;
        target = input_val;
        Serial.print("目标 -> "); Serial.println(target);
        input_val = 0;
        input_has = false;
      }
    }
  }

  // ② 读角度 → 电角度
  float mech = read_mech();
  if (mech < 0) { delay(5); return; }
  float elec = (mech - OFFSET) * P * PI / 180.0f;

  // ③ 位置环 PID（P + D + 条件积分）
  float error = target - mech;
  static float last_error = 0.0f;
  float derror = error - last_error;
  last_error = error;
  if (fabs(error) < 5.0f) {
    integral += error;
  }
  if (integral >  2000.0f) integral =  2000.0f;
  if (integral < -2000.0f) integral = -2000.0f;
  float Vq = KP * error + KI * integral + KD * derror;
  if (Vq >  VQ_LIMIT) Vq =  VQ_LIMIT;
  if (Vq < -VQ_LIMIT) Vq = -VQ_LIMIT;

  // ④ 反 Park
  float valpha = -Vq * sin(elec);
  float vbeta  =  Vq * cos(elec);

  // ⑤ 反 Clark + SVPWM
  float ua = valpha;
  float ub = -0.5f*valpha + 0.8660254f*vbeta;
  float uc = -0.5f*valpha - 0.8660254f*vbeta;
  float mid = (fmin(ua, fmin(ub, uc)) + fmax(ua, fmax(ub, uc))) * 0.5f;
  ua += mid;  ub += mid;  uc += mid;

  // ⑥ 占空比 → PWM
  analogWrite(IN1, (int)(255.0f * (ua + UDC*0.5f) / UDC));
  analogWrite(IN2, (int)(255.0f * (ub + UDC*0.5f) / UDC));
  analogWrite(IN3, (int)(255.0f * (uc + UDC*0.5f) / UDC));

  // ⑦ 每 100ms 打印一次
  static int cnt = 0;
  if (++cnt >= 20) {
    cnt = 0;
    Serial.print("mech="); Serial.print(mech);
    Serial.print("  target="); Serial.println(target);
  }

  delay(5);
}
