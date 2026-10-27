#include <Wire.h>
#include <math.h>

// ============ Vq 对齐 + 打印 OFFSET ============
// 目的：重新校准电角度偏移，跟旧的 321.30 对比，判断 offset 到底偏没偏

const int IN1 = PA0, IN2 = PA1, IN3 = PA2;
const float UDC = 12.0f;          // 母线电压
const float P = 7.0f;             // 极对数
const float OLD_OFFSET = 321.30f; // 旧的 offset（被怀疑的对象）

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

// 按固定电角度 elec 施加 Vq 电压（反Park → 反Clark → SVPWM → PWM）
void apply_voltage(float Vq, float elec) {
  float valpha = -Vq * sin(elec);
  float vbeta  =  Vq * cos(elec);

  float ua = valpha;
  float ub = -0.5f*valpha + 0.8660254f*vbeta;
  float uc = -0.5f*valpha - 0.8660254f*vbeta;

  float mid = (fmin(ua, fmin(ub, uc)) + fmax(ua, fmax(ub, uc))) * 0.5f;
  ua += mid;  ub += mid;  uc += mid;

  analogWrite(IN1, (int)(255.0f * (ua + UDC*0.5f) / UDC));
  analogWrite(IN2, (int)(255.0f * (ub + UDC*0.5f) / UDC));
  analogWrite(IN3, (int)(255.0f * (uc + UDC*0.5f) / UDC));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT);
  analogWriteFrequency(20000);

  Serial.println("Vq 对齐开始：固定 elec=0，Vq=3V，等 3 秒让转子对齐...");

  // 对齐：固定电角度 elec=0，Vq=3V，转子会被拉到固定位置
  apply_voltage(3.0f, 0.0f);
  delay(3000);

  // 读对齐后的机械角（读 10 次取平均，防干扰）
  float M = 0;
  int n = 0;
  for (int i = 0; i < 10; i++) {
    float m = read_mech();
    if (m >= 0) { M += m; n++; }
    delay(50);
  }
  if (n == 0) { Serial.println("读角度失败！检查接线/干扰"); }
  else M /= n;

  // 停输出（三相 50% = 中性，无力矩）
  apply_voltage(0.0f, 0.0f);

  // Vq 对齐公式：OFFSET = M - 90/P
  float OFFSET = M - 90.0f / P;

  Serial.print("对齐后机械角 M = "); Serial.println(M);
  Serial.print("新 OFFSET = M - 90/7 = "); Serial.println(OFFSET);
  Serial.print("旧 OFFSET = "); Serial.println(OLD_OFFSET);
  Serial.print("差值 = "); Serial.println(OFFSET - OLD_OFFSET);
}

void loop() {
  delay(1000);
}
