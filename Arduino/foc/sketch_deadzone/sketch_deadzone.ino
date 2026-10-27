#include <Wire.h>
#include <math.h>

// ============ 死区测试：真实 θ，Vq 三角波扫描，测电机"刚能动"的最小电压 ============
// 用法：上传后看串口。Vq 从 0 慢慢涨到 2V 再回 0。
//       盯住 "mech" 这个数：它一开始变化（电机动了）时，对应的 Vq 就是死区阈值。

const int IN1 = PA0, IN2 = PA1, IN3 = PA2;
const float UDC = 12.0f;       // 母线电压，按 SimpleFOC mini 实际供电改
const float P = 7.0f;          // 极对数
const float OFFSET = 321.30f;  // 你之前校准的电角度偏移（若怀疑它错了，这里就是被怀疑的对象）

float Vq = 0.0f;   // 扫描电压
int dir = 1;       // 扫描方向：1 涨 / -1 跌

float read_mech() {
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
  Wire.setClock(100000);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT);
  analogWriteFrequency(20000);
  Serial.println("死区测试：Vq 三角波 0~2V。盯 mech 何时开始变，那一刻的 Vq = 死区阈值");
}

void loop() {
  float mech = read_mech();
  if (mech < 0.0f) { delay(50); return; }   // 读失败跳过本轮

  float elec = (mech - OFFSET) * P * PI / 180.0f;   // 真实电角度（力矩始终垂直转子）

  // 反 Park
  float valpha = -Vq * sin(elec);
  float vbeta  =  Vq * cos(elec);

  // 反 Clark
  float ua = valpha;
  float ub = -0.5f*valpha + 0.8660254f*vbeta;
  float uc = -0.5f*valpha - 0.8660254f*vbeta;

  // SVPWM 鞍形波（三次谐波注入）
  float mid = (fmin(ua, fmin(ub, uc)) + fmax(ua, fmax(ub, uc))) * 0.5f;
  ua += mid;  ub += mid;  uc += mid;

  // 占空比 → PWM
  analogWrite(IN1, (int)(255.0f * (ua + UDC*0.5f) / UDC));
  analogWrite(IN2, (int)(255.0f * (ub + UDC*0.5f) / UDC));
  analogWrite(IN3, (int)(255.0f * (uc + UDC*0.5f) / UDC));

  // 打印：当前 Vq 和角度（mech 变了 = 电机动了）
  Serial.print("Vq="); Serial.print(Vq);
  Serial.print("  mech="); Serial.println(mech);

  // 三角波扫描：0 → 2V → 0，每 1 秒步进 0.05V（慢，方便盯）
  Vq += dir * 0.05f;
  if (Vq >= 2.0f) { Vq = 2.0f; dir = -1; }
  if (Vq <= 0.0f) { Vq = 0.0f; dir = 1; }
  delay(1000);
}
