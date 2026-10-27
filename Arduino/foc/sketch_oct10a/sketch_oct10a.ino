#include <Wire.h>
#include <math.h>

const int IN1 = PA0, IN2 = PA1, IN3 = PA2;
const float UDC = 12.0f;
const float P = 7.0f;
const float OFFSET = 321.30f;

float target = 60.0f;
float KP = 0.03f;
const float KI = 0.002f;   // 积分系数
const float KD = 0.1f;     // 微分系数（刹车用，先小点）
const float VQ_LIMIT = 4.0f;
float integral = 0.0f;     // 积分状态（跨采样保持）

// 读角度：返回 -1 表示读取失败（I2C 被干扰 / 芯片没响应）
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
  Wire.setClock(100000);   // I2C 降到 100kHz，抗干扰更强（默认可能是 400kHz）
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT);
  analogWriteFrequency(20000);
}

void loop() {
  float mech = read_mech();

  // 读失败：角度没读准，闭环会瞎指挥 → 跳过本轮，保持上次 PWM
  if (mech < 0.0f) {
    Serial.println("READ FAIL");
    delay(50);
    return;
  }

  float elec = (mech - OFFSET) * P * PI / 180.0f;

    // ① 位置环 PID
  float error = target - mech;

  static float last_error = 0.0f;                 // D 项用：上一次误差
  float derror = error - last_error;              // 误差变化量 ≈ 电机速度的负值
  last_error = error;

    // ② 条件积分：只在误差小（接近目标）时积分；大误差=扰动，不积分防饱和
  if (fabs(error) < 5.0f) {
    integral += error;
  }
  if (integral >  2000.0f) integral =  2000.0f;   // ③ 积分限幅（防饱和）
  if (integral < -2000.0f) integral = -2000.0f;

    float Vq = KP * error + KI * integral + KD * derror;   // ④ PID 输出（D 刹车）
  if (Vq >  VQ_LIMIT) Vq =  VQ_LIMIT;           // ⑤ Vq 限幅
  if (Vq < -VQ_LIMIT) Vq = -VQ_LIMIT;

    // ⑥ 反 Park（真实 θ）
  float valpha = -Vq * sin(elec);
  float vbeta  =  Vq * cos(elec);

    // ⑦ 反 Clark + SVPWM
  float ua = valpha;
  float ub = -0.5f*valpha + 0.8660254f*vbeta;
  float uc = -0.5f*valpha - 0.8660254f*vbeta;

  float mid = (fmin(ua, fmin(ub, uc)) + fmax(ua, fmax(ub, uc))) * 0.5f;
  ua += mid;  ub += mid;  uc += mid;

    // ⑧ 占空比 → PWM
  analogWrite(IN1, (int)(255.0f * (ua + UDC*0.5f) / UDC));
  analogWrite(IN2, (int)(255.0f * (ub + UDC*0.5f) / UDC));
  analogWrite(IN3, (int)(255.0f * (uc + UDC*0.5f) / UDC));

  // 每 100ms 打印一次（原来每 5ms 打一次，太快 + 串口阻塞太久）
  static int cnt = 0;
  if (++cnt >= 20) {
    cnt = 0;
    Serial.print("mech="); Serial.print(mech);
    Serial.print(" target="); Serial.print(target);
    Serial.print(" Vq="); Serial.println(Vq);
  }

  delay(5);
}
