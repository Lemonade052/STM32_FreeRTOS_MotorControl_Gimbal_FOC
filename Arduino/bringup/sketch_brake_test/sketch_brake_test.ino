// 载板 12V 首次上电保险测试
// 三相 IN 全低 = 下桥全开 = 电机刹车：电机不会转、不会出声
// 用法：先烧这块代码 → 断电接电机 + 12V → 先 USB 后 12V（限流 0.3A）
// 期望：电源电流很小、模块不烫、电机不转

#define IN1 PA0
#define IN2 PA1
#define IN3 PA2

void setup() {
  // 先第一时间把三相拉低（别等 Serial）
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);

  Serial.begin(115200);
  delay(200);
  Serial.println("BRAKE: IN1=IN2=IN3=LOW");
  Serial.println("USB ok. Now apply 12V with 0.3A current limit.");
}

void loop() {
  // 保持全低；每秒打印一次，确认程序在跑、没复位
  static unsigned long t = 0;
  if (millis() - t >= 1000) {
    t = millis();
    Serial.println("holding brake");
  }
}
