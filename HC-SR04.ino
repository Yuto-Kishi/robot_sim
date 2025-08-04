#define TRIG_PIN 9
#define ECHO_PIN 10

// デフォルト音速（20℃時、m/s）
const float SOUND_SPEED = 343.0;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // トリガーパルス送信（10us以上）
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Echoパルス幅計測（タイムアウト35ms = 約6m相当）
  long duration = pulseIn(ECHO_PIN, HIGH, 35000);

  if (duration == 0) {
    Serial.println("No Echo received (timeout)");
  } else {
    // 距離 = 時間 × 音速 / 2
    float distance_cm = duration * (SOUND_SPEED / 10000.0) / 2;
    Serial.print("Distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
  }

  delay(500); // 測定間隔は200ms以上
}
