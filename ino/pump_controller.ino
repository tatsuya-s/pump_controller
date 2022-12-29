const int ena_pin = 4;
const int dir_pin = 5;
const int pwm_pin = 6;

void setup() {
  Serial.begin(9600);
  pinMode(ena_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    const String data = Serial.readStringUntil('\n');
    if (data.length() > 0) {
      const long value = data.toInt();
      Serial.print(value);

      const bool dir = value >= 0;
      const long pulse = abs(value);

      digitalWrite(ena_pin, HIGH);
      digitalWrite(dir_pin, dir);

      for (int i = 0; i < pulse; ++i) {
        digitalWrite(pwm_pin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(pwm_pin, LOW);
        delayMicroseconds(2000);
      }

      digitalWrite(ena_pin, LOW);
      Serial.println(" -> Done");
    }
  }
}
