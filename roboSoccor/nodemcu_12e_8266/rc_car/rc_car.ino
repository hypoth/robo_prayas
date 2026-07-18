// Activity 1b: Blink + Serial print
#define LED_PIN 2

void setup() {

  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);   // start serial communication at 115200 baud

  Serial.println("ESP32 Serial started!");

}

void loop() {

  digitalWrite(LED_PIN, HIGH);

  Serial.println("LED ON");

  delay(1000);

  digitalWrite(LED_PIN, LOW);

  Serial.println("LED OFF");

  delay(1000);

}
