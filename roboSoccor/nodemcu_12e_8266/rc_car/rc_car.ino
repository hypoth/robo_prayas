// Activity 1: Blink the onboard LED

#define LED_PIN 2   // change to 4 if your board's onboard LED doesn't respond

void setup() {

  pinMode(LED_PIN, OUTPUT);

}

void loop() {

  digitalWrite(LED_PIN, HIGH);  // LED ON

  delay(1000);                  // wait 1 second

  digitalWrite(LED_PIN, LOW);   // LED OFF

  delay(1000);                  // wait 1 second

}
