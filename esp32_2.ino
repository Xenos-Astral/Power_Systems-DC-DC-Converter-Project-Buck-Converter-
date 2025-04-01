
*/

// the number of the LED pin
const int ledPin = 16;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int resolution = 8;
const int pwmChannel = 0;  // Define a PWM channel

void setup() {
  Serial.begin(115200);  // Initialize Serial Monitor
  
  // Configure LED PWM channel
  ledcAttach(ledPin,freq,resolution);
}

void loop() {
  // Increase the LED brightness
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    ledcWrite(pwmChannel, dutyCycle);
    Serial.print("PWM Duty Cycle: ");
    Serial.println(dutyCycle);
    delay(1000);
  }

  // Decrease the LED brightness
  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    ledcWrite(pwmChannel, dutyCycle);
    Serial.print("PWM Duty Cycle: ");
    Serial.println(dutyCycle);
    delay(1000);
  }
}
