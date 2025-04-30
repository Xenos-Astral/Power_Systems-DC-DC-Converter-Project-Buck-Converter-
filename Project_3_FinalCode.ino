#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>

// ==== Pin Assignments ====
const int buttonUp = 2;         // Increase voltage setpoint
const int buttonDown = 3;       // Decrease voltage setpoint
const int LED_PWM = 18;         // PWM output to LED

// ==== Control Variables ====
float DesiredVoltage = 2.0;     // Initial voltage setpoint
float MeasuredVoltage = 0.0;    // Smoothed simulated voltage
int DutyCycle = 0;              // PWM output duty cycle

// ==== PI Controller Parameters ====
float kp = 0.8;
float ki = 0.2;
float ErrorSum = 0;
const float maxIntegral = 50.0;

// ==== Filter & Voltage Adjust ====
float alpha = 0.1;
const float voltageStep = 0.1;
const float minVoltage = 0.0;
const float maxVoltage = 3.3;

// ==== Time Tracking ====
unsigned long timePassed = 0;

// ==== INA219 Sensor & LCD ====
Adafruit_INA219 ina219;
bool sensorAvailable = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== Setup ====
void setup() {
  // Pin Config
  pinMode(buttonUp, INPUT);
  pinMode(buttonDown, INPUT);
  pinMode(LED_PWM, OUTPUT);

  // Serial Monitor
  Serial.begin(115200);
  Wire.begin();

  // INA219 Setup
  sensorAvailable = ina219.begin();
  if (!sensorAvailable) {
    Serial.println("INA219 not detected, continuing without sensor data.");
  }

  // LCD Setup
  lcd.init();
  lcd.backlight();

  Serial.println("System Initialized...");
  timePassed = millis();
}

// ==== PI Controller ====
int PI_Controller(float desired, float measured) {
  float error = desired - measured;
  ErrorSum += error;
  ErrorSum = constrain(ErrorSum, -maxIntegral, maxIntegral);

  float output = (kp * error) + (ki * ErrorSum);
  int pwm = constrain(map(output, 0, 3.3, 0, 255), 0, 255);
  return pwm;
}

// ==== Loop ====
void loop() {
  // --- Button Handling ---
  if (digitalRead(buttonUp) == HIGH)
    DesiredVoltage = min(DesiredVoltage + voltageStep, maxVoltage);

  if (digitalRead(buttonDown) == HIGH)
    DesiredVoltage = max(DesiredVoltage - voltageStep, minVoltage);

  // --- PI Update every 50ms ---
  if (millis() - timePassed > 50) {
    timePassed = millis();

    // Simulate measured voltage using duty cycle
    float rawVoltage = (DutyCycle / 255.0) * 3.3;
    MeasuredVoltage = alpha * rawVoltage + (1 - alpha) * MeasuredVoltage;

    // Update Duty Cycle using PI control
    DutyCycle = PI_Controller(DesiredVoltage, MeasuredVoltage);
    analogWrite(LED_PWM, DutyCycle);
  }

  float busvoltage = 0.0, current_mA = 0.0, power_mW = 0.0, efficiency = 0.0;

  // --- INA219 Reading ---
  if (sensorAvailable) {
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    efficiency = (power_mW > 0) ? ((busvoltage * current_mA) / power_mW) * 100 : 0;
  }

  // --- Serial Output ---
  Serial.print("Desired: "); Serial.print(DesiredVoltage, 2);
  Serial.print(" | Measured (Est.): "); Serial.print(MeasuredVoltage, 2);
  Serial.print(" | PWM: "); Serial.print(DutyCycle);

  if (sensorAvailable) {
    Serial.print(" | V: "); Serial.print(busvoltage, 2);
    Serial.print(" V | I: "); Serial.print(current_mA, 1);
    Serial.print(" mA | Eff: "); Serial.print(efficiency, 1);
    Serial.println(" %");
  } else {
    Serial.println(" | Sensor not available");
  }

  // --- LCD Output ---
  lcd.clear();
  lcd.setCursor(0, 0);

  if (sensorAvailable) {
    lcd.print("V:"); lcd.print(busvoltage, 2);
    lcd.print(" I:"); lcd.print(current_mA, 1);
  } else {
    lcd.print("V: -- I: --");
  }

  lcd.setCursor(0, 1);
  if (sensorAvailable) {
    lcd.print("Eff:"); lcd.print(efficiency, 1); lcd.print("% ");
  } else {
    lcd.print("Eff: ---% ");
  }

  lcd.print("SP:"); lcd.print(DesiredVoltage, 1);

  delay(500);  // Display update rate
}
