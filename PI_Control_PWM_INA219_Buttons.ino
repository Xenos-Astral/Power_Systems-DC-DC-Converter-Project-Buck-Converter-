#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>

// Pin Definitions
const int PWM_OUTPUT = 11;  // PWM output to gate driver
const int VOLTAGE_DIVIDER_PIN = A0; // Analog input for voltage measurement
const int BUTTON_UP = 7;     // Button to increase voltage
const int BUTTON_DOWN = 8;   // Button to decrease voltage

// Control Variables
float DesiredVoltage = 3.3; // Default output voltage
float MeasuredVoltage = 0.0;
int DutyCycle = 0;

// PI Control Parameters
float kp = 0.6;  // Proportional gain
float ki = 0.1;  // Integral gain
float ErrorSum = 0;
const float maxIntegral = 50.0; // Anti-windup limit

// Voltage Divider Constants
const float R1 = 10000.0; // 10kΩ
const float R2 = 6800.0;  // 6.8kΩ
const float VREF = 3.3;   // ADC Reference voltage
const int ADC_MAX = 4095; // ESP32 ADC Resolution (12-bit)

// INA219 & LCD Setup
Adafruit_INA219 ina219;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    Serial.begin(9600);
    Wire.begin();

    pinMode(PWM_OUTPUT, OUTPUT);
    pinMode(BUTTON_UP, INPUT_PULLUP);
    pinMode(BUTTON_DOWN, INPUT_PULLUP);

    if (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        while (1) { delay(10); }
    }

    lcd.init();
    lcd.backlight();
    Serial.println("DC-DC Converter with Adjustable Voltage & Power Monitoring");
}

void loop() {
    static unsigned long lastButtonPress = 0;
    
    // Debounce and adjust desired voltage
    if (millis() - lastButtonPress > 300) {
        if (digitalRead(BUTTON_UP) == LOW) {
            DesiredVoltage = min(DesiredVoltage + 0.1, 5.5);
            lastButtonPress = millis();
        } 
        if (digitalRead(BUTTON_DOWN) == LOW) {
            DesiredVoltage = max(DesiredVoltage - 0.1, 2.5);
            lastButtonPress = millis();
        }
    }

    // Read ADC voltage
    int rawADC = analogRead(VOLTAGE_DIVIDER_PIN);
    float rawVoltage = (rawADC * VREF) / ADC_MAX;
    MeasuredVoltage = rawVoltage * ((R1 + R2) / R2);

    // Compute new PWM value using PI control
    DutyCycle = PI_Controller(DesiredVoltage, MeasuredVoltage);
    analogWrite(PWM_OUTPUT, DutyCycle);

    // Read power data
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW = ina219.getPower_mW();
    float efficiency = (power_mW > 0) ? ((busVoltage * current_mA) / power_mW) * 100 : 0;

    // Display on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("V:"); lcd.print(busVoltage, 2);
    lcd.print(" I:"); lcd.print(current_mA, 1);

    lcd.setCursor(0, 1);
    lcd.print("Eff: "); lcd.print(efficiency, 1);
    lcd.print("%");

    // Debug Output
    Serial.print("Setpoint: "); Serial.print(DesiredVoltage); Serial.print(" V, ");
    Serial.print("Measured: "); Serial.print(MeasuredVoltage); Serial.print(" V, ");
    Serial.print("PWM: "); Serial.print(DutyCycle);
    Serial.print(" Power: "); Serial.print(power_mW); Serial.println(" mW");

    delay(200);
}

// PI Controller Function
int PI_Controller(float desired, float measured) {
    float error = desired - measured;
    ErrorSum += error;
    ErrorSum = constrain(ErrorSum, -maxIntegral, maxIntegral);

    float controlSignal = (kp * error) + (ki * ErrorSum);
    int pwmOutput = constrain(map(controlSignal, 0, VREF, 0, 255), 0, 255);
    return pwmOutput;
}
