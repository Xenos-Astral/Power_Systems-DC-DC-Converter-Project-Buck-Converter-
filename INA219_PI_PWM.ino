//PI and INA219 Voltage Control with Controlled set Voltage point.

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>

// Initialize INA219 Current Sensor
Adafruit_INA219 ina219;

// Initialize LCD (Change address if needed)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin Definitions
const int LED_PWM = 11;  // PWM output to control voltage

// Control Variables
const float DesiredVoltage = 2.0; // Target voltage
float MeasuredVoltage = 0.0;
int DutyCycle = 0;

// PI Control Parameters
float kp = 0.8;
float ki = 0.2;
float ErrorSum = 0;
const float maxIntegral = 50.0;

// Low-pass filter factor
float alpha = 0.1;

// Time Tracking
unsigned long timePassed = 0;

// PI Controller Function
int PI_Controller(float desired, float measured) {
    float error = desired - measured;
    ErrorSum += error;
    ErrorSum = constrain(ErrorSum, -maxIntegral, maxIntegral);
    float controlSignal = (kp * error) + (ki * ErrorSum);
    int pwmOutput = constrain(map(controlSignal, 0, 3.3, 0, 255), 0, 255);
    return pwmOutput;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize INA219
    if (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        while (1) { delay(10); }
    }

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    pinMode(LED_PWM, OUTPUT);
    timePassed = millis();
    Serial.println("System Initialized...");
}

void loop() {
    if (millis() - timePassed > 50) {
        timePassed = millis();
        
        // Read INA219 Sensor Data
        float busvoltage = ina219.getBusVoltage_V();
        float current_mA = ina219.getCurrent_mA();
        float power_mW = ina219.getPower_mW();
        float efficiency = (power_mW > 0) ? ((busvoltage * current_mA) / power_mW) * 100 : 0;
        
        // Simulated measured voltage with smoothing
        float rawVoltage = (DutyCycle / 255.0) * 3.3;
        MeasuredVoltage = (alpha * rawVoltage) + ((1 - alpha) * MeasuredVoltage);

        // Compute new PWM value
        DutyCycle = PI_Controller(DesiredVoltage, MeasuredVoltage);
        analogWrite(LED_PWM, DutyCycle);

        // Print to Serial Monitor
        Serial.print("V: "); Serial.print(busvoltage); Serial.print(" V, ");
        Serial.print("I: "); Serial.print(current_mA); Serial.print(" mA, ");
        Serial.print("P: "); Serial.print(power_mW); Serial.print(" mW, ");
        Serial.print("Eff: "); Serial.print(efficiency); Serial.print(" %, ");
        Serial.print("Measured: "); Serial.print(MeasuredVoltage); Serial.print(" V, ");
        Serial.print("Duty: "); Serial.println(DutyCycle);
        
        // Display on LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("V:"); lcd.print(busvoltage, 2);
        lcd.print(" I:"); lcd.print(current_mA, 1);

        lcd.setCursor(0, 1);
        lcd.print("Eff: "); lcd.print(efficiency, 1);
        lcd.print("%");
    }
}
