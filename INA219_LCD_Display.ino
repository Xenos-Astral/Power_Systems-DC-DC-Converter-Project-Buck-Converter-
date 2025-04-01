#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>

// Initialize INA219 Current Sensor
Adafruit_INA219 ina219;

// Initialize LCD (Change address if needed)
LiquidCrystal_I2C lcd(0x27, 16, 2); 

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

  Serial.println("Monitoring with INA219 & LCD...");
}

void loop() {
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float efficiency = (power_mW > 0) ? ((busvoltage * current_mA) / power_mW) * 100 : 0;

  // Print to Serial Monitor
  Serial.print("Bus Voltage: "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Current: "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power: "); Serial.print(power_mW); Serial.println(" mW");
  Serial.print("Efficiency: "); Serial.print(efficiency); Serial.println(" %");
  Serial.println("");

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V:"); lcd.print(busvoltage, 2);
  lcd.print(" I:"); lcd.print(current_mA, 1);

  lcd.setCursor(0, 1);
  lcd.print("Eff: "); lcd.print(efficiency, 1);
  lcd.print("%");

  delay(2000);
}
