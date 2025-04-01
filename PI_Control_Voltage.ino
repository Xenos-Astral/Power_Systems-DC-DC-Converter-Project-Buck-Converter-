// Code for the PI Control, using a voltage divider to Voltage measurements for Gate driver control
// Pin Definitions
const int PWM_OUTPUT = 11;  // PWM output pin for gate driver
const int VOLTAGE_DIVIDER_PIN = A0; // Analog input for voltage measurement

// Control Variables
const float DesiredVoltage = 2.0; // Desired output voltage (adjust as needed)
float MeasuredVoltage = 0.0;
int DutyCycle = 0;

// Time Tracking
unsigned long timePassed = 0;

// PI Control Parameters
float kp = 0.6;  // Proportional gain
float ki = 0.1;  // Integral gain
float ErrorSum = 0;
const float maxIntegral = 50.0; // Anti-windup limit

// Low-pass filter factor (smoothing the voltage measurement)
float alpha = 0.1;

// Voltage Divider Constants (Adjust according to resistor values)
const float R1 = 10000.0; // 10kΩ
const float R2 = 6800.0;  // 6.8kΩ
const float VREF = 3.3;   // Reference voltage of ADC
const int ADC_MAX = 4095; // ESP32 ADC Resolution (12-bit)

// PI Controller Function
int PI_Controller(float desired, float measured) {
    float error = desired - measured;
    ErrorSum += error;
    
    // Anti-windup: Limit ErrorSum within bounds
    ErrorSum = constrain(ErrorSum, -maxIntegral, maxIntegral);

    // Compute PI control output
    float controlSignal = (kp * error) + (ki * ErrorSum);
    
    // Convert to PWM range (0-255)
    int pwmOutput = constrain(map(controlSignal, 0, VREF, 0, 255), 0, 255);
    return pwmOutput;
}

void setup() {
    Serial.begin(9600);
    pinMode(PWM_OUTPUT, OUTPUT);
    timePassed = millis();
}

void loop() {
    if (millis() - timePassed > 50) { // Faster update for stability
        timePassed = millis();
        
        // Read ADC voltage from voltage divider
        int rawADC = analogRead(VOLTAGE_DIVIDER_PIN);
        float rawVoltage = (rawADC * VREF) / ADC_MAX;
        
        // Compute actual measured voltage using voltage divider formula
        MeasuredVoltage = rawVoltage * ((R1 + R2) / R2);
        
        // Compute new PWM value using PI control
        DutyCycle = PI_Controller(DesiredVoltage, MeasuredVoltage);
        
        // Apply PWM output
        analogWrite(PWM_OUTPUT, DutyCycle);
        
        // Print data for debugging
        Serial.print("Setpoint: "); Serial.print(DesiredVoltage); Serial.print(" V, ");
        Serial.print("Measured: "); Serial.print(MeasuredVoltage); Serial.print(" V, ");
        Serial.print("PWM: "); Serial.println(DutyCycle);
    }
}
