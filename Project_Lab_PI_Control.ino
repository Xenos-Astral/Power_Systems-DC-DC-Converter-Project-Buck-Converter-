// PI-Controlled PWM with Improved Stability

// Pin Definitions
const int LED_PWM = 11;  // PWM output to LED

// Control Variables
const float DesiredVoltage = 2.0; // Fixed voltage for testing (set between 0 - 3.3V)
float MeasuredVoltage = 0.0; // Simulated measured voltage
int DutyCycle = 0;           // PWM output duty cycle

// Time Tracking
unsigned long timePassed = 0;

// PI Control Parameters (Tuned for Stability)
float kp = 0.6;  // Reduced Proportional gain
float ki = 0.1;  // Reduced Integral gain
float ErrorSum = 0; // Integral term accumulator
const float maxIntegral = 50.0; // Anti-windup limit

// Low-pass filter factor (smoothing the voltage measurement)
float alpha = 0.1; 

// PI Controller Function
int PI_Controller(float desired, float measured) {
    float error = desired - measured; // Compute error
    ErrorSum += error; 

    // Anti-windup: Limit ErrorSum within bounds
    ErrorSum = constrain(ErrorSum, -maxIntegral, maxIntegral);

    // Compute PI control output
    float controlSignal = (kp * error) + (ki * ErrorSum);
    
    // Convert to PWM range (0-255)
    int pwmOutput = constrain(map(controlSignal, 0, 3.3, 0, 255), 0, 255);
    return pwmOutput;
}

void setup() {
    Serial.begin(9600);
    pinMode(LED_PWM, OUTPUT);
    timePassed = millis();
}

void loop() {
    if (millis() - timePassed > 50) { // Faster update for stability
        timePassed = millis();
        
        // Simulated measured voltage with smoothing (low-pass filter)
        float rawVoltage = (DutyCycle / 255.0) * 3.3;
        MeasuredVoltage = (alpha * rawVoltage) + ((1 - alpha) * MeasuredVoltage);

        // Compute new PWM value using PI control
        DutyCycle = PI_Controller(DesiredVoltage, MeasuredVoltage);
        
        // Apply PWM output
        analogWrite(LED_PWM, DutyCycle);
        
        // Print data for plotting
        Serial.print(DesiredVoltage); Serial.print(",");
        Serial.print(MeasuredVoltage); Serial.print(",");
        Serial.println(DutyCycle);
    }
}
