// Pin Assignments
const int buttonUp = 2;         // Increase voltage setpoint
const int buttonDown = 3;       // Decrease voltage setpoint
const int LED_PWM = 11;         // PWM output to LED

// Control Variables
float DesiredVoltage = 2.0; // Initial voltage setpoint
float MeasuredVoltage = 0.0; // Simulated measured voltage
int DutyCycle = 0;           // PWM output duty cycle

// Time Tracking
unsigned long timePassed = 0;

// PI Control Parameters (Tuned for Stability)
float kp = 0.8;  // Reduced Proportional gain
float ki = 0.2;  // Reduced Integral gain
float ErrorSum = 0; // Integral term accumulator
const float maxIntegral = 50.0; // Anti-windup limit

// Low-pass filter factor (smoothing the voltage measurement)
float alpha = 0.1; 

// Voltage adjustment step
const float voltageStep = 0.1; // 0.1V per button press
const float minVoltage = 0.0;
const float maxVoltage = 3.3;

void setup() {
    pinMode(buttonUp, INPUT);   // External 10K pull-down resistor required
    pinMode(buttonDown, INPUT); // External 10K pull-down resistor required
    pinMode(LED_PWM, OUTPUT);
    Serial.begin(9600);
    timePassed = millis();
}

// PI Controller Function
int PI_Controller(float desired, float measured) {
    float error = desired - measured; // Compute error
    ErrorSum += error; 

    // Anti-windup: Limit ErrorSum within bounds
    ErrorSum = constrain(ErrorSum, -maxIntegral, maxIntegral);

    // Compute PI control output
    float PropError = kp * error;
    float TotError = PropError + (ki * ErrorSum);

    // Convert to PWM range (0-255)
    int pwmOutput = constrain(map(TotError, 0, 3.3, 0, 255), 0, 255);
    return pwmOutput;
}

void loop() {
    // Read button states
    int button1State = digitalRead(buttonUp);
    int button2State = digitalRead(buttonDown);

    // Adjust desired voltage based on button presses
    if (button1State == HIGH) {
        DesiredVoltage = min(DesiredVoltage + voltageStep, maxVoltage);
    }
    if (button2State == HIGH) {
        DesiredVoltage = max(DesiredVoltage - voltageStep, minVoltage);
    }

    // Print button states and desired voltage to Serial Monitor
    Serial.print("Button Up: ");
    Serial.print(button1State == HIGH ? "Pressed" : "Not Pressed");
    Serial.print(" | Button Down: ");
    Serial.print(button2State == HIGH ? "Pressed" : "Not Pressed");
    Serial.print(" | Desired Voltage: ");
    Serial.println(DesiredVoltage, 2);

    // Update PI controller every 50ms
    if (millis() - timePassed > 50) {
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
    
    delay(100);  // Small delay to avoid excessive processing
}
