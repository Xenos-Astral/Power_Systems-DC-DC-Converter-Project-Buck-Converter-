// Pin Assignments
const int buttonUp = 2;         // Increase voltage setpoint
const int buttonDown = 3;       // Decrease voltage setpoint

double setpoint = 3.3;         // Desired output voltage (default 3.3V)

void setup() {
    pinMode(buttonUp, INPUT);   // External 10K pull-down resistor required
    pinMode(buttonDown, INPUT); // External 10K pull-down resistor required

    Serial.begin(9600);  // Initialize Serial Monitor
    Serial.println("Button Control Initialized");
}

void loop() {
    // Read button states
    int button1State = digitalRead(buttonUp);
    int button2State = digitalRead(buttonDown);

    // Adjust setpoint with buttons
    if (button1State == HIGH) {
        setpoint += 0.1;  // Increase by 0.1V
        delay(200);        // Debounce delay
    }
    if (button2State == HIGH) {
        setpoint -= 0.1;  // Decrease by 0.1V
        delay(200);        // Debounce delay
    }

    // Print button states to Serial Monitor
    Serial.print("Button Up: ");
    Serial.print(button1State == HIGH ? "Pressed" : "Not Pressed");

    Serial.print(" | Button Down: ");
    Serial.println(button2State == HIGH ? "Pressed" : "Not Pressed");

    delay(500);
}
