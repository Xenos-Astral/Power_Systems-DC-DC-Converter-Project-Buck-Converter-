// Pin Assignments
const int buttonUp = 2;         // Increase voltage setpoint
const int buttonDown = 3;       // Decrease voltage setpoint

void setup() {
    pinMode(buttonUp, INPUT);   // External 10K pull-down resistor required
    pinMode(buttonDown, INPUT); // External 10K pull-down resistor required
    Serial.begin(9600);  // Initialize Serial Monitor
}

void loop() {
    // Read button states
    int button1State = digitalRead(buttonUp);
    int button2State = digitalRead(buttonDown);

    // Print button states to Serial Monitor
    Serial.print("Button Up: ");
    Serial.print(button1State == HIGH ? "Pressed" : "Not Pressed");

    Serial.print(" | Button Down: ");
    Serial.println(button2State == HIGH ? "Pressed" : "Not Pressed");

    delay(1000);  // Small delay for readability
}
