void setup() {
    Serial.begin(9600);  // Initialize hardware serial for communication with Xilinx board
}

void loop() {
    if (Serial.available() >= 8) {
        double esc1 = (Serial.read() << 8) | Serial.read();
        double esc2 = (Serial.read() << 8) | Serial.read();
        double esc3 = (Serial.read() << 8) | Serial.read();
        double esc4 = (Serial.read() << 8) | Serial.read();

        // Process the ESC values as needed
        // For example, setting PWM values for motor control

        // Print received values to serial monitor (optional for debugging)
        Serial.print("ESC1: ");
        Serial.print(esc1);
        Serial.print("\t");
        Serial.print(" ESC2: ");
        Serial.print(esc2);
        Serial.print("\t");
        Serial.print(" ESC3: ");
        Serial.print(esc3);
        Serial.print("\t");
        Serial.print(" ESC4: ");
        Serial.println(esc4);
    }
    else{
      Serial.print("00000");
    }
}
