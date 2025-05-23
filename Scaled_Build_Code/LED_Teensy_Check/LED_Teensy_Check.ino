// Simple LED control via Serial on Teensy 4.0

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // onboard LED pin (usually D13)
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);            // open serial at 9600 baud
  while(!Serial) ;               // wait for USB Serial to connect
  Serial.println("Ready. Type 'on' or 'off'.");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();                  // remove CR/LF and whitespace
    if (cmd.equalsIgnoreCase("on")) {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("LED is ON");
    }
    else if (cmd.equalsIgnoreCase("off")) {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("LED is OFF");
    }
    else {
      Serial.println("Unknown command. Use 'on' or 'off'.");
    }
  }
}
