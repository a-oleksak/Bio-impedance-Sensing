void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("=== Bluetooth Remote Control ===");
  Serial.println("Commands:");
  Serial.println("  '1' - Turn LED ON");
  Serial.println("  '0' - Turn LED OFF");
  Serial.println("  'b' - Blink LED 5 times");
}

void loop() {
  if(Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case '1':
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("LED ON");
        break;
        
      case '0':
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("LED OFF");
        break;
        
      case 'b':
        Serial.println("Blinking LED 5 times...");
        for(int i = 0; i < 5; i++) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(200);
          digitalWrite(LED_BUILTIN, LOW);
          delay(200);
        }
        Serial.println("\n Blink complete!");
        break;
        
      default:
        Serial.print("Unknown");
    }
  }
}