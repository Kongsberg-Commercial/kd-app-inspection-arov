const int MOSFET_A_PIN = 5;  // Retrieve
const int MOSFET_B_PIN = 6;  // Deploy

int currentCommand = 0;       // -1 (deploy), 0 (stop), 1 (retrieve)
unsigned long commandStartTime = 0;
const unsigned long ACTIVE_DURATION = 10000;  // 10 seconds in milliseconds
bool commandActive = false;

void setup() {
  pinMode(MOSFET_A_PIN, OUTPUT);
  pinMode(MOSFET_B_PIN, OUTPUT);
  digitalWrite(MOSFET_A_PIN, LOW);
  digitalWrite(MOSFET_B_PIN, LOW);

  Serial.begin(9600);
}

void loop() {
  // Check for new serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Stop immediately if "0"
    if (input == "0") {
      Serial.println("Stop...");

      stopAll();
    }
    // Start timed action
    else if (input == "-1" || input == "1") {
      currentCommand = input.toInt();
      commandStartTime = millis();
      commandActive = true;

      if (currentCommand == -1) {
        Serial.println("Deploying...");
        digitalWrite(MOSFET_A_PIN, LOW);
        digitalWrite(MOSFET_B_PIN, HIGH);  // Deploy
      } else if (currentCommand == 1) {
        Serial.println("Retrieving...");
        digitalWrite(MOSFET_A_PIN, HIGH);  // Retrieve
        digitalWrite(MOSFET_B_PIN, LOW);
      }
    }
  }

  // Check if 10s has passed
  if (commandActive && millis() - commandStartTime >= ACTIVE_DURATION) {
    Serial.println("10 seconds passed. Stopping.");
    stopAll();
  }
}

void stopAll() {
  digitalWrite(MOSFET_A_PIN, LOW);
  digitalWrite(MOSFET_B_PIN, LOW);
  commandActive = false;
  currentCommand = 0;
}
