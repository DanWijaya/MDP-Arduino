bool stringReceived = false;
String inputString = "";

void setup() {
  Serial.begin(9600);
  inputString.reserve(200);
}

void loop() {
  if (stringReceived) {
    Serial.print(inputString);
    inputString = "";
    stringReceived = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringReceived = true;
    }
  }
}


