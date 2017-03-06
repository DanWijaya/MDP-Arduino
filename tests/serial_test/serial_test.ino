bool stringReceived = false;
String inputString = "";

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  pinMode(13, OUTPUT);
}

void loop() {
  if (stringReceived) {
    if (inputString[0] == 'w'){
      Serial.print(inputString);
    }
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


