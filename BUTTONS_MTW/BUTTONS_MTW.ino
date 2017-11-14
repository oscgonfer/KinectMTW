// RESET BUTTONS
#define RESET_BUTTON_PIN      A0
#define RESET_BUTTON_LED_PIN  3
// LAYER BUTTONS
#define LYR1_BUTTON_PIN      A1
#define LYR1_BUTTON_LED_PIN  4
#define LYR2_BUTTON_PIN      A2
#define LYR2_BUTTON_LED_PIN  5
#define LYR3_BUTTON_PIN      A3
#define LYR3_BUTTON_LED_PIN  6
#define LYR4_BUTTON_PIN      A4
#define LYR4_BUTTON_LED_PIN  7

// Button
bool buttonA0 = false; //RESET
bool buttonA1 = false; //LAYER 1
bool buttonA2 = false; //LAYER 2
bool buttonA3 = false; //LAYER 3
bool buttonA4 = false; //LAYER 4

int LastButtonPressed = 0;

void setup() {
    //LED
    pinMode(RESET_BUTTON_LED_PIN, OUTPUT);
    pinMode(LYR1_BUTTON_LED_PIN, OUTPUT);
    pinMode(LYR2_BUTTON_LED_PIN, OUTPUT);
    pinMode(LYR3_BUTTON_LED_PIN, OUTPUT);
    pinMode(LYR4_BUTTON_LED_PIN, OUTPUT);

    digitalWrite(RESET_BUTTON_LED_PIN, LOW);  
    digitalWrite(LYR1_BUTTON_LED_PIN, HIGH);
    digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
    digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
    digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
                    
    Serial.begin(9600);
    Serial.println("all good here");    
}

void loop() {

  int a0val = 0; // RESET
  int a1val = 0; // LAYER 1
  int a2val = 0; // LAYER 2
  int a3val = 0; // LAYER 3
  int a4val = 0; // LAYER 4

  a0val= analogRead(RESET_BUTTON_PIN);
  a1val= analogRead(LYR1_BUTTON_PIN);
  a2val= analogRead(LYR2_BUTTON_PIN);
  a3val= analogRead(LYR3_BUTTON_PIN);
  a4val= analogRead(LYR4_BUTTON_PIN);
  
  if (a0val > 1000 && buttonA0 == false) {

    buttonA0 = true;
    buttonA1 = false;
    buttonA2 = false;
    buttonA3 = false;
    buttonA4 = false;
    
  } else {
    buttonA0 = false;
  }

   if (a1val > 1000 && buttonA1 == false) {
    buttonA0 = false;
    buttonA1 = true;
    buttonA2 = false;
    buttonA3 = false;
    buttonA4 = false;
  } else {
    buttonA1 = false;
  }

    if (a2val > 1000 && buttonA2 == false) {
    buttonA0 = false;
    buttonA1 = false;
    buttonA2 = true;
    buttonA3 = false;
    buttonA4 = false;
  } else {
    buttonA2 = false;
  }

    if (a3val > 1000 && buttonA3 == false) {
    buttonA0 = false;
    buttonA1 = false;
    buttonA2 = false;
    buttonA3 = true;
    buttonA4 = false;
  } else {
    buttonA3 = false;
  }

    if (a4val > 1000 && buttonA4 == false) {
    buttonA0 = false;
    buttonA1 = false;
    buttonA2 = false;
    buttonA3 = false;
    buttonA4 = true;
  } else {
    buttonA4 = false;
  }

  //Send Serial messages
  if (buttonA0 == true){
    Serial.write("0");
    LastButtonPressed = 0;
  }

  if (buttonA1 == true){
    Serial.write(1);
    LastButtonPressed = 1;
  }

  if (buttonA2 == true){
    Serial.write(2);
    LastButtonPressed = 2;
  }

  if (buttonA3 == true){
    Serial.write(3);
    LastButtonPressed = 3;
  }

  if (buttonA4 == true){
    Serial.write(4);
    LastButtonPressed = 4;
  }

  switch (LastButtonPressed) {
    case 0: //RESET
        digitalWrite(RESET_BUTTON_LED_PIN, HIGH);
        delay(2000);
        digitalWrite(LYR1_BUTTON_LED_PIN, HIGH);
        digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR1_BUTTON_LED_PIN, HIGH);
        delay(20);
        digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR1_BUTTON_LED_PIN, HIGH);
        digitalWrite(RESET_BUTTON_LED_PIN, LOW);
        delay(2000);
        LastButtonPressed = -1;
        break;
    case 1:
        digitalWrite(RESET_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR1_BUTTON_LED_PIN, HIGH);
        delay(2000);
        digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR1_BUTTON_LED_PIN, HIGH);
        delay(20);
        digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR1_BUTTON_LED_PIN, HIGH);
        digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
        
        LastButtonPressed = -1;
        break;
    case 2:
        digitalWrite(RESET_BUTTON_LED_PIN, LOW);        
        digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR2_BUTTON_LED_PIN, HIGH);
        delay(2000);
        digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR2_BUTTON_LED_PIN, HIGH);
        delay(20);
        digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR2_BUTTON_LED_PIN, HIGH);        
        digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
        delay(2000);
        LastButtonPressed = -1;
        break;
    case 3:
        digitalWrite(RESET_BUTTON_LED_PIN, LOW);    
        digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR3_BUTTON_LED_PIN, HIGH);
        delay(2000);
        digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR3_BUTTON_LED_PIN, HIGH);
        delay(20);
        digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR3_BUTTON_LED_PIN, HIGH);
        digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
        delay(2000);        
        LastButtonPressed = -1;
        break;
    case 4:
        digitalWrite(RESET_BUTTON_LED_PIN, LOW);    
        digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
        digitalWrite(LYR4_BUTTON_LED_PIN, HIGH);
        delay(2000);
        digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR4_BUTTON_LED_PIN, HIGH);
        delay(20);
        digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
        delay(20);
        digitalWrite(LYR4_BUTTON_LED_PIN, HIGH);       
        delay(2000);
        LastButtonPressed = -1;
        break;
  }

  digitalWrite(RESET_BUTTON_LED_PIN, HIGH);
  delay(10);
  digitalWrite(RESET_BUTTON_LED_PIN, LOW);
  delay(20);
}

/*
void changeButton() {

  stayOn = false;

  analogWrite(BUTTON_LED_PIN, 255);
  blinking = 70;
  buttonDir = 10;

  bool state = digitalRead(BUTTON_PIN);

  // Debouncing
  if (state != oldState && timeSinceLast > 20) {
    if (state == LOW) pressed();
    else released();
    pressTime = millis();
    oldState = state;
  }

  changed = false;
}

void buttonLedUpdate() {

  if (!stayOn) {

    if (blinking > 0) {
      buttonLight = 255 * ((blinking / 10) % 2);
      blinking = blinking -1;
    } else {
      // Sine breathing
      buttonDir += 0.05;
      if (buttonDir >= 6.283) buttonDir = 0;
      buttonLight = sin(buttonDir) * 127.5 + 127.5;
    }

    if (!digitalRead(BUTTON_PIN)) buttonLight = 255;

    analogWrite(BUTTON_LED_PIN, buttonLight);
  }
}*/
