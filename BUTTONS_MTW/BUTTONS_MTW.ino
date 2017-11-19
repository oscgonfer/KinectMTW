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

int LastButtonPressed = 1;
int ButtonPressed = -1;
int buttonPressedFadeTime = 4;

float buttonDir = 0;
float buttonLight = 0;

void setup() {
    //LED
    pinMode(RESET_BUTTON_LED_PIN, OUTPUT);
    pinMode(LYR1_BUTTON_LED_PIN, OUTPUT);
    pinMode(LYR2_BUTTON_LED_PIN, OUTPUT);
    pinMode(LYR3_BUTTON_LED_PIN, OUTPUT);
    pinMode(LYR4_BUTTON_LED_PIN, OUTPUT);
                    
    Serial.begin(9600);
    Serial.println("All good here");    
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

  //SEND SERIAL MESSAGES
  if (buttonA0 == true){
    Serial.write("0");
    ButtonPressed = 0;
  }

  if (buttonA1 == true){
    Serial.write(1);
    ButtonPressed = 1;
  }

  if (buttonA2 == true){
    Serial.write(2);
    ButtonPressed = 2;
  }

  if (buttonA3 == true){
    Serial.write(3);
    ButtonPressed = 3;
  }

  if (buttonA4 == true){
    Serial.write(4);
    ButtonPressed = 4;
  }

  //SEQUENCE WHEN A BUTTON HAS BEEN PRESSED
  if (ButtonPressed != -1) {
    ButtonPressedSeq(ButtonPressed);
    LastButtonPressed = max(ButtonPressed,1);
    ButtonPressed = -1;
    buttonDir = 0;
  }

  //NORMAL SEQUENCE
  if (LastButtonPressed != -1) {
    FadeButton(LastButtonPressed);
  }

}

void ButtonPressedSeq (int ButtonPressed) {
  float pressTime = millis()/1000;
  float aux = millis()/1000;
  float buttonDirPressed = 0;
  float buttonLightPressed = 0;
  
  while (aux-pressTime < buttonPressedFadeTime ) {
    buttonDirPressed +=0.05;
    if (buttonDirPressed >= 6.283) buttonDirPressed = 0;
    buttonLightPressed = sin(buttonDirPressed) * 127.5 + 127.5;
    
    switch (ButtonPressed) {
    case 0: //RESET
      //ALL TO ZERO
      digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(RESET_BUTTON_LED_PIN, buttonLightPressed);
      analogWrite(LYR1_BUTTON_LED_PIN, HIGH);
      break;
    case 1:
      //ALL TO ZERO
      digitalWrite(RESET_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(LYR1_BUTTON_LED_PIN, buttonLightPressed);
      break;
    case 2:
      //ALL TO ZERO
      digitalWrite(RESET_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(LYR2_BUTTON_LED_PIN, buttonLightPressed);
      break;
    case 3:
      //ALL TO ZERO
      digitalWrite(RESET_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(LYR3_BUTTON_LED_PIN, buttonLightPressed);
      break;
    case 4:
      //ALL TO ZERO
      digitalWrite(RESET_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(LYR4_BUTTON_LED_PIN, buttonLightPressed);
      break;
    }
    aux = millis()/1000;
  }
}

void FadeButton (int LastButtonPressed) {
  // Sine breathing
  buttonDir += 0.05;
  if (buttonDir >= 6.283) buttonDir = 0;
  buttonLight = sin(buttonDir) * 127.5 + 127.5;
  digitalWrite(RESET_BUTTON_LED_PIN, LOW);
  switch (LastButtonPressed) {
    case 1:
      //ALL TO ZERO
      digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(LYR1_BUTTON_LED_PIN, buttonLight);
      break;
    case 2:
      //ALL TO ZERO
      digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(LYR2_BUTTON_LED_PIN, buttonLight);
      break;
    case 3:
      //ALL TO ZERO
      digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR4_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(LYR3_BUTTON_LED_PIN, buttonLight);
      break;
    case 4:
      //ALL TO ZERO
      digitalWrite(LYR1_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR2_BUTTON_LED_PIN, LOW);
      digitalWrite(LYR3_BUTTON_LED_PIN, LOW);
      //EXCEPT
      analogWrite(LYR4_BUTTON_LED_PIN, buttonLight);
      break;
  }
}