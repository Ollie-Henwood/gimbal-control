/*Flight mode switcher*/

int mode_pin = 2; //connected to AUX1 on RX
int arm_pin = 3; //connected to 
unsigned long pulse_start;
volatile byte state = LOW;
int16_t pulse_width_M;
int16_t pulse_width_A;

void setup() {
  Serial.begin(9600);
  pinMode(mode_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(mode_pin), mode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(arm_pin), arm, CHANGE); //Usable pins for interrupts are 2 and 3
}

void loop() {
  /*Using the interruption method*/
  
}

void mode() {
  if (digitalRead(mode_pin) == 0) {//means pulse changed high->low
    pulse_width_M = micros() - pulse_start;
    if (pulse_width_M < 2100 & pulse_width_M > 1600) {
      Serial.println("HIGH");
    }
    else if (pulse_width_M < 1400 & pulse_width_M > 900) {
      Serial.println("LOW");
    }
  }

  else {
    pulse_start = micros();
  }

}
void arm() {
  if (digitalRead(arm_pin) == 0) {//means pulse changed high->low
    pulse_width_A = micros() - pulse_start;
    if (pulse_width_A < 2100 & pulse_width_A > 1600) {
      Serial.println("HIGH");
    }
    else if (pulse_width_A < 1400 & pulse_width_A > 900) {
      Serial.println("LOW");
    }
  }

  else {
    pulse_start = micros();
  }
}