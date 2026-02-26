/*Flight mode switcher*/

int mode_pin = 2; //connected to AUX1 on RX
unsigned long flight_time;
volatile byte state = LOW;
uint16_t pulse_width;

void setup() {
  Serial.begin(9600);
  pinMode(mode_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(mode_pin), change, CHANGE); //Usable pins for interrupts are 2 and 3
}

void loop() {
  /*Using the interruption method*/
  
}

void change() {
  if (digitalRead(mode_pin) == 0) {
    pulse_width = micros() - flight_time;
    if (pulse_width < 2100 & pulse_width > 1600) {
      Serial.println("HIGH");
    }
    else if (pulse_width < 1400 & pulse_width > 900) {
      Serial.println("LOW");
    }
  }

  else {
    flight_time = micros();
  }
}