/*Flight mode switcher*/

int mode_pin = 2; //connected to AUX1 on RX
unsigned long flight_time;
volatile byte state = LOW;
uint16_t pulse_width;

void setup() {
  Serial.begin(9600);
  pinMode(mode_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(mode_pin), rise, RISING);
  attachInterrupt(digitalPinToInterrupt(mode_pin), fall, FALLING); //Usable pins for interrupts are 2 and 3
}

void loop() {
  /*Using the interruption method*/
  
}

void rise() {
  flight_time = micros();
}
void fall() {
  pulse_width = micros() - flight_time;
  if (pulse_width < 2100 & pulse_width > 1600) {
    Serial.println("HIGH");
  }
  else if (pulse_width < 1400 & pulse_width > 900) {
    Serial.println("LOW");
  }
}