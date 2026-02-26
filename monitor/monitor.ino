/*Flight mode switcher*/

int mode_pin = 2; //connected to AUX1 on RX
unsigned long flight_time;
volatile byte state = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(mode_pin, INPUT);
}

void loop() {
  /*Using the interruption method*/
  pinMode(mode_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(mode_pin), ISR, CHANGE); //Usable pins for interrupts are 2 and 3

}

void ISR() {
  if (state == HIGH) {
    isLocked = true;
  }

  else () {
    isLocked = false;
  }
}