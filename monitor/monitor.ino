/*Flight mode switcher*/

int pin = 2;
unsigned long duration;
volatile byte state = LOW;

void setup() {
  /*Using pulseIn method*/
  //Serial.begin(9600);
  //pinMode(mode_pin, INPUT);

  /*Using the interruption method*/
  pinMode(mode_pin, INPUT_PULLUP)
  attachInterrupt(digitalPinToInterrupt(mode_pin), ISR, CHANGE); //Usable pins for interrupts are 2 and 3
}

void loop() {
  //duration = pulseIn(mode_pin, HIGH);
  //Serial.println(duration);
  delay(15); //because PWM frequency is 50 Hz
}

void ISR() {
  if (state == HIGH) {
    isLocked = True;
  }

  else () {
    isLocked = False;
  }

}