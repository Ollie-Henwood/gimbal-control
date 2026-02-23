/*Flight mode switcher*/

int mode_pin = 2; //connected to AUX1 on RX
unsigned long duration;

void setup() {
  Serial.begin(9600);
  pinMode(mode_pin, INPUT);
}

void loop() {
  duration = pulseIn(mode_pin, HIGH);
  Serial.println(duration);
  delay(15); //because PWM frequency is 50 Hz
}