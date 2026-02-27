#include<Wire.h>
#include<MPU6050.h>
#include<Servo.h>

Servo sg90;
Servo sg91;
int servo_pin = 5;
int servo_pin2 = 6;
MPU6050 sensor;  //SDA into A4, SCL into A5

int16_t ax, ay, az;
int16_t gx, gy, gz;

float microsecond = 1000000.0;
float dt;
unsigned long currentTime;

float x = 0.0; //absolute angle x from IMU
float y = 0.0; //absolute angle y from IMU
unsigned long lastTime = 0;
float Kp = 0.008;
float Ki = 0.0;
float Kd = 0.0;
float alpha = 0.96; // Complementary filter constant

float setpoint_x;
float setpoint_y;

float error_x[2]; //[0] for last iteration, [1] for current iteration
float error_y[2];

float Px, Py; //PID terms for x and y axes
float Ix, Iy;
float Dx, Dy;

float offset_x, offset_y;

float temp_x; float temp_y;

int pin_elevator = 2; //connected to ELE on RX
int pin_aileron = 3; //connected to AIL on RX
unsigned long pulse_start_x, pulse_start_y;
volatile byte state = LOW;
int16_t pulse_width_x;
int16_t pulse_width_y;

float rc_input_x[100];
float rc_input_y[100];

void do_x() {
  if (digitalRead(pin_elevator) == 0) {//means pulse changed high->low
    pulse_width_x = micros() - pulse_start_x;
    if (pulse_width_x>1000 & pulse_width_x<2000){
      rc_input_x[0] = float(pulse_width_x - 1500)/500*90; //mapping the 1000-2000us pulse width to -90-90 degrees
    }
  }
  else {
    pulse_start_x = micros();
  }
}
void do_y() {
  if (digitalRead(pin_aileron) == 0) {//means pulse changed high->low
    pulse_width_y = micros() - pulse_start_y;
    if (pulse_width_x>1000 & pulse_width_x<2000){
      rc_input_y[0] = float(pulse_width_x - 1500)/500*90; //mapping the 1000-2000us pulse width to -90-90 degrees
    }
  }
  else {
    pulse_start_y = micros();
  }
}

void setup() {
  sg90.attach(servo_pin);
  sg91.attach(servo_pin2);
  Wire.begin();
  Serial.begin(9600);
  sensor.initialize();
  lastTime = micros();

  error_x[0] = 0; //initial set of values
  error_y[0] = 0;
  offset_x = 90;
  offset_y = 90;
  setpoint_x = 0;
  setpoint_y = 0;

  pinMode(pin_aileron, INPUT); pinMode(pin_elevator, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_elevator), do_x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_aileron), do_y, CHANGE); //Usable pins for interrupts are 2 and 3

  for (byte i = 0; i<100; i++) {
    rc_input_x[i] = 0.0;
    rc_input_y[i] = 0.0;
  }
}

void loop() {
  temp_x = 0;
  temp_y = 0;

  for (byte i = 0; i<100; i++) { //do averaging
    temp_x += rc_input_x[i];
    temp_y += rc_input_y[i];
  }
  setpoint_x = round(temp_x/1000)*10; //truncate to nearest 10 degrees
  setpoint_y = temp_y/100; //modify setpoint to be equal to RC channel

  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Time difference
  currentTime = micros();
  dt = float(currentTime - lastTime) / microsecond;
  
  // Gyro contribution (integration)
  float gyroRate = gx / 131.0;
  float gyrox = x + gyroRate * dt;
  float gyroRate2 = gy / 131.0;
  float gyroy = y + gyroRate2 * dt;
  
  // Accelerometer contribution (tilt angle from gravity)
  float accelx = atan2(ay, az) * 180 / PI;
  float accely = atan2(ax, az) * 180 / PI;
  
  // Complementary filter
  x = alpha * gyrox + (1.0 - alpha) * accelx; //calculated angle x
  y = alpha * gyroy - (1.0 - alpha) * accely; //calculated angle y

  error_x[1] = setpoint_x - x;
  error_y[1] = setpoint_y - y;

  Px = error_x[1] * Kp; //proportional terms
  Py = error_y[1] * Kp;

  Ix = 0.5 * (error_x[0] + error_x[1]) * dt * Ki; //integral terms
  Iy = 0.5 * (error_y[0] + error_y[1]) * dt * Ki;

  Dx = (error_x[1] - error_x[0]) / dt * Kd; //derivative terms
  Dy = (error_y[1] - error_y[0]) / dt * Kd;

  offset_x -= Px + Ix + Dx; //calculated PID error + 90 degree offset (keep servos in working range of 0->180 deg)
  offset_y -= Py + Iy + Dy;

  //offset_x = constrain(offset_x, 0, 180); //ensure values do not go beyond servo's range and change direction based on IMU orientation
  offset_y = constrain(offset_y, 0, 180);
  
  sg90.write(offset_x);
  //sg91.write(offset_y);
  Serial.print(">Setpoint:"); Serial.print(setpoint_x);
  Serial.print(",Gyro:"); Serial.print(x);
  Serial.print(",Error:"); Serial.print(error_x[1]); // following Serial Plotter syntax, eg: >Error:0.0342,Offset:234\r\n
  Serial.print(",Offset:"); Serial.print(offset_x); Serial.println("\r\n");
  
  
  lastTime = currentTime; //setup for next iteration
  error_x[0] = error_x[1];
  error_y[0] = error_y[1];

  for (byte i = 99; i >=1 ; i--) { //shifting arrays
    rc_input_x[i] = rc_input_x[i-1];
    rc_input_y[i] = rc_input_y[i-1];
  }
  delay(10);
}