#include<Wire.h>
#include<MPU6050.h>
#include<Servo.h>

Servo servo_x;
Servo servo_y;
int servo_pin_x = 5;
int servo_pin_y = 6;
MPU6050 sensor;  //SDA into A4, SCL into A5

int16_t ax, ay, az;
int16_t gx, gy, gz;

float second = 1000000.0; // one second in us
float dt;
unsigned long currentTime;

float x = 0.0; //absolute angle x from IMU
float y = 0.0; //absolute angle y from IMU
unsigned long lastTime = 0;
float Kpx = 0.03; //x axis coefficients
float Kix = 0.006;
float Kdx = 0.000;
float Kpy = 0.03; //y axis coefficients
float Kiy = 0.006;
float Kdy = 0.0;
float alpha = 0.96; // Complementary filter constant

float setpoint_x;
float setpoint_y;

float error_x[2]; //[0] for last iteration, [1] for current iteration
float error_y[2];

float Px, Py; //PID terms for x and y axes
float Ix, Iy;
float Dx, Dy;

float commanded_x, commanded_y;
float offset_x, offset_y;

int16_t pulse_width_M; int16_t pulse_width_A;
int mode_pin = 2; int arm_pin = 3;
unsigned long pulse_start_M; unsigned long pulse_start_A;
bool Mode; //1 = Stabilised; 0 = Locked

float accelx_filtered = 0; //low pass filter
float accely_filtered = 0;
float accel_alpha = 0.1; // low-pass weight — tune this (smaller = smoother)

void mode() {
  if (digitalRead(mode_pin) == 0) {// Mode switch changes mode
    pulse_width_M = micros() - pulse_start_M;
    if (pulse_width_M < 2100 & pulse_width_M > 1600) {
      //Serial.println("HIGH");
      Mode = 1;
    }
    else if (pulse_width_M < 1400 && pulse_width_M > 900) {
      //Serial.println("LOW");
      Mode = 0;
    }
  }
  else {
    pulse_start_M = micros();
  }
}

void arm() {
  if (digitalRead(arm_pin) == 0) {// Arm switch changes setpoint
    pulse_width_A = micros() - pulse_start_A;
    if (pulse_width_A < 2100 & pulse_width_A > 1600) {
      //Serial.println("HIGH");
      setpoint_x = -45;
      setpoint_y = -45;
    }
    else if (pulse_width_A < 1400 && pulse_width_A > 900) {
      //Serial.println("LOW");
      setpoint_x = 0;
      setpoint_y = 0;
    }
  }
  else {
    pulse_start_A = micros();
  }
}

void setup() {
  servo_x.attach(servo_pin_x);
  servo_y.attach(servo_pin_y);
  Wire.begin();
  Serial.begin(115200);
  sensor.initialize();
  lastTime = micros();

  error_x[0] = 0; //initial set of values
  error_y[0] = 0;
  commanded_x = 90;
  commanded_y = 90;
  offset_x = 86.5; offset_y = 83.6; //Servo angles for straight and level (locked mode)

  setpoint_x = 0;
  setpoint_y = 0;

  pulse_start_M = 0; pulse_start_A = 0;
  attachInterrupt(digitalPinToInterrupt(mode_pin), mode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(arm_pin), arm, CHANGE);

  Mode = 1;
}

void loop() {
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Time difference
  currentTime = micros();
  dt = float(currentTime - lastTime) / second;
  
  // Gyro contribution (integration)
  float gyroRate = gx / 131.0;
  float gyrox = x + gyroRate * dt;
  float gyroRate2 = gy / 131.0;
  float gyroy = y + gyroRate2 * dt;
  
  // Accelerometer contribution (tilt angle from gravity)
  float accelx = atan2(ay, az) * 180 / PI;
  float accely = atan2(ax, az) * 180 / PI;
  
  // Low-pass filter
  accelx_filtered = accel_alpha * accelx + (1.0 - accel_alpha) * accelx_filtered;
  accely_filtered = accel_alpha * accely + (1.0 - accel_alpha) * accely_filtered;

  x = alpha * gyrox + (1.0 - alpha) * accelx_filtered;
  y = alpha * gyroy - (1.0 - alpha) * accely_filtered;

  error_x[1] = setpoint_x - x;
  error_y[1] = setpoint_y - y;

  if (Mode == 1) { // if in stabilising mode
    Px = error_x[1] * Kpx; //proportional terms
    Py = error_y[1] * Kpy;

    Ix += 0.5 * (error_x[0] + error_x[1]) * dt * Kix; //integral terms
    Iy += 0.5 * (error_y[0] + error_y[1]) * dt * Kiy;

    Ix = constrain(Ix, -30, 30); //prevent integral windup if gimbal is bocked
    Iy = constrain(Iy, -30, 30);

    Dx = (error_x[1] - error_x[0]) / dt * Kdx; //derivative terms
    Dy = (error_y[1] - error_y[0]) / dt * Kdy;

    commanded_x -= (Px + Ix + Dx); //calculated PID error
    commanded_y -= (Py + Iy + Dy);

    commanded_x = constrain(commanded_x, 0, 180); //ensure values do not go beyond servo's range and change direction based on IMU orientation
    commanded_y = constrain(commanded_y, 0, 180);
  }
  else { // if in locked mode
    commanded_x = offset_x; commanded_y = offset_y;
  }
  servo_x.write(commanded_x);
  servo_y.write(commanded_y);
  Serial.print(">Setpoint_x:"); Serial.print(setpoint_x,4); // following Serial Plotter syntax, eg: >Error:0.0342,Offset:234\r\n
  Serial.print(",Error_x:"); Serial.print(error_x[1],4); 
  Serial.print(",Gyro_x:"); Serial.print(x,4);
  Serial.print(",P_x:"); Serial.print(Px,6);
  Serial.print(",I_x:"); Serial.print(Ix,6);
  Serial.print(",D_x:"); Serial.print(Dx,6);
  //Serial.print(",Setpoint_y:"); Serial.print(setpoint_y);
  //Serial.print(",Error_y:"); Serial.print(error_y[1]); 
  //Serial.print(",Gyro_y:"); Serial.print(y);
  Serial.println("\r\n");
  
  lastTime = currentTime; //setup for next iteration
  error_x[0] = error_x[1];
  error_y[0] = error_y[1];
  delay(0);
}