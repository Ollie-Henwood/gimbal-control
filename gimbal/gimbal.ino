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
float Kp = 0.15;
float Ki = 0.01;
float Kd = 0.01;
float alpha = 0.96; // Complementary filter constant

float setpoint_x = 0;
float setpoint_y = 0;

float error_x[2]; //[0] for last iteration, [1] for current iteration
float error_y[2];

float Px, Py; //PID terms for x and y axes
float Ix, Iy;
float Dx, Dy;

float offset_x, offset_y;

void setup() {
  sg90.attach(servo_pin);
  sg91.attach(servo_pin2);
  Wire.begin();
  Serial.begin(115200);
  sensor.initialize();
  lastTime = micros();

  error_x[0] = 0; //initial set of values
  error_y[0] = 0;
  offset_x = 90;
  offset_y = 90;
}

void loop() {
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

  offset_x = constrain(offset_x, 0, 180); //ensure values do not go beyond servo's range and change direction based on IMU orientation
  offset_y = constrain(offset_y, 0, 180);
  
  sg90.write(offset_x);
  sg91.write(offset_y);
  Serial.print(">Error:"); Serial.print(error_x[1]); Serial.print(","); // following Serial Plotter syntax, eg: >Error:0.0342,Offset:234\r\n
  Serial.print("Offset:"); Serial.print(offset_x); Serial.println("\r\n");
  
  lastTime = currentTime; //setup for next iteration
  error_x[0] = error_x[1];
  error_y[0] = error_y[1];
  delay(10);
}