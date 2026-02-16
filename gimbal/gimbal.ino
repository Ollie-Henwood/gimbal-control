#include<Wire.h>
#include<MPU6050.h>
#include<Servo.h>

Servo sg90;
Servo sg91;
int servo_pin = 9;
int servo_pin2 = 10;
MPU6050 sensor;
<<<<<<< Updated upstream
int16_t ax, ay, az; //accelerometer - linear acceleration
int16_t gx, gy, gz; //gyroscope - angular acceleration
int16_t v = 0;
int angular_disp;

void setup(){
sg90.attach(servo_pin);
Wire.begin();
Serial.begin(115200);
Serial.println("Initializing the sensor");
sensor.initialize();
Serial.println(sensor.testConnection() ? "Successfully Connected":"Connection failed");
delay(200);
Serial.println("Taking Values from the sensor");
delay(200);
}

void loop() {
sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
sensor.dmpGetYawPitchRoll(ypr, &q, &gravity);
Serial.pritnln(ypr)
gy = map(gy, -32768, 32767 , 0, 180);
//Serial.println(gy);
//sg90.write(angular_disp);
delay(100);
v = v + gy * 0.1;
//Serial.println(gy);
}
=======

int16_t ax, ay, az;
int16_t gx, gy, gz;

float angle = 0.0;
float angle2 = 0.0;
unsigned long lastTime = 0;
float alpha = 0.96; // Complementary filter constant

void setup() {
  sg90.attach(servo_pin);
  sg91.attach(servo_pin2);
  Wire.begin();
  Serial.begin(9600);
  sensor.initialize();
  lastTime = millis();
}

void loop() {
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Gyro contribution (integration)
  float gyroRate = gy / 131.0;
  float gyroAngle = angle + gyroRate * dt;
  float gyroRate2 = gx / 131.0;
  float gyroAngle2 = angle2 + gyroRate2 * dt;
  
  // Accelerometer contribution (tilt angle from gravity)
  float accelAngle = atan2(ay, az) * 180 / PI;
  float accelAngle2 = atan2(ay, az) * 180 / PI;
  
  // Complementary filter
  angle = alpha * gyroAngle + (1.0 - alpha) * accelAngle;
  angle2 = alpha * gyroAngle2 + (1.0 - alpha) * accelAngle2;

  // Constrain and map if needed
  angle = constrain(angle, 0, 180);
  angle2 = constrain(angle2, 0, 180);
  
  sg90.write((int)angle);
  sg91.write((int)angle2);
  Serial.print("Angle-y: "); Serial.println(angle);
  Serial.print("Angle-x: "); Serial.println(angle2);
  delay(10);
}


>>>>>>> Stashed changes
