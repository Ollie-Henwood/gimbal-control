#include<Wire.h>
#include<MPU6050.h>
#include<Servo.h>

Servo sg90; //SCL into A5, SDA into A4
int servo_pin = 9; //Servo connected to D9
Servo sg90;
Servo sg91;
int servo_pin = 9;
int servo_pin2 = 10;
MPU6050 sensor;
int16_t ax, ay, az; //accelerometer - linear acceleration
int16_t gx, gy, gz; //gyroscope - angular acceleration

void setup(){
sg90.attach(servo_pin);
Wire.begin();
Serial.begin(9600);
Serial.println("Initializing the sensor");
sensor.initialize();
Serial.println(sensor.testConnection() ? "Successfully Connected":"Connection failed");
delay(200);
Serial.println("Taking Values from the sensor");
delay(200);

int16_t ax, ay, az;
int16_t gx, gy, gz;

float x = 0.0;
float y = 0.0;
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
gy = map(gy, -17000, 17000, 0, 180);
Serial.println(gy);
sg90.write(gy);
delay(100);
}  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Gyro contribution (integration)
  float gyroRate = gx / 131.0;
  float gyrox = x + gyroRate * dt;
  float gyroRate2 = gy / 131.0;
  float gyroy = y + gyroRate2 * dt;
  
  // Accelerometer contribution (tilt angle from gravity)
  float accelx = atan2(ay, az) * 180 / PI;
  float accely = atan2(ax, az) * 180 / PI;
  
  // Complementary filter
  x = alpha * gyrox + (1.0 - alpha) * accelx;
  y = alpha * gyroy - (1.0 - alpha) * accely;

  // Constrain and map if needed
  //x = constrain(x, 0, 180);
  //y = constrain(y, 0, 180);
  
  sg90.write((int)x + 90);
  sg91.write((int)y);
  Serial.print("Angle-x: "); Serial.print(x);
  Serial.print("   Angle-y: "); Serial.println(y);
  delay(10);
}


