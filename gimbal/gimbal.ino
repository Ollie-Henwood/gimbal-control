#include<Wire.h>
#include<MPU6050.h>
#include<Servo.h>

Servo sg90; //SCL into A5, SDA into A4
int servo_pin = 9; //Servo connected to D9
MPU6050 sensor;
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