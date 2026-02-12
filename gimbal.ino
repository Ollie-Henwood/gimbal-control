#include<Wire.h>
#include<MPU6050.h>
#include<Servo.h>

Servo sg90; //SCL into A5, SDA into A4
int servo_pin = 9; //Servo connected to D9
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
delay(1000);
Serial.println("Taking Values from the sensor");
delay(1000);
}

void loop() {
sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
gy = map(gy, -17000, 17000, 0, 180);
Serial.println(gy);
sg90.write(gy);
delay(2);
}