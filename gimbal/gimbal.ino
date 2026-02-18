#include<Wire.h>
#include<MPU6050.h>
#include<Servo.h>

Servo sg90;
Servo sg91;
int servo_pin = 9;
int servo_pin2 = 10;
MPU6050 sensor;  //SDA into A4, SCL into A5

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
  
  sg90.write((int)x+90);
  sg91.write((int)y+90);
  Serial.print("Angle-x: "); Serial.print(x);
  Serial.print("   Angle-y: "); Serial.println(y);
  delay(10);
}



#include "Kalman.h"

Kalman::Kalman() {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset the angle
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getAngle(float newAngle, float newRate, float dt) {

    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float Kalman::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };