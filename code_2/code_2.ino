#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

//SD things
SdFat32 sd;
File32 file;
File32 root;
File32 entry;
char flightDatName[32];

int8_t packet_number = 0;
unsigned long currentTime;
const byte len_packet = 22;
uint16_t offset;

volatile bool should_close = false;

const byte CS_pin = 10;

bool started_writing;
bool done_writing;

//gyro data
int pid_error_x; int p_x; int i_x; int d_x; 
int pid_error_y; int p_y; int i_y; int d_y;

// ✅ CHANGED: 512 → 256
byte databuffer[256];

// packets per 256-byte block (11 * 22 = 242 bytes)
const byte packets_per_block = 11;

//Radio things
int mode_pin = 2;
int arm_pin = 3;
unsigned long pulse_start_M; 
unsigned long pulse_start_A;
int16_t pulse_width_M;
int16_t pulse_width_A;

bool Mode;
bool Arm;

//Servo things
Servo servo_x;
Servo servo_y;
int servo_pin_x = 5;
int servo_pin_y = 6;

//Gyro things
MPU6050 sensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float dt;
unsigned long lastTime = 0;

// constants (flash storage)
const float Kpx = 0.07;
const float Kix = 0.9;
const float Kdx = 0.01;
const float Kpy = 0.1;
const float Kiy = 0.01;
const float Kdy = 0.006;
const float alpha = 0.96;
const float accel_alpha = 0.1;

float x = 0.0;
float y = 0.0;

float setpoint_x;
float setpoint_y;

float error_x[2];
float error_y[2];

float Px, Py;
float Ix, Iy;
float Dx, Dy;

float commanded_x, commanded_y;
float offset_x, offset_y;

float accelx_filtered = 0;
float accely_filtered = 0;

void mode() {
  if (digitalRead(mode_pin) == 0) {
    pulse_width_M = micros() - pulse_start_M;
    if (pulse_width_M < 2100 && pulse_width_M > 1600) {
      Mode = 1;
      Ix = 0; Iy = 0;
    }
    else if (pulse_width_M < 1400 && pulse_width_M > 900) {
      Mode = 0;
    }
  }
  else {
    pulse_start_M = micros();
  }
}

void arm() {
  if (digitalRead(arm_pin) == 0) {
    pulse_width_A = micros() - pulse_start_A;

    if (pulse_width_A < 2100 && pulse_width_A > 1600) {
      Arm = 1;
      done_writing = 0; //I added this so that if we wanted to have it so that if you switch arm mode back on, it will carry-on writing data
    }
    else if (pulse_width_A < 1400 && pulse_width_A > 900) {
      Arm = 0;
      done_writing = 1;
    }
  }
  else {
    pulse_start_A = micros();
  }
}

int degToUs(float deg) {
  return (int)(544 + (deg / 180.0) * (2400 - 544));
}

void setup() {
  Arm = 1;
  servo_x.attach(servo_pin_x);
  servo_y.attach(servo_pin_y);
  Wire.begin();
  sensor.initialize();
  lastTime = micros();

  Serial.begin(9600);

  pulse_start_A = 0;
  Arm = 0;
  done_writing = 0;

  error_x[0] = 0;
  error_y[0] = 0;

  commanded_x = 90;
  commanded_y = 90;

  offset_x = 86.5;
  offset_y = 83.6;

  setpoint_x = 0;
  setpoint_y = 0;

  pulse_start_M = 0;
  Mode = 1;

  attachInterrupt(digitalPinToInterrupt(mode_pin), mode, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(arm_pin), arm, CHANGE);

  Serial.println(F("\nInitializing SD card..."));

  if (!sd.begin(CS_pin, SPI_FULL_SPEED)) {
    sd.initErrorHalt("sd:");
  }

  Serial.println(F("SD card initialized."));

  int index = 0;

  root.open("/");

  while (true) {
    entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDir()) {
      index++;
    }
    entry.close();
  }

  sprintf(flightDatName, "test%d.bin", index);

  file = sd.open(flightDatName, O_CREAT | O_WRITE);

  if (!file) {
    Serial.println("File open failed!");
    while (1);
  }
}

void loop() {

  pid_loop();

  // ✅ UPDATED: 22 → 11 packets
  if (Arm == 1) && (done_writing == 1) {
    done_writing = 0;
  }

  if ((Arm == 1) && (done_writing == 0)) {

    if (packet_number < packets_per_block) {
      write_packet();
      packet_number++;
    }
    else {
      write_packet();

      // ✅ FIXED: padding for 256 bytes
      for (int i = packets_per_block * len_packet; i < 256; i++) {
        databuffer[i] = 0;
      }

      if (file.write(databuffer, 256) != 256) {
        sd.errorHalt("write failed");
      }

      file.sync();

      packet_number = 0;
    }
  }

  if (done_writing == 1) && (!file.isOpen()){
    if (packet_number > 0) {
      file.write(databuffer, 256);
      file.sync();
    }

    file.close();
    Serial.println(F("Done writing."));
  }
  //if (Arm == 0) delay(10); //Adds a delay if Arm == 0 so that the gimbal doesn't lose control
}

void write_packet() {
  offset = len_packet * packet_number;

  currentTime = micros();

  databuffer[0 + offset] = currentTime >> 24;
  databuffer[1 + offset] = currentTime >> 16;
  databuffer[2 + offset] = currentTime >> 8;
  databuffer[3 + offset] = currentTime;

  pid_error_x = error_x[0];

  databuffer[4 + offset] = pid_error_x >> 8;
  databuffer[5 + offset] = pid_error_x;

  databuffer[6 + offset] = p_x >> 8;
  databuffer[7 + offset] = p_x;

  databuffer[8 + offset] = i_x >> 8;
  databuffer[9 + offset] = i_x;

  databuffer[10 + offset] = d_x >> 8;
  databuffer[11 + offset] = d_x;

  databuffer[12 + offset] = pid_error_y >> 8;
  databuffer[13 + offset] = pid_error_y;

  databuffer[14 + offset] = p_y >> 8;
  databuffer[15 + offset] = p_y;

  databuffer[16 + offset] = i_y >> 8;
  databuffer[17 + offset] = i_y;

  databuffer[18 + offset] = d_y >> 8;
  databuffer[19 + offset] = d_y;

  databuffer[20 + offset] = Arm;
  databuffer[21 + offset] = Mode;
  Serial.println(pid_error_x);
}

void pid_loop() {
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  currentTime = micros();
  dt = (currentTime - lastTime) * 1e-6;

  float gyroRate = gx / 131.0;
  float gyrox = x + gyroRate * dt;

  float gyroRate2 = gy / 131.0;
  float gyroy = y + gyroRate2 * dt;

  float accelx = atan2(ay, az) * 180 / PI;
  float accely = atan2(ax, az) * 180 / PI;

  accelx_filtered = accelx * accel_alpha + accelx_filtered * (1.0 - accel_alpha);
  accely_filtered = accely * accel_alpha + accely_filtered * (1.0 - accel_alpha);

  x = alpha * gyrox + (1.0 - alpha) * accelx_filtered;
  y = alpha * gyroy - (1.0 - alpha) * accely_filtered;

  error_x[1] = setpoint_x - x;
  error_y[1] = setpoint_y - y;

  if (Mode == 1) {
    Px = error_x[1] * Kpx;
    Py = error_y[1] * Kpy;

    Ix += 0.5 * (error_x[0] + error_x[1]) * dt * Kix;
    Iy += 0.5 * (error_y[0] + error_y[1]) * dt * Kiy;

    Dx = (error_x[1] - error_x[0]) / dt * Kdx;
    Dy = (error_y[1] - error_y[0]) / dt * Kdy;

    commanded_x -= (Px + Ix + Dx);
    commanded_y -= (Py + Iy + Dy);

    commanded_x = constrain(commanded_x, 0, 180);
    commanded_y = constrain(commanded_y, 0, 180);
  }
  else {
    commanded_x = offset_x;
    commanded_y = offset_y;
  }

  servo_x.writeMicroseconds(degToUs(commanded_x));
  servo_y.writeMicroseconds(degToUs(commanded_y));

  lastTime = currentTime;
  error_x[0] = error_x[1];
  error_y[0] = error_y[1];

}
