#include <SPI.h>
#include <SdFat.h> //using SdFat library as it is faster to read and write
#include<Wire.h>
#include<MPU6050.h>
#include<Servo.h>

/*Wiring data:
CLK/SCK: Pin 13
DO/MISO: Pin 12
DI/MOSI: Pin 11
CS: Pin 10 (or any digital pin)*/

/*Things to add:
File folder that stores all previous flights
Folder that just stores current flight data
Assign each byte to data*/

//SD things
SdFat32 sd;
File32 file;
File32 root;
File32 entry;
char flightDatName[11];

int8_t packet_number = 0;
unsigned long currentTime;
const byte len_packet = 22;
uint16_t offset;

const byte CS_pin = 10;

//gyro data
int pid_error_x; int p_x; int i_x; int d_x; 
int pid_error_y; int p_y; int i_y; int d_y;

byte databuffer[256];

// packets per 256-byte block (11 * 22 = 242 bytes)
const byte packets_per_block = 11;

//Radio things
const byte mode_pin = 2; //connected to AUX1 on RX
const byte arm_pin = 3; //connected to GEAR on RX
unsigned long pulse_start_M; unsigned long pulse_start_A;
int16_t pulse_width_M;
int16_t pulse_width_A;

bool Mode; //1 = Stabilised; 0 = Locked
bool Arm; //1 = Armed; 0 = Disarmed

bool file_isopen;

//Servo things
Servo servo_x;
Servo servo_y;
const byte servo_pin_x = 5;
const byte servo_pin_y = 6;

//Gyro things
MPU6050 sensor;  //SDA into A4, SCL into A5

int16_t ax, ay, az;
int16_t gx, gy, gz;

float dt;
unsigned long lastTime = 0;

const float Kpx = 0.04;
const float Kix = 0.02;
const float Kdx = 0.0007;
const float Kpy = 0.04;
const float Kiy = 0.02;
const float Kdy = 0.0007;
const float alpha = 0.96;
const float accel_alpha = 0.1;

bool is_accelerating;
float total_accel;

float x = 0.0;
float y = 0.0;

float setpoint_x;
float setpoint_y;

float error_x[2]; //[0] for last iteration, [1] for current iteration
float error_y[2];

float Px, Py; //PID terms for x and y axes
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
    }
    else if (pulse_width_A < 1400 && pulse_width_A > 900) {
      Arm = 0;
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
  Mode = 1;
  Arm = 0;
  servo_x.attach(servo_pin_x);
  servo_y.attach(servo_pin_y);
  Wire.begin();
  sensor.initialize();
  lastTime = micros();

  pulse_start_M = 0;
  pulse_start_A = 0;

  error_x[0] = 0;
  error_y[0] = 0;

  commanded_x = 90;
  commanded_y = 90;

  offset_x = 81.6;
  offset_y = 87.95;

  setpoint_x = 0;
  setpoint_y = 0;

  packet_number = 0;
  file_isopen = 0;

  attachInterrupt(digitalPinToInterrupt(mode_pin), mode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(arm_pin), arm, CHANGE);

  if (!sd.begin(CS_pin, SPI_FULL_SPEED)) {
    sd.initErrorHalt("sd:");
  }
}

void open_file() {
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
  root.close();
  sprintf(flightDatName, "log%d.bin", index);

  file = sd.open(flightDatName, O_CREAT | O_WRITE);
}

void loop() {

  pid_loop();

  //open file
  if ((Arm == 1) && (file_isopen == 0)) {
    open_file();
    file_isopen = 1;
  }

  if (file_isopen == 1) {
    do_write();
  }

  if ((Arm == 0) && (file_isopen == 1)){
    close_file();
    file_isopen = 0;
  }
  if (Arm == 0) {
    delayMicroseconds(1600); //Adds a delay if Arm == 0 so that the gimbal doesn't lose control
  }
}


void do_write() {
  if (packet_number < 10) {
    write_packet();
    packet_number++;
  }
  else {
    write_packet();

    for (int i = 242; i < 256; i++) {
      databuffer[i] = 0;
    }

    file.write(databuffer, 256);
    file.sync();

    packet_number = 0;
  }
}

void close_file() { //checks if any data is left over, writes it, and closes file
  if (packet_number > 0) {
      file.write(databuffer, 256);
      file.sync();
    }

  file.close();
}

void write_packet() {
  offset = len_packet * packet_number;

  currentTime = micros();

  databuffer[0 + offset] = currentTime >> 24;
  databuffer[1 + offset] = currentTime >> 16;
  databuffer[2 + offset] = currentTime >> 8;
  databuffer[3 + offset] = currentTime;

  pid_error_x = error_x[0]*100;
  pid_error_y = error_y[0]*100;
  p_x = Px*1000;
  i_x = Ix*1000;
  d_x = Dx*1000;
  p_y = Py*1000;
  i_y = Iy*1000;
  d_y = Dy*1000;

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
}

void pid_loop() {
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  total_accel = sqrt(ax*ax + ay*ay + az*az) / 16384.0
  is_accelerating = (total_accel < 0.8) || (total_accel > 1.2);

  currentTime = micros();
  dt = (currentTime - lastTime) * 1e-6;

  float gyroRate = gx / 131.0;
  float gyrox = x + gyroRate * dt;

  float gyroRate2 = gy / 131.0;
  float gyroy = y + gyroRate2 * dt;

  float accelx = atan2(ay, az) * 180 / PI;
  float accely = atan2(ax, az) * 180 / PI;

  if (!is_accelerating) {
    accelx_filtered = accelx * accel_alpha + accelx_filtered * (1.0 - accel_alpha);
    accely_filtered = accely * accel_alpha + accely_filtered * (1.0 - accel_alpha);

    x = alpha * gyrox + (1.0 - alpha) * accelx_filtered;
    y = alpha * gyroy - (1.0 - alpha) * accely_filtered;
  } else {
    x = gyrox;
    y = gyroy;
  }

  error_x[1] = setpoint_x - x;
  error_y[1] = setpoint_y - y;

  if (Mode == 1) {
    Px = error_x[1] * Kpx;
    Py = error_y[1] * Kpy;

    Ix += 0.5 * (error_x[0] + error_x[1]) * dt * Kix;
    Iy += 0.5 * (error_y[0] + error_y[1]) * dt * Kiy;

    Dx = (error_x[1] - error_x[0]) / dt * Kdx;
    Dx = constrain(Dx, -0.5, 0.5);
    Dy = (error_y[1] - error_y[0]) / dt * Kdy;
    Dy = constrain(Dy, -0.5, 0.5);

    commanded_x -= (Px + Ix + Dx);
    commanded_y -= (Py + Iy + Dy);

    commanded_x = constrain(commanded_x, 0, 145);
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