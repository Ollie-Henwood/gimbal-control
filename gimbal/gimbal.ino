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

byte packet_number;
unsigned long time;
const byte len_packet = 22;
uint16_t offset;

const byte CS_pin = 10;

bool started_writing;// set to 1 when file is first written to
bool done_writing;// set to 1 when file is closed; no further SD actions

//gyro data to be populated:
int16_t pid_error_x; int16_t p_x; int16_t i_x; int16_t d_x; 
int16_t pid_error_y; int16_t p_y; int16_t i_y; int16_t d_y;

const char* filename = "test2.bin"; //change to read all files, then name this file +1 greater than previous

byte databuffer[512];//Fill this before writing to SD

//Radio things
byte mode_pin = 2; //connected to AUX1 on RX
byte arm_pin = 3; //connected to GEAR on RX
unsigned long pulse_start_M; unsigned long pulse_start_A;
int16_t pulse_width_M;
int16_t pulse_width_A;

bool Mode; //1 = Stabilised; 0 = Locked
bool Arm; //1 = Armed; 0 = Disarmed

//Servo things
Servo servo_x;
Servo servo_y;
byte servo_pin_x = 5;
byte servo_pin_y = 6;

//Gyro things
MPU6050 sensor;  //SDA into A4, SCL into A5

int16_t ax, ay, az;
int16_t gx, gy, gz;

float second = 1000000.0; // one second in us
float dt;
unsigned long currentTime;

float x = 0.0; //absolute angle x from IMU
float y = 0.0; //absolute angle y from IMU
unsigned long lastTime = 0;
float Kpx = 0.07; //x axis coefficients
float Kix = 0.9;
float Kdx = 0.01;
float Kpy = 0.1; //y axis coefficients
float Kiy = 0.01;
float Kdy = 0.006;
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

float accelx_filtered = 0; //low pass filter
float accely_filtered = 0;
float accel_alpha = 0.1; // low-pass weight — tune this (smaller = smoother)

void mode() {
  if (digitalRead(mode_pin) == 0) {//means pulse changed high->low
    pulse_width_M = micros() - pulse_start_M;
    if (pulse_width_M < 2100 & pulse_width_M > 1600) {
      //Serial.println("HIGH");
      Mode = 1;
      Ix = 0; Iy = 0;  // reset integral on mode switch
    }
    else if (pulse_width_M < 1400 & pulse_width_M > 900) {
      //Serial.println("LOW");
      Mode = 0;
    }
  }
  else {
    pulse_start_M = micros();
  }
}

void arm() {
  if (digitalRead(arm_pin) == 0) {//means pulse changed high->low
    pulse_width_A = micros() - pulse_start_A;
    if (pulse_width_A < 2100 & pulse_width_A > 1600) {
      Arm = 1; //is armed; start writing to SD

      if (started_writing == 0) { //only change this once
        started_writing = 1;
      }
    }
    else if (pulse_width_A < 1400 && pulse_width_A > 900) {
      Arm = 0; //is disarmed; stop writing to SD and close file

      if ((started_writing == 1) && (done_writing == 0)) {// ensure this only happens once
        file.write(databuffer, 512); //final write

        file.close(); // Close the file
        Serial.println(F("Done writing."));

        done_writing = 1;// prevent file from being closed again
      }
    }
  }
  else {
    pulse_start_A = micros();
  }
}

int degToUs(float deg) {
  // Maps 0-180 degrees to 544-2400 microseconds (Arduino servo library standard)
  return (int)(544 + (deg / 180.0) * (2400 - 544));
}

void setup() {  
  servo_x.attach(servo_pin_x);
  servo_y.attach(servo_pin_y);
  Wire.begin();
  sensor.initialize();
  lastTime = micros();

  Serial.begin(9600);
  //while (!Serial) {} // Wait for serial

  pulse_start_A = 0;
  Arm = 0;
  started_writing = 0;

  error_x[0] = 0; //initial set of values
  error_y[0] = 0;
  commanded_x = 90;
  commanded_y = 90;
  offset_x = 86.5; offset_y = 83.6; //Servo angles for straight and level (locked mode)

  setpoint_x = 0;
  setpoint_y = 0;

  packet_number = 0;

  pulse_start_M = 0; pulse_start_A = 0;//initial values
  Mode = 1; Arm = 0;

  attachInterrupt(digitalPinToInterrupt(mode_pin), mode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(arm_pin), arm, CHANGE); //Usable pins for interrupts are 2 and 3

  Serial.println(F("\nInitializing SD card..."));

  // Initialize SdFat or print a detailed error message and halt
  if (!sd.begin(CS_pin, SPI_FULL_SPEED)) { // Use SPI_FULL_SPEED for better performance (when soldered)
    sd.initErrorHalt("sd:");
  }
  Serial.println(F("SD card initialized."));

    // Open the file for writing
  file.open(filename, O_CREAT | O_WRITE);

}

void loop() {

  pid_loop();// do a full gimbal PID loop

  if ((Arm == 1) && (done_writing == 0)) {//start recording when armed, but not when disarmed again

    if (packet_number < 22) { //databuffer is not yet full
      write_packet();
      packet_number ++;
    }
    else {// we are on packet 22 - final packet; add padding and write to SD
      //add padding up to 512 bytes
      write_packet();
      for (int i = 506; i > 512; i++) {
        databuffer[i] = 0;
      }
    
      //Serial.println(F("Writing to file"));

      if (file.write(databuffer, 512) != 512) { //write block to SD
      sd.errorHalt("write failed");
      }

      packet_number = 0; //reset packet number for next block
    }
  }
}


void write_packet() {
  offset = len_packet * packet_number; //offset for packets 2, 3 ... 36

  pid_error_x = error_x[0] * 100;
  p_x = Px * 100;
  i_x = Ix * 100;
  d_x = Dx * 100;
  pid_error_y = error_y[0] * 100;
  p_y = Py * 100;
  i_y = Iy * 100;
  d_y = Dy * 100;

  //time first
  time = micros();
  databuffer[0 + offset] = time >> 24;
  databuffer[1 + offset] = time >> 16;
  databuffer[2 + offset] = time >> 8;
  databuffer[3 + offset] = time;

  //gyro data
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

  //arm and mode
  databuffer[20 + offset] = Arm;
  databuffer[21 + offset] = Mode;
}

void pid_loop() {
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
  servo_x.writeMicroseconds(degToUs(commanded_x));
  servo_y.writeMicroseconds(degToUs(commanded_y));
  /*Serial.print(">Setpoint_x:"); Serial.print(setpoint_x,4); // following Serial Plotter syntax, eg: >Error:0.0342,Offset:234\r\n
  Serial.print(",Error_x:"); Serial.print(error_x[1],4); 
  Serial.print(",Gyro_x:"); Serial.print(x,4);
  Serial.print(",P_x:"); Serial.print(Px,6);
  Serial.print(",I_x:"); Serial.print(Ix,6);
  Serial.print(",D_x:"); Serial.print(Dx,6);
  //Serial.print(",Setpoint_y:"); Serial.print(setpoint_y);
  //Serial.print(",Error_y:"); Serial.print(error_y[1]); 
  //Serial.print(",Gyro_y:"); Serial.print(y);
  Serial.println("\r\n");*/
  
  lastTime = currentTime; //setup for next iteration
  error_x[0] = error_x[1];
  error_y[0] = error_y[1];
  //delayMicroseconds(1);
}