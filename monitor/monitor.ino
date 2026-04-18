#include <SPI.h>
#include <SdFat.h> //using SdFat library as it is faster to read and write

/*Wiring data:
CLK/SCK: Pin 13
DO/MISO: Pin 12
DI/MOSI: Pin 11
CS: Pin 10 (or any digital pin)*/

/*Things to add:
File folder that stores all previous flights
Folder that just stores current flight data
Assign each byte to data*/

SdFat32 sd;
File32 file;

int8_t packet_number;
unsigned long time;
const byte len_packet = 22;
uint16_t offset;

const byte CS_pin = 10;

//gyro data to be populated:
int pid_error_x; int p_x; int i_x; int d_x; 
int pid_error_y; int p_y; int i_y; int d_y;

const char* filename = "test2.bin"; //change to read all files, then name this file +1 greater than previous

byte databuffer[512];//Fill this before writing to SD

int mode_pin = 2; //connected to AUX1 on RX
int arm_pin = 3; //connected to GEAR on RX
unsigned long pulse_start_M; unsigned long pulse_start_A;
volatile byte state = LOW;
int16_t pulse_width_M;
int16_t pulse_width_A;

bool Arm;
bool Mode;
bool started_writing; //set to 1 when file is first writtent to

void mode() {
  if (digitalRead(mode_pin) == 0) {//means pulse changed high->low
    pulse_width_M = micros() - pulse_start_M;
    if (pulse_width_M < 2100 & pulse_width_M > 1600) {
      //Serial.println("HIGH");
      Mode = 1;
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
      Arm = 1; //is armed
    }
    else if (pulse_width_A < 1400 && pulse_width_A > 900) {
      //Serial.println("LOW");
      Arm = 0; //is disarmed
    }
  }
  else {
    pulse_start_A = micros();
  }
}


void setup() {
//wait for arming

  Serial.begin(9600);
  //while (!Serial) {} // Wait for serial

  pulse_start_A = 0;
  Arm = 0;
  started_writing = 0;

  attachInterrupt(digitalPinToInterrupt(mode_pin), mode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(arm_pin), arm, CHANGE); //Usable pins for interrupts are 2 and 3

  Serial.println("\nInitializing SD card...");

  // Initialize SdFat or print a detailed error message and halt
  if (!sd.begin(CS_pin, SPI_HALF_SPEED)) { // Use SPI_FULL_SPEED for better performance (when soldered)
    sd.initErrorHalt("sd:");
  }
  Serial.println("SD card initialized.");

    // Open the file for writing
  file.open(filename, O_CREAT | O_WRITE);

}

void loop() {
  while (Arm == 1) {//start recording when armed

    Serial.println("Writing to file");
    started_writing = 1;

    while (packet_number < 23) { //databuffer is not yet full
      offset = len_packet * packet_number; //offset for packets 2, 3 ... 36

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
      
      //Serial.println("iteration complete");
      packet_number ++;
      //delay(10);
    }
    //add padding up to 512 bytes
    for (int i = 506; i > 512; i++) {
      databuffer[i] = 0;
    }

    if (file.write(databuffer, 512) != 512) { //write block to SD
      sd.errorHalt("write failed");
    }

    packet_number = 0; //reset packet number for next block
  }

  if ((Arm == 0) && (started_writing == 1)) {//when disarmed and SD has been written to
    file.close(); // Close the file
    Serial.println("Done writing.");
    delay(1000000000000);
  }
}