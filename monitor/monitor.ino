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
const byte len_packet = 28;
uint16_t offset;
byte blocks_to_write = 4;

const byte CS_pin = 10;

//flight channels, to be populated:
uint16_t thr = 0;
uint16_t ail = 0;
uint16_t ele = 0;
uint16_t rud = 0;

//gyro data to be populated:
int acc_x = 0;
int acc_y = 0;
int acc_z = 0;
int gyro_x = 0;
int gyro_y = 0;
int gyro_z = 0;

const char* filename = "test2.bin"; //change to read all files, then name this file +1 greater than previous

byte databuffer[512]; //Fill this before writing to SD

int mode_pin = 2; //connected to AUX1 on RX
int arm_pin = 3; //connected to GEAR on RX
unsigned long pulse_start;
volatile byte state = LOW;
int16_t pulse_width_M;
int16_t pulse_width_A;

bool Arm;
bool Mode;

void mode() {
  if (digitalRead(mode_pin) == 0) {//means pulse changed high->low
    pulse_width_M = micros() - pulse_start;
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
    pulse_start = micros();
  }
}
void arm() {
  if (digitalRead(arm_pin) == 0) {//means pulse changed high->low
    pulse_width_A = micros() - pulse_start;
    if (pulse_width_A < 2100 & pulse_width_A > 1600) {
      //Serial.println("        HIGH");
      Arm = 1;
    }
    else if (pulse_width_A < 1400 & pulse_width_A > 900) {
      //Serial.println("        LOW");
      Arm = 0;
    }
  }
  else {
    pulse_start = micros();
  }
}


void setup() {
  Serial.begin(9600);
  while (!Serial) {} // Wait for serial

  pinMode(mode_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(mode_pin), mode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(arm_pin), arm, CHANGE); //Usable pins for interrupts are 2 and 3

  Serial.println("\nInitializing SD card...");

  // Initialize SdFat or print a detailed error message and halt
  if (!sd.begin(CS_pin, SPI_HALF_SPEED)) { // Use SPI_FULL_SPEED for better performance (when soldered)
    sd.initErrorHalt("sd:");
  }
  Serial.println("SD card initialized.");

    // Open the file for writing
  if (file.open(filename, O_CREAT | O_WRITE)) {
    Serial.println("Writing to file");
  }

}

void loop() {
  while (packet_number < 18) { //databuffer is not yet full
    offset = len_packet * packet_number; //offset for packets 2, 3 ... 18

    //time first
    time = micros();
    databuffer[0 + offset] = time >> 24;
    databuffer[1 + offset] = time >> 16;
    databuffer[2 + offset] = time >> 8;
    databuffer[3 + offset] = time;

    //flight channels
    databuffer[4 + offset] = thr >> 8;
    databuffer[5 + offset] = thr;
    
    databuffer[6 + offset] = ail >> 8;
    databuffer[7 + offset] = ail;
    
    databuffer[8 + offset] = ele >> 8;
    databuffer[9 + offset] = ele;
    
    databuffer[10 + offset] = rud >> 8;
    databuffer[11 + offset] = rud;

    //gyro data
    databuffer[12 + offset] = acc_x >> 8;
    databuffer[13 + offset] = acc_x;
    
    databuffer[14 + offset] = acc_y >> 8;
    databuffer[15 + offset] = acc_y;
    
    databuffer[16 + offset] = acc_z >> 8;
    databuffer[17 + offset] = acc_z;
    
    databuffer[18 + offset] = gyro_x >> 8;
    databuffer[19 + offset] = gyro_x;

    databuffer[20 + offset] = gyro_y >> 8;
    databuffer[21 + offset] = gyro_y;
    
    databuffer[22 + offset] = gyro_z >> 8;
    databuffer[23 + offset] = gyro_z;

    //arm and mode
    databuffer[24 + offset] = Arm;
    databuffer[25 + offset] = Mode;

    //padding to 28 bytes:
    databuffer[26 + offset] = 0;
    databuffer[27 + offset] = 0;
    
    //Serial.println("iteration complete");
    packet_number ++;
    //delay(10);
  }
  //add padding and write to SD
  for (int i = 504; i > 512; i++) {
    databuffer[i] = 0;
  }

  if (file.write(databuffer, 512) != 512) { //write block to SD
    sd.errorHalt("write failed");
  }

  packet_number = 0; //reset packet number for next block
  blocks_to_write --;

  if (blocks_to_write == 0) {

    file.close(); // Close the file
    Serial.println("Done writing.");
    delay(1000000000);
  }
  
}