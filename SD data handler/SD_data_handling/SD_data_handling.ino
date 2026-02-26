#include <SPI.h>
#include <SdFat.h> //using SdFat library as it is faster to read and write

SdFat32 sd;
File32 file;

/*Wiring data:
CLK/SCK: Pin 13
DO/MISO: Pin 12
DI/MOSI: Pin 11
CS: Pin 10 (or any digital pin)*/

const byte CS_pin = 10;

const char* filename = "test1.bin"; //change to read all files, then name this file +1 greater than previous

byte databuffer[512]; //Fill this before writing to SD
byte packet[28]; //data packet to be filled with data to be logged


void setup() {
  for (byte i = 0; i<28; i++) { //fill packet with zeroes TESTING
    packet[i] = i;
  }

  for (byte i = 0; i<18; i++) { //loop through all packets
    for (byte j = 0; j<28; j++) { //for every byte in packet
      databuffer[28*i + j] = packet[j];
    }
  }

  for (uint16_t i = 504; i>512; i++) { //write padding at end of databuffer to make up to 512 bytes
    databuffer[i] = 0;
  }

  Serial.begin(9600);
  while (!Serial) {} // Wait for serial
  Serial.println("\nInitializing SD card...");

  // Initialize SdFat or print a detailed error message and halt
  if (!sd.begin(CS_pin, SPI_HALF_SPEED)) { // Use SPI_FULL_SPEED for better performance (when soldered)
    sd.initErrorHalt("sd:");
  }
  Serial.println("SD card initialized.");

    // Open the file for writing
  if (file.open(filename, O_CREAT | O_WRITE)) {
    Serial.println("Writing to file");

    if (file.write(databuffer, 512) != 512) { //write block to SD
      sd.errorHalt("write failed");
    }

    file.close(); // Close the file
    Serial.println("Done writing.");
  } else {
    Serial.println("Error opening test.txt for writing");
}

  // Open the file for reading
  if (file.open(filename, O_READ)) {
    Serial.println("Reading from file");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close(); // Close the file
  } else {
    Serial.println("Error opening test.txt for reading");
  }
}

void loop() {
  
}