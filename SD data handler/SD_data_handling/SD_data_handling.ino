/*SD card data handling*/

/*What we want the coded to do:
  1. Open a file when flight starts
  2. Write data from other Arduino ports into the open file on the SD card
  3. When the flight is concluded, save the flight data to another file on the SD*/
#include<SPI.h>
#include<SD.h>

void setup() {
  /*Wiring data:
    CLK/SCK: Pin 13
    DO/MISO: Pin 12
    DI/MOSI: Pin 11
    CS: Pin 10 (or any digital pin)*/

  SD.begin(10);
  flight_Data = SD.open("Flight_Record.txt", FILE_WRITE);


}

void loop() {
  /* Data that we wan to record:
     Gyro data - Linear/angular acceleration, velocity
     Port data - Data from each transmitter port
  */

}