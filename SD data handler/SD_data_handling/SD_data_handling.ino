/*SD card data handling*/
#include<SPI.h>
#include<SD.h>

void setup() {

  SD.begin(10);
  flight_Data = SD.open("Flight_Record.txt", FILE_WRITE);

}

void loop() {
  /* Data that we wan to record:
     Linear/angular acceleration, velocity
  */

}