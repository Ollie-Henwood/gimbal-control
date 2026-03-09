#include <SoftwareSerial.h> 

#define MSP_RAW_IMU 102
#define MSP_ATTITUDE 108

int RX_pin = 10;
int TX_pin = 11;

struct raw_imu {
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
};

struct attitude {
  int16_t roll, pitch, yaw;
};

raw_imu imu;
attitude att;

const long telemBaud = 57600;
SoftwareSerial fcSerial(RX_pin, TX_pin);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  Serial.println("Connection with Skyline started");

  fcSerial.begin(telemBaud);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("New loop");

  if (fcSerial.available()) {
    Serial.write(fcSerial.read());
  }

  Serial.prinlt(imu.gyroX);
  /*
  //Collecting imu data
  if (requestMSP(MSP_RAW_IMU) && receiveMSP()) {
    Serial.println(imu.gyroX);
    Serial.println(imu.gyroY);
    Serial.println(imu.gyroZ);
    Serial.println(imu.accelX);
    Serial.println(imu.accelY);
    Serial.println(imu.accelZ);
  }

  //Collecting attitude data
  else if (requestMSP(MSP_ATTITUDE) && receiveMSP()) {
    Serial.println(att.pitch);
    Serial.println(att.roll);
    Serial.println(att.yaw);
    */
  }

}

bool requestMSP(uint8_t command) {
  fcSerial.write('$');
  fcSerial.write('M');
  fcSerial.write('<');
  return true;
}

bool receiveMSP() {
  // Wait for response header
  if (fcSerial.read() != '$') return false;
  if (fcSerial.read() != 'M') return false;
  if (fcSerial.read() != '>') return false;
  
  int dataSize = fcSerial.read();
  int command = fcSerial.read();
  
  // Read data based on command
  if (command == MSP_RAW_IMU && dataSize >= 18) {
    imu.accelX = fcSerial.read() | (fcSerial.read() << 8);
    imu.accelY = fcSerial.read() | (fcSerial.read() << 8);
    imu.accelZ = fcSerial.read() | (fcSerial.read() << 8);
    imu.gyroX = fcSerial.read() | (fcSerial.read() << 8);
    imu.gyroY = fcSerial.read() | (fcSerial.read() << 8);
    imu.gyroZ = fcSerial.read() | (fcSerial.read() << 8);

    for(int i=0; i<6; i++) fcSerial.read();
  }
  else if (command == MSP_ATTITUDE && dataSize >= 6) {
    att.roll = fcSerial.read() | (fcSerial.read() << 8);
    att.pitch = fcSerial.read() | (fcSerial.read() << 8);
    att.yaw = fcSerial.read() | (fcSerial.read() << 8);
  }

  fcSerial.read();
  return true;
}