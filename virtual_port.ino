#include <UM7.h>
#include <SoftwareSerial.h>
UM7 imu;
SoftwareSerial mySerial(10, 11); // RX, TX
float rollAngle,pitchAngle,yawAngle;
float rollRate,pitchRate,yawRate;
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // set the data rate for the SoftwareSerial port
  mySerial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available() > 0) {
    if (imu.encode(mySerial.read())) {  // Reads byte from buffer.  Valid packet returns true.
      if(imu.address==0x70)
      {
        pitchAngle = imu.pitch/91.02222;
        rollAngle = imu.roll/91.02222;
        yawAngle = imu.yaw/91.02222;
        rollRate = imu.roll_rate/16.0;
        pitchRate = imu.pitch_rate/16.0;
        yawRate = imu.yaw_rate/16.0;
        Serial.print("Roll: ");Serial.println(rollAngle);
        Serial.print("Pitch: ");Serial.println(pitchAngle);
        Serial.print("Yaw: ");Serial.println(yawAngle);

        Serial.print("Roll Rate: ");Serial.println(rollRate);
        Serial.print("Pitch Rate: ");Serial.println(pitchRate);
        Serial.print("Yaw Rate: ");Serial.println(yawRate);
        
        //imu.pitch);
      }
    }
  }
}


