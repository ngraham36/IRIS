#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

// Pins that worked for your ESP32-C6 Scanner
#define I2C_SDA 6
#define I2C_SCL 7
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

//The AR/VR stabalized rotation vector is best for our purposes (AR headset) 
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000; // setting the update rate to 5000 we can change this if necessary

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports...");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable report!");
  }
}

void setup(void) {
  Serial.begin(115200);
  // Give the Serial Monitor a moment to connect
  delay(2000); 

  Serial.println("--- ESP32-C6 BNO08x YPR Test ---");

  // opens pins 6 (SDA) and 7 (SCL)
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(500); // wait for sensor to be ready

  // Initialize with the specific address (0x4A)
  if (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("Failed to find BNO08x chip at 0x4A");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);
  Serial.println("Reading events...");
}

//Converts the quaternions (4D) that the BNO085 returns (r, i, j, k) into Euler (roll, pitch, yaw)
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    //take the square of each 4d value
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);
    //converts the 4d values into roll/pitvh/yaw using corresponding formulas
    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)); //atan, asin, return values in radians
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));  
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
    // convert the output from radians into into degrees 
    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void loop() {
  // WILL TRIGGER IF WIRE MOVES OR POWER DIPS (reboots the sensor)
  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset! ");
    setReports(reportType, reportIntervalUs);
  }
  
  //reports new coordinates when new movement is detected
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == reportType) {
      // Use the appropriate union member based on the report type
      quaternionToEuler(sensorValue.un.arvrStabilizedRV.real, 
                       sensorValue.un.arvrStabilizedRV.i, 
                       sensorValue.un.arvrStabilizedRV.j, 
                       sensorValue.un.arvrStabilizedRV.k, &ypr, true);

      static long last = 0;
      long now = micros();
      // prints out the data in format: Time: , Yaw: , Pitch: , and Roll:
      Serial.print("Time: ");   Serial.print(now - last);     Serial.print("\t");
      //T = time between updates
      last = now;
      Serial.print("Yaw: ");    Serial.print(ypr.yaw, 2);     Serial.print("\t");
      Serial.print("Pitch: ");  Serial.print(ypr.pitch, 2);   Serial.print("\t");
      Serial.print("Roll: ");   Serial.println(ypr.roll, 2);
    }
  }
}
