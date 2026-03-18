#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// Pins that worked for your ESP32-C6 Scanner
#define I2C_SDA 6
#define I2C_SCL 7
#define BNO08X_RESET -1

// ---------------- TFT ----------------
#define TFT_CS   10
#define TFT_RST  9
#define TFT_DC   8

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ---------------- AR CONFIG ----------------
#define FOV_H  30.0
#define FOV_V  16.0
// TFT display size
#define SCREEN_W 160
#define SCREEN_H 80

// Head tracking
float headPitch = 0;
float headYaw   = 0;
float headRoll  = 0;

// Smoothing
float smoothYaw = 0;
float smoothPitch = 0;
float alpha = 0.1;


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



  // TFT Setups
  tft.initR(INITR_MINI160x80);  // Init ST7735S mini display

  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  tft.setSPISpeed(40000000);
  
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println(F("Tft Initialized"));


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



// --------------- TFT METHODS ------------------------------
int worldToScreenX(float worldAngle, float headYaw){
  float delta = worldAngle - headYaw;
  //Serial.println((int)(delta / (FOV_H / 2.0) + 1.0) * (SCREEN_W/2.0));
  return (int)((delta / (FOV_H / 2.0) + 1.0) * (SCREEN_W/2.0));

}

int worldToScreenY(float worldAngle, float headPitch){
  float delta = worldAngle - headPitch;
  return (int)((delta / (FOV_V / 2.0) + 1.0) * (SCREEN_H/2.0));

}

// Draws triangle at fixed size, centerX and centerY are anchor points that will be updated as head motion occurs
void drawTriangleCenter(int centerX, int centerY, uint16_t color) {
    tft.drawTriangle(centerX, centerY-15, centerX-15, centerY+15, centerX+15, centerY+15, color);
}

// ------------------------------------------------------------



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

  headPitch = ypr.pitch;
  headYaw = ypr.yaw;
  headRoll = ypr.roll;

  // -------- SMOOTHING --------
  smoothYaw = alpha * headYaw + (1 - alpha) * smoothYaw;
  smoothPitch = alpha * headPitch + (1 - alpha) * smoothPitch;

  // -------- WORLD OBJECTS --------

  float obj1Yaw = 0;
  float obj1Pitch = 0;

  float obj2Yaw = 15;
  float obj2Pitch = 5;

  float obj3Yaw = -10;
  float obj3Pitch = -5;

 // -------- MAP TO SCREEN --------
  // all three focus on both yaw and pitch changes instead of the alternating in the tft script
  int cx  = worldToScreenX(obj1Yaw, smoothYaw);
  int cy  = worldToScreenY(obj1Pitch, smoothPitch);

  int cx2 = worldToScreenX(obj2Yaw, smoothYaw);
  int cy2 = worldToScreenY(obj2Pitch, smoothPitch);

  int cx3 = worldToScreenX(obj3Yaw, smoothYaw);
  int cy3 = worldToScreenY(obj3Pitch, smoothPitch);

  // -------- DRAW --------
  tft.fillScreen(ST77XX_BLACK);

  drawTriangleCenter(cx,  cy,  ST77XX_CYAN);
  drawTriangleCenter(cx2, cy2, ST77XX_RED);
  drawTriangleCenter(cx3, cy3, ST77XX_WHITE);


//Backwards then flip sign delta = wrapAngle(headYaw - worldAngle);
// movement too sensitive * (SCREEN_W / 2.0)
//jitter lower alpha = 0.05;

  delay(10); // ~100 Hz






  





}