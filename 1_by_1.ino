#include <Wire.h>
#include <ICM_20948.h>
#include "MadgwickAHRS.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// ---------------- IMU ----------------
ICM_20948_I2C imu;
Madgwick filter;
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1


// ---------------- TFT ----------------
#define TFT_CS   10
#define TFT_RST  9
#define TFT_DC   33

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ---------------- AR CONFIG ----------------
#define FOV_H  30.0
#define FOV_V  16.0

#define SCREEN_W 160
#define SCREEN_H 80

// -------- WORLD OBJECTS --------

//Initilaize anchor of the real world object
float anchorWorldX = 0.0;
float anchorWorldY = 0.0;

// Real object 2 - starting from different starting point in real world
float anchorWorldX2 = 1.0;
float anchorWorldY2 = 1.0;

// Real object 3 - starting from different starting point in real world
float anchorWorldX3 = 5.0;
float anchorWorldY3 = 5.0;

// Timing
unsigned long lastUpdate = 0;
float dt = 0.005; // 200 hz

// Smoothing
float smoothYaw = 0;
float smoothPitch = 0;
float alpha = 0.1;

// Head tracking
float headPitch = 0;
float headYaw   = 0;
float headRoll  = 0;


// ---------------- HELPERS ----------------
float wrapAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
//acc for 360 rotation hopefully
}

int worldToScreenX(float worldAngle, float headYaw){
  float delta = worldAngle - headYaw;
  //Serial.println((int)(-delta / (FOV_H / 2.0) + 1.0) * (SCREEN_W/12.0));
  return (int)((-delta / (FOV_H / 2.0) + 1.0) * (SCREEN_W/12.0));

}

int worldToScreenY(float worldAngle, float headPitch){
  float delta = worldAngle - headPitch;
  return (int)((-delta / (FOV_V / 2.0) + 1.0) * (SCREEN_H/12.0));

}

void drawTriangleCenter(int centerX, int centerY, uint16_t color) {
    tft.drawTriangle(centerX, centerY-15, centerX-15, centerY+15, centerX+15, centerY+15, color);
}

void setup() {
  Serial.begin(115200);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000); //Setting the clock frequency

  imu.enableDebugging();

  bool initialized = false;
  while (!initialized)
  {
      imu.begin(WIRE_PORT, AD0_VAL);
   
      SERIAL_PORT.print(F("Initialization of the sensor returned: "));
      SERIAL_PORT.println(imu.statusString());
      if (imu.status != ICM_20948_Stat_Ok)
      {
        SERIAL_PORT.println("Trying again...");
        delay(500);
      }
      else
      {
        initialized = true;
      }
  }

  Serial.println("Madgwick AR Head Tracking");

  /*if (imu.begin(Wire, 0x68) != ICM_20948_Stat_Ok) {
    Serial.println("Trying 0x69...");
    if (imu.begin(Wire, 0x69) != ICM_20948_Stat_Ok) {
      Serial.println("IMU not detected. Check Qwiic cable.");
      while (1);
    }
  }*/

  Serial.println("IMU connected via Qwiic!");
  filter.begin(20); // madgwick gain tweaked for smoothness

  // TFT setup
  tft.setSPISpeed(40000000);
  tft.initR(INITR_MINI160x80);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println(F("Tft Initialized"));

  Serial.println("AR System Ready");
}

void loop() {
  tft.invertDisplay(true);

  //Edit - trying to use real-delta timing
  static unsigned long lastUpdate = 0;
  const unsigned long interval = 5000;

  unsigned long microNow = micros();

  // Timing of redraw and data polling; around 200 Hz
  if (microNow-lastUpdate >= interval){
    lastUpdate += microNow;

    imu.getAGMT();

    //printScaledAGMT(&imu); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    getReadings(&imu);
    //delay(100);

  }

  //Wrapping angles
  headYaw = wrapAngle(headYaw);
  headPitch = wrapAngle(headPitch);

  // Smoothing
  float deltaYaw = wrapAngle(headYaw-smoothYaw);
  smoothYaw += alpha * deltaYaw;

  float deltaPitch = wrapAngle(headPitch-smoothPitch);
  smoothPitch += alpha * deltaPitch;


  // -------- MAP TO SCREEN --------
  // all three focus on both yaw and pitch changes instead of the alternating in the tft script
  int cx  = worldToScreenX(anchorWorldX, smoothYaw);
  int cy  = worldToScreenY(anchorWorldY, smoothPitch);

  int cx2 = worldToScreenX(anchorWorldX2, smoothYaw);
  int cy2 = worldToScreenY(anchorWorldY2, smoothPitch);

  int cx3 = worldToScreenX(anchorWorldX3, smoothYaw);
  int cy3 = worldToScreenY(anchorWorldY3, smoothPitch);

  // -------- DRAW --------
  tft.fillScreen(ST77XX_BLACK);

  drawTriangleCenter(cx,  cy,  ST77XX_CYAN);
  drawTriangleCenter(cx2, cy2, ST77XX_RED);
  drawTriangleCenter(cx3, cy3, ST77XX_WHITE);
}



void getReadings(ICM_20948_I2C *sensor){
  float ax = sensor->accX();
  float ay = sensor->accY();
  float az = sensor->accZ();
  float gx = sensor->gyrX(); 
  float gy = sensor->gyrY();
  float gz = sensor->gyrZ();
  float mx = sensor->magX();
  float my = sensor->magY();
  float mz = sensor->magZ();

  // madgwick filter
  /*filter.update(
  gx,
  gy,
  gz,
  ax, ay, az,
  mx, my, mz);*/

  //euler angles in degrees
  // FIND OUT HOW TO GET PITCH ROLL AND YAW WITHOUT FILTER
  float headPitch = filter.getPitch();
  float headYaw   = filter.getYaw();
  float headRoll  = filter.getRoll();

  Serial.print("Pitch: "); Serial.print(headPitch);
  Serial.print(" | Roll: "); Serial.print(headRoll);
  Serial.print(" | Yaw: "); Serial.println(headYaw);
}

