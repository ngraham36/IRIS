#include <Wire.h>
#include <ICM_20948.h>
#include "MadgwickAHRS.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// ---------------- IMU ----------------
ICM_20948_I2C imu;
Madgwick filter;


// ---------------- TFT ----------------
#define TFT_CS   10
#define TFT_RST  9
#define TFT_DC   8

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ---------------- AR CONFIG ----------------
#define FOV_H  30.0
#define FOV_V  16.0

#define SCREEN_W 160
#define SCREEN_H 80
//120

// Head tracking
//float headPitch = 0;
//float headYaw   = 0;
//float headRoll  = 0;

// Smoothing
float smoothYaw = 0;
float smoothPitch = 0;
float alpha = 0.1;

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

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  tft.setSPISpeed(40000000);

  // IMU
  Wire.begin();
  if (imu.begin(Wire, 0x68) != ICM_20948_Stat_Ok) {
    imu.begin(Wire, 0x69);
  }
  filter.begin(200);

  // TFT
  tft.initR(INITR_MINI160x80);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
Serial.println(F("Tft Initialized"));

  Serial.println("AR System Ready");
}

// ---------------- LOOP ----------------
void loop() {

   tft.invertDisplay(true);

   // Frame timer
  static unsigned long lastFrame = 0;
  unsigned  long now = millis();
 
  // -------- IMU UPDATE --------
  imu.getAGMT();

  // sensors all 9 degrees
  float ax = imu.accX();
  float ay = imu.accY();
  float az = imu.accZ();
  float gx = imu.gyrX(); 
  float gy = imu.gyrY();
  float gz = imu.gyrZ();
  float mx = imu.magX();
  float my = imu.magY();
  float mz = imu.magZ();

  // madgwick filter
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  // Skip redraw if frame hasn't reached 33 fps limit
  if (now - lastFrame < 33) return;
  lastFrame = now;

  float headPitch = filter.getPitch();
  float headYaw   = filter.getYaw();
  float headRoll  = filter.getRoll();

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

  //Initilaize anchor of the real world object
  float anchorWorldX = 0.0;
  float anchorWorldY = 0.0;

  // Real object 2 - starting from different starting point in real world
  float anchorWorldX2 = 1.0;
  float anchorWorldY2 = 1.0;

  // Real object 3 - starting from different starting point in real world
  float anchorWorldX3 = 5.0;
  float anchorWorldY3 = 5.0;

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

  // -------- DEBUG --------
  Serial.print("Yaw: "); Serial.print(headYaw);
  Serial.print(" Pitch: "); Serial.println(headPitch);

//Backwards then flip sign delta = wrapAngle(headYaw - worldAngle);
// movement too sensitive * (SCREEN_W / 2.0)
//jitter lower alpha = 0.05;

  //delay(5); // ~100 Hz
}