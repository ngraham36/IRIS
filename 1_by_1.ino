#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// ---------------- IMU ----------------
ICM_20948_I2C imu;
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

//Polling
#define IMU_INTERVALS_MS  50 


// ---------------- TFT ----------------
#define TFT_CS   10
#define TFT_RST  3
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
float anchorWorldX2 = 30.0;
float anchorWorldY2 = 20.0;

// Real object 3 - starting from different starting point in real world
float anchorWorldX3 = 50.0;
float anchorWorldY3 = 60.0;

// Head tracking
float headPitch = 0.0;
float headYaw   = 0.0;
float headRoll  = 0.0;

// ---------------- HELPERS ----------------
void readIMU(){
  static unsigned long lastIMU = 0;
  unsigned long now = millis();
  if (now - lastIMU < IMU_INTERVALS_MS) return;
  lastIMU = now;

   if (!imu.dataReady()){
    SERIAL_PORT.println("IMU not ready");
    return;
  }

  imu.getAGMT();

  float ax = imu.accX();
  float ay = imu.accY();
  float az = imu.accZ();

  headPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  headRoll  = atan2(ay, az) * 180.0 / PI;

  float mx = imu.magX();
  float my = imu.magY();

  headYaw = atan2(my, mx) * 180.0 / PI;


  //SERIAL_PORT.println();
  //SERIAL_PORT.print("Pitch: "); Serial.print(headPitch);
  //SERIAL_PORT.print(" | Roll: "); Serial.print(headRoll);
  //SERIAL_PORT.print(" | Yaw: "); Serial.println(headYaw);

  SERIAL_PORT.println();
  SERIAL_PORT.print("ax: "); Serial.print(ax);
  SERIAL_PORT.print(" | ay: "); Serial.print(ay);
  SERIAL_PORT.print(" | az: "); Serial.print(az);
  SERIAL_PORT.print(" | mx: "); Serial.print(mx);
  SERIAL_PORT.print(" | my: "); Serial.println(my);

}


int worldToScreenX(float worldAngle, float headYaw){
  float delta = worldAngle - headYaw;
   // 04/7 - This equation works better
  return (int)((-delta / (FOV_H / 2.0) + 1.0) * (SCREEN_W/12.0));

  /*SERIAL_PORT.print("WorldX: "); SERIAL_PORT.print((int)((delta / (FOV_H / 2.0) + 1.0) * (SCREEN_W / 2.0)));
  //SERIAL_PORT.println();
  //return (int)((delta / (FOV_H / 2.0) + 1.0) * (SCREEN_W / 2.0));*/

}

int worldToScreenY(float worldAngle, float headPitch){
  float delta = worldAngle - headPitch;
  // 04/7 - This equation works better
  return (int)((-delta / (FOV_V / 2.0) + 1.0) * (SCREEN_H/12.0));

  /*SERIAL_PORT.print("WorldY: "); SERIAL_PORT.print((int)((delta / (FOV_V / 2.0) + 1.0) * (SCREEN_H / 2.0)));
  return (int)((delta / (FOV_V / 2.0) + 1.0) * (SCREEN_H / 2.0));*/

}

void drawTriangleCenter(int centerX, int centerY, uint16_t color) {
    tft.drawTriangle(centerX, centerY-15, centerX-15, centerY+15, centerX+15, centerY+15, color);
}

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT){};

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000); //Setting the clock frequency

  imu.enableDebugging();

  //Initialize IMU
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

  SERIAL_PORT.println("AR Head Tracking");
  delay(250);

  // TFT setup
  tft.setSPISpeed(40000000);
  tft.initR(INITR_MINI160x80);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.invertDisplay(true);
  SERIAL_PORT.println(F("Tft Initialized"));

  SERIAL_PORT.println("AR System Ready");
}

void loop() {
  //tft.invertDisplay(true);

  //Poll IMU data
  readIMU();

  //Edit - trying to use real-delta timing
  static unsigned long lastFrame = 0;
  unsigned  long now = millis();
  if (now - lastFrame < 33) return;
  lastFrame = now;

  // -------- MAP TO SCREEN --------
  // all three focus on both yaw and pitch changes instead of the alternating in the tft script
  int cx  = worldToScreenX(anchorWorldX, headYaw);
  int cy  = worldToScreenY(anchorWorldY, headPitch);

  int cx2 = worldToScreenX(anchorWorldX2, headYaw);
  int cy2 = worldToScreenY(anchorWorldY2, headPitch);

  int cx3 = worldToScreenX(anchorWorldX3, headYaw);
  int cy3 = worldToScreenY(anchorWorldY3, headPitch);

  // -------- DRAW --------
  tft.fillScreen(ST77XX_BLACK);

  drawTriangleCenter(cx,  cy,  ST77XX_CYAN);
  drawTriangleCenter(cx2, cy2, ST77XX_RED);
  drawTriangleCenter(cx3, cy3, ST77XX_WHITE);
}


