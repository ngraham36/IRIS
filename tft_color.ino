#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#if defined(ARDUINO_FEATHER_ESP32) // Feather Huzzah32
  #define TFT_CS         14
  #define TFT_RST        15
  #define TFT_DC         32

#elif defined(ESP8266)
  #define TFT_CS         4
  #define TFT_RST        16                                            
  #define TFT_DC         5

#else
  // For the breakout board, you can use any 2 or 3 pins.
  // These pins will also work for the 1.8" TFT shield.
  #define TFT_CS        10
  #define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
  #define TFT_DC         8
#endif

// Field of View (represents what user can see ontop of world view)
#define FOV_H  30.0
#define FOV_V  16.0

// TFT display size
#define SCREEN_W  160
#define SCREEN_H  80


// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// For ST7735-based displays, we will use this call
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Will be collected from IMU
float headPitch = 0.0;  // Up and down;  Purposes for isolated testing of "yaw" (won't chnage in loop for triangle 1)
float headPitch2 = 0.0; 
float headRoll = 0.0;   // Head rotation
float headYaw = 0.0;    // Left and right
float headYaw2 = 0.0;   // Purposes for isolated testing of "pitch" (won't chnage in loop for triangle 3)

/*
---------------------------------
Functions for mapping world coordinates to pixel coordinates

delta = how far object is from where head is pointing
FOV_H / 2.0 = normalize range so everything outside of range is clipped 
+ 1.0 = Centers range to be positive numbers [2,0] rather than negative [-1,1]
SCREEN_W/2.0 = Scales to pixel rnage
---------------------------------
*/

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

void setup(void) {
  Serial.begin(9600);
  

  // use this initializer (uncomment) if using a 0.96" 160x80 TFT:
  tft.initR(INITR_MINI160x80);  // Init ST7735S mini display

  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  tft.setSPISpeed(40000000);
  
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println(F("Tft Initialized"));

}

void loop() {
  tft.invertDisplay(true);

  // Frame timer
  static unsigned long lastFrame = 0;
  unsigned  long now = millis();

  // Skip redraw if frame hasn't reached 33 fps limit
  if (now - lastFrame < 33) return;
  lastFrame = now;


  // Starting with simulating head motion with sine wave (for smoothness), essentially 30 degree "head swings"
  headYaw = 30.0 * sin((float)now / 2000.0f);
  headPitch2 = 30.0 * sin((float)now / 2000.0f);

  //Initilaize anchor of the real world object
  float anchorWorldX = 0.0;
  float anchorWorldY = 0.0;

  // Real object 2 - starting from different starting point in real world
  float anchorWorldX2 = 1.0;
  float anchorWorldY2 = 1.0;

  // Real object 3 - starting from different starting point in real world
  float anchorWorldX3 = 5.0;
  float anchorWorldY3 = 5.0;

  //Triangle 1 - focused on yaw changes (horizontal motion)
  int cx = worldToScreenX(anchorWorldX, headYaw);
  int cy = worldToScreenY(anchorWorldY, headPitch);

  //Triangle 2 - focused on pitch + yaw changes (diagonal motion)
  int cx2 = worldToScreenX(anchorWorldX2, headYaw);
  int cy2 = worldToScreenY(anchorWorldY2, headPitch2);

  //Triangle 3 - focused on pitch (vertcial motion)
  int cx3 = worldToScreenX(anchorWorldX3, headYaw2);
  int cy3 = worldToScreenY(anchorWorldY3, headPitch2);

  tft.fillScreen(ST77XX_BLACK);

  // Redrawing triangles based on changing anchored center points (cx.cy)
  drawTriangleCenter(cx, cy, ST77XX_CYAN);
  drawTriangleCenter(cx2, cy2, ST77XX_RED);
  drawTriangleCenter(cx3, cy3, ST77XX_WHITE);
}
