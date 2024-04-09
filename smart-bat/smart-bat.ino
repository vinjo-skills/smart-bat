/*******************************************************************************
 * LVGL Hello World
 * This is a simple example for LVGL - Light and Versatile Graphics Library
 * import from: https://github.com/lvgl/lv_demos.git
 *
 * Dependent libraries:
 * LVGL: https://github.com/lvgl/lvgl.git
 *
 * LVGL Configuration file:
 * Copy your_arduino_path/libraries/lvgl/lv_conf_template.h
 * to your_arduino_path/libraries/lv_conf.h
 *
 * In lv_conf.h around line 15, enable config file:
 * #if 1 // Set it to "1" to enable content
 *
 * Then find and set:
 * #define LV_COLOR_DEPTH     16
 * #define LV_TICK_CUSTOM     1
 *
 * For SPI/parallel 8 display set color swap can be faster, parallel 16/RGB screen don't swap!
 * #define LV_COLOR_16_SWAP   1 // for SPI and parallel 8
 * #define LV_COLOR_16_SWAP   0 // for parallel 16 and RGB
 ******************************************************************************/
#include <Arduino.h>
#include <lvgl.h>
#include "ui.h"
#include "QMI8658.h"
#include "DEV_Config.h"
#include <stdlib.h>
#include <math.h>

/*******************************************************************************
 * Start of Arduino_GFX setting
 *
 * Arduino_GFX try to find the settings depends on selected board in Arduino IDE
 * Or you can define the display dev kit not in the board list
 * Defalult pin list for non display dev kit:
 * Arduino Nano, Micro and more: CS:  9, DC:  8, RST:  7, BL:  6, SCK: 13, MOSI: 11, MISO: 12
 * ESP32 various dev board     : CS:  5, DC: 27, RST: 33, BL: 22, SCK: 18, MOSI: 23, MISO: nil
 * ESP32-C3 various dev board  : CS:  7, DC:  2, RST:  1, BL:  3, SCK:  4, MOSI:  6, MISO: nil
 * ESP32-S2 various dev board  : CS: 34, DC: 38, RST: 33, BL: 21, SCK: 36, MOSI: 35, MISO: nil
 * ESP32-S3 various dev board  : CS: 40, DC: 41, RST: 42, BL: 48, SCK: 36, MOSI: 35, MISO: nil
 * ESP8266 various dev board   : CS: 15, DC:  4, RST:  2, BL:  5, SCK: 14, MOSI: 13, MISO: 12
 * Raspberry Pi Pico dev board : CS: 17, DC: 27, RST: 26, BL: 28, SCK: 18, MOSI: 19, MISO: 16
 * RTL8720 BW16 old patch core : CS: 18, DC: 17, RST:  2, BL: 23, SCK: 19, MOSI: 21, MISO: 20
 * RTL8720_BW16 Official core  : CS:  9, DC:  8, RST:  6, BL:  3, SCK: 10, MOSI: 12, MISO: 11
 * RTL8722 dev board           : CS: 18, DC: 17, RST: 22, BL: 23, SCK: 13, MOSI: 11, MISO: 12
 * RTL8722_mini dev board      : CS: 12, DC: 14, RST: 15, BL: 13, SCK: 11, MOSI:  9, MISO: 10
 * Seeeduino XIAO dev board    : CS:  3, DC:  2, RST:  1, BL:  0, SCK:  8, MOSI: 10, MISO:  9
 * Teensy 4.1 dev board        : CS: 39, DC: 41, RST: 40, BL: 22, SCK: 13, MOSI: 11, MISO: 12
 ******************************************************************************/
#include <Arduino_GFX_Library.h>
#define GFX_BL 25

/* More dev device declaration: https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration */
#if defined(DISPLAY_DEV_KIT)

Arduino_DataBus *bus = new Arduino_RPiPicoSPI(8 /* DC */, 9 /* CS */, 10 /* SCK */, 11 /* MOSI */, 12 /* MISO */, spi1 /* spi */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, 12 /* RST */, 0 /* rotation */, true /* IPS */);

#endif /* !defined(DISPLAY_DEV_KIT) */
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

unsigned long startTime = 0;  // Variable to store the start time
unsigned long endTime = 0;
bool onPitch = false;  // Variable to track if the player is on pitch
float fullTimeOnSec = 0;
float fullTimeOnMin = 0;
float fullTimeOnHour = 0;
const float distanceInKm = 0.1;  // Distance in kilometers (100 meters)
float maxSpeedKmph = 0;
/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static uint32_t bufSize;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;
bool shotInProgress = false;
float maxAccX, maxAccY, maxAccZ;
float maxGyroX, maxGyroY, maxGyroZ;
String shotPayled;
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
#ifndef DIRECT_MODE
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
#endif  // #ifndef DIRECT_MODE

  lv_disp_flush_ready(disp);
}

void setup() {
  // Initialize Bluetooth serial
  Serial.begin(9600); // Initialize serial communication with the built-in Serial port (USB)
  
  if (DEV_Module_Init() != 0)
    Serial.println("GPIO Init Fail!");
  else
    Serial.println("GPIO Init successful!");

  DEV_SET_PWM(50);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX LVGL Hello World example");

  QMI8658_init();
  Serial.println("QMI8658_init\r\n");
  DEV_SET_PWM(100);
#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  lv_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();

#ifdef DIRECT_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 40;
#endif

#ifdef ESP32
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) {
    // remove MALLOC_CAP_INTERNAL flag try again
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * bufSize, MALLOC_CAP_8BIT);
  }
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * bufSize);
#endif
  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, bufSize);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
#ifdef DIRECT_MODE
    disp_drv.direct_mode = true;
#endif
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);

    /* Create simple label */
    /* lv_obj_t *label = lv_label_create(lv_scr_act());
      lv_label_set_text(label, "Hello Arduino! (V" GFX_STR(LVGL_VERSION_MAJOR) "." GFX_STR(LVGL_VERSION_MINOR) "." GFX_STR(LVGL_VERSION_PATCH) ")");
      lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);*/

    ui_init();
    startTime = millis();  // Record the start time   
  }
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  
  float acc[3], gyro[3];
  unsigned int tim_count = 0;
  unsigned long currentTime = millis();  // Get current time
  QMI8658_read_xyz(acc, gyro, &tim_count);
  endTime = millis();                                                                     // Record the end time
  float acceleration = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]) / 1200;  // Calculate total acceleration
  float timeInHours = (endTime - startTime) / 1000.0 / 3600.0;
  maxSpeedKmph = distanceInKm / timeInHours;
  if (acceleration > 1.5) {                 // Check if player is on pitch
    float timeOnPitch = millis() / 1000.0;  // Convert elapsed time to seconds
    fullTimeOnSec++;

    if (fullTimeOnSec > 60) {
      fullTimeOnMin++;
      fullTimeOnSec = 0;
    }

    if (fullTimeOnMin > 60) {
      fullTimeOnHour++;
      fullTimeOnMin = 0;
    }
  }

  float bat_lift_angle = atan2(gyro[0], gyro[2]);
  float bat_lift_angle_degrees = degrees(bat_lift_angle);
  if (bat_lift_angle_degrees < 0) {
    bat_lift_angle_degrees += 360;
  }

   // Check if a shot is in progress
  if (!shotInProgress && acceleration > 1) {
    // Start tracking the shot
    shotInProgress = true;
    maxAccX = acc[0];
    maxAccY = acc[1];
    maxAccZ = acc[2];
    maxGyroX = gyro[0];
    maxGyroY = gyro[1];
    maxGyroZ = gyro[3];
  }

  if (shotInProgress) {
    maxAccX = max(maxAccX, abs(acc[0]));
    maxAccY = max(maxAccY, abs(acc[1]));
    maxAccZ = max(maxAccZ, abs(acc[2]));
    maxGyroX = max(maxGyroX, abs(gyro[0]));
    maxGyroY = max(maxGyroY, abs(gyro[1]));
    maxGyroZ = max(maxGyroZ, abs(gyro[2]));
  }

   // Check if the shot has ended
  if (shotInProgress && acceleration < 2) {
    // Shot has ended, determine the direction
    determineDirection(maxAccX, maxAccY, maxAccZ, maxGyroX, maxGyroY, maxGyroZ);

    // Reset the flag and maximum values
    shotInProgress = false;
    maxAccX = maxAccY = maxAccZ = 0;
    maxGyroX = maxGyroY = maxGyroZ = 0;
  }



  char buf[10];  // sprintf text buffer
  char bufMin[10];
  char bufHour[10];
  char bufAnglez[15];
  char bufAnglex[15];
  char bufAngley[15];
  char bufSpeed[10];
  char bufShotPlayed[20]; 
  
  shotPayled.toCharArray(bufShotPlayed,sizeof(bufShotPlayed));

  lv_label_set_text(ui_Label3, dtostrf(fullTimeOnSec, 0, 0, buf));
  lv_label_set_text(ui_Label2, dtostrf(fullTimeOnMin, 0, 0, bufMin));
  lv_label_set_text(ui_Label1, dtostrf(fullTimeOnHour, 0, 0, bufHour));
  lv_label_set_text(ui_Label6, dtostrf(bat_lift_angle_degrees, 0, 0, bufAnglez));
  lv_label_set_text(ui_Label5, dtostrf(maxSpeedKmph, 0, 0, bufSpeed));
  lv_label_set_text(ui_Label11, bufShotPlayed); 
  //lv_arc_set_value(ui_Arc1, temp);

  //Serial.println(acceleration, 3);
  // Serial.print("ACC.X :");
  // Serial.println(acceleration,3);
  //Serial.print("Acceleration (m/s^2): ");
  //Serial.print(acc[0]);
  //Serial.print(", ");
  //Serial.print(acc[1]);
  //Serial.print(", ");
  //Serial.println(acc[2]);

  //Serial.print("Rotation (deg/s): ");
  //Serial.print(gyro[0]);
  //Serial.print(", ");
  //Serial.print(gyro[1]);
  //Serial.print(", ");
  //Serial.println(gyro[2]);


  /*
#ifdef DIRECT_MODE
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#else
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif
#endif // #ifdef DIRECT_MODE

#ifdef CANVAS
  gfx->flush();
#endif

  delay(5);
*/
delay(500);
}

void determineDirection(float maxAX, float maxAY, float maxAZ, float maxGX, float maxGY, float maxGZ) {
  // Determine the direction based on the maximum acceleration and rotation values
  if (maxAY > 2 && maxGX > 100) {
    shotPayled = "off side";
    //Serial.println("Shot played towards off side");
  } else if (maxAY < -2 && maxGX < -100) {
    shotPayled = "leg side";
    //Serial.println("Shot played towards leg side");
  } else if (maxAZ > 2 && maxGY > 100) {
    shotPayled = "square leg";
    //Serial.println("Shot played towards square leg");
  } else if (maxAZ > 2 && maxGY < -100) {
    shotPayled = "square off";
    //Serial.println("Shot played towards square off");
  } else {
    shotPayled ="What a shot!";
    //Serial.println("Shot direction unknown");
  }
  Serial.println(shotPayled);

}