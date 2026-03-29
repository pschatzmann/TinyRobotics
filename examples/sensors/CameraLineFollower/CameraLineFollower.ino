/**
 * @file CameraLineFollower.ino
 * @brief Example: Line detection using the ESP32 camera and TinyRobotics CameraLineFollower.
 *
 * Demonstrates how to use the CameraLineFollower class with the ESP32-CAM (AI-Thinker)
 * to detect a line in grayscale camera images. The example configures the camera,
 * processes frames in grayscale, and prints the detected line position and error.
 *
 * - Uses the ESP32 camera API (esp_camera.h) in grayscale mode for efficient processing.
 * - Pinout is set for the AI-Thinker ESP32-CAM; update pins if using a different module.
 * - Uses Scheduler to periodically process camera frames.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - ESP32 Arduino core (with camera support)
 *
 * @author Phil Schatzmann
 */
#include <TinyRobotics.h>

#include "esp_camera.h"

// AI-Thinker (ESP32-CAM) default pinout - adjust if you use a different module
// NOTE: if your board differs, update these pins accordingly
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

camera_config_t config;
Scheduler scheduler;
CameraLineFollower follower(80, 5);  // threshold=80, minWidth=5

void setupCameraConfig() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format =
      PIXFORMAT_GRAYSCALE;  // request grayscale to simplify processing

  // Try QVGA for faster processing (320x240)
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("CameraLineFollower example starting");

  // ...existing camera config...
  setupCameraConfig();
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (true) delay(1000);
  }
  Serial.println("Camera initialized");

  // setup scheduler
  scheduler.begin(200, processCameraLine, nullptr);
}

void processCameraLine(void*) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  if (fb->format != PIXFORMAT_GRAYSCALE) {
    Serial.println("Camera not in grayscale mode; results may be invalid");
  }
  CameraLineFollower::Result res =
      follower.process(fb->buf, fb->width, fb->height);
  if (res.found) {
    Serial.print("Line found at x=");
    Serial.print(res.position);
    Serial.print(" error=");
    Serial.println(res.error);
  } else {
    Serial.println("Line not found");
  }
  esp_camera_fb_return(fb);
}

void loop() { scheduler.run(); }
