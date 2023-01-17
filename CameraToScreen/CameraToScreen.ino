#include <GxEPD.h>
#include <GxGDEW075Z08/GxGDEW075Z08.h> 
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "esp_camera.h"
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

GxIO_Class io(SPI, /*CS=5*/ SS, /*DC=*/ 2, /*RST=*/ 12); // arbitrary selection of 17, 16
GxEPD_Class display(io, /*RST=*/ 12, /*BUSY=*/ 4); // arbitrary selection of (16), 4 
#define PIN_BTN 16

void init_camera() {
  camera_config_t config;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB888;
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
    Serial.println("PSRAM");
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
  #endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  Serial.printf("PID: %d", s->id.PID);
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    Serial.println("PID");
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 2); // up the brightness just a bit
    s->set_saturation(s, 0); 
  }
  
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  init_camera();
  display.init(115200); // enable diagnostic output on Serial
  Serial.println("setup done");
  take_picture();
}


void prepare_picture(unsigned char* newBitmapData, unsigned char* oldBitmapData) {
  unsigned char gdeByte = 0;
  for (int y = 0; y < 48000; y++) {
        for (int i = 0; i < 8; i++) {
          unsigned char pixel = oldBitmapData[8 * y + i];
          if (pixel > 0x88) {
            gdeByte = gdeByte << 1 | 1; 
          } else {
            gdeByte = gdeByte << 1;  
          }
        } 
        newBitmapData[y] = gdeByte;
        gdeByte = 0;
    }
}

void rgbToGrayscale(unsigned char* bitmapData) {
  for(int i = 0; i < 800 * 600; i++) {
    bitmapData[i] = 0.3 * bitmapData[i * 3] + 0.59 * bitmapData[i * 3 + 1] + 0.11 * bitmapData[i * 3 + 2];
  }
}

void performFloydSteinbergDithering(uint8_t * bitmap, int height, int width) {
  unsigned char ditheringFilter[4] = { 7, 3, 5, 1 };
  {
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            if(x - 1 >= 0 && y + 1 < height && x + 1 < width)
            {
                unsigned char oldPixel;
                unsigned char newPixel = 0xFF;
                unsigned char others[4];
                unsigned char otherValues[4];
                char error = 0x00;
                //unsigned float accumulatedValue = 0f;

                oldPixel = bitmap[x + y * width];
                others[0] = bitmap[x + 1 + y * width];
                others[1] = bitmap[x - 1 + (y + 1) * width];         
                others[2] = bitmap[x + (y + 1) * width]; 
                others[3] = bitmap[x + 1 + (y + 1) * width];  

                if(oldPixel < 0x80)
                    newPixel = 0x00;

                bitmap[x + y * width] = newPixel;
                error = oldPixel - newPixel;

                for(int i = 0; i < sizeof(ditheringFilter) / sizeof(unsigned char); i++)
                {
                    otherValues[i] = others[i] + error * ditheringFilter[i] * 0.0625;
                }

                bitmap[x + 1 + y * width] = otherValues[0];
                bitmap[x - 1 + (y + 1) * width] = otherValues[1];        
                bitmap[x + (y + 1) * width] = otherValues[2];
                bitmap[x + 1 + (y + 1) * width] = otherValues[3];
            }
        }
    }
  }
}

void take_picture(){
  Serial.println("Taking picture..");
    camera_fb_t * fb = NULL;
    uint8_t * convertedBitmap;
    convertedBitmap = (uint8_t *)malloc(480*100);
    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
    }
    rgbToGrayscale(fb->buf);
    performFloydSteinbergDithering(fb->buf, 600, 800);
    prepare_picture(convertedBitmap, fb->buf); 
    display.drawPicture(convertedBitmap, 0, 100*480, 0); 
}

void loop() {
}
