/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-post-image-photo-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

//#include <Arduino.h>
#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_wifi.h>
#include "esp_camera.h"
#include "string.h"
//#include <WiFiClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
// FTP Client Lib
#include "ESP32_FTPClient.h"
#include <TimeLib.h>
#include <esp_task_wdt.h>
//#include "ESPAsyncWebServer.h"

#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_WROVER_KIT 
//#include "camera_pins.h"
#define FRAME_SIZE FRAMESIZE_UXGA
#define WIDTH 1600
#define HEIGHT 1200
#define BLOCK_SIZE 10
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 0.2
#define IMAGE_DIFF_THRESHOLD 0.1
#define DEBUG 1


uint16_t prev_frame[H][W] = { 0 };
uint16_t current_frame[H][W] = { 0 };


bool setup_camera(framesize_t);
bool capture_still();
bool motion_detect();
void update_frame();
void print_frame(uint16_t frame[H][W]);
//const char* ssid = "COSMOTE-189DDC";
//const char* password = "UXYebdfUddddKqAq";

const char* ssid = "conn-xe73110";
const char* password = "dc028ee73110";

char ftp_server[] = "192.168.1.28";
char ftp_user[]   = "esp32cam";
char ftp_pass[]   = "esp32cam";

// Camera buffer, URL and picture name
camera_fb_t *fb = NULL;
String pic_name = "esp32_cam2-";

//String serverName = "192.168.1.28";   // REPLACE WITH YOUR Raspberry Pi IP ADDRESS
//String serverName = "example.com";   // OR REPLACE WITH YOUR DOMAIN NAME

String serverPath = "/upload.php";     // The default serverPath should be upload.php

//const int serverPort = 80;
const char* ntpServer = "192.168.1.28";
const long  gmtoffset_sec = 0;   //Replace with your GMT offset (seconds)
const int   daylightOffset_sec = 0;  //Replace with your daylight offset 

WiFiClient client;
unsigned long last=millis();

void FTP_upload( void );
bool take_picture(void);

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define POWER_DOWN_NUM 26

#define WDT_TIMEOUT 300
#define TIME_TO_SLEEP  60            //time ESP32 will go to sleep (in seconds)
#define uS_TO_S_FACTOR 1000000ULL   //conversion factor for micro seconds to seconds */
String sendPhoto();

const int timerInterval = 30000;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent
ESP32_FTPClient ftp (ftp_server, ftp_user, ftp_pass,5000,2);
AsyncWebServer server(80);

void setup() {
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);
  int timoutTimer = 40000;
  long startTimer = millis();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if ((startTimer+timoutTimer) < millis()) ESP.restart();
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
  delay(2000);
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
  config.xclk_freq_hz = 20000000; //originally set to 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //config.grab_mode = CAMERA_GRAB_LATEST;



  // init with high specs to pre-allocate larger buffersfreq

  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // originally FRAMESIZE_SVGA;
    config.jpeg_quality = 9; //originally 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  configTime(gmtoffset_sec, daylightOffset_sec, ntpServer);
  //delay(2000);
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);

    ESP.restart();
    
  }
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  //adjustments---------
//sensor_t * s = esp_camera_sensor_get();
/*s->set_brightness(s, 0);     // -2 to 2
s->set_contrast(s, 0);       // -2 to 2
s->set_saturation(s, 0);     // -2 to 2
s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
s->set_aec2(s, 0);           // 0 = disable , 1 = enable
s->set_ae_level(s, 0);       // -2 to 2
s->set_aec_value(s, 300);    // 0 to 1200
s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
s->set_agc_gain(s, 0);       // 0 to 30
s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
s->set_bpc(s, 0);            // 0 = disable , 1 = enable
s->set_wpc(s, 1);            // 0 = disable , 1 = enable
s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
s->set_lenc(s, 1);           // 0 = disable , 1 = enable
s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
s->set_vflip(s, 0);          // 0 = disable , 1 = enable
s->set_dcw(s, 1);            // 0 = disable , 1 = enable
s->set_colorbar(s, 0);       // 0 = disable , 1 = enable*/
//s->set_reg(s,0xff,0xff,0x00);//banksel
//s->set_reg(s,0xd3,0xff,0x82);//clock
//s->set_quality(s,9);

delay(1000);
//pinMode(4, INPUT);
//digitalWrite(4, LOW);
//rtc_gpio_hold_dis(GPIO_NUM_4);
//------------------------------------------
Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
  delay(1000);
  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()) );
  });
 
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain","ok");
    delay(2000);
    ESP.restart();
  });
 
  server.begin();
  //sendPhoto(); 
  //update_frame();

    /*if (millis() - last >= 180000)
  {
    esp_task_wdt_reset();
    last = millis();
  }*/
    //pinMode(4, OUTPUT);
    //digitalWrite(4, LOW);
    //rtc_gpio_hold_en(GPIO_NUM_4);
    //esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);

   // Serial.println("This will never be printed");
  
}

void loop() {

    if( take_picture() )
    {
      delay(1000);
      FTP_upload();
      Serial.println("Going to sleep now");
      delay(1000);
      ESP.getFreeHeap();
      esp_deep_sleep_start();
    }
    else
    {
      Serial.println("Capture failed, sleeping");
      esp_deep_sleep_start();
    }
    
  
  ////unsigned long currentMillis = millis();
  //if (currentMillis - previousMillis >= timerInterval) {
  //  sendPhoto();
   // previousMillis = currentMillis;
 // }

 //*****************motion detection*************
  /*if (!capture_still()) {
        Serial.println("Failed capture");
        delay(3000);

        return;
    }
  if (motion_detect()) {
        Serial.println("Motion detected");
        sendPhoto(); 
    }
  update_frame();
  */
}

/*String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-"+String(WiFi.RSSI())+"cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    //client.println("Keep-Alive: timeout=5, max=38");
    client.println("Connection: Keep-Alive");
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
        Serial.printf("fb->len = %d", n);
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
        Serial.printf("reminder->len = %d", fbLen-n);
      }
    }   
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 4000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
  return getBody;
}

bool capture_still() {
    camera_fb_t *frame_buffer = esp_camera_fb_get();

    if (!frame_buffer)
        return false;

    // set all 0s in current frame
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] = 0;


    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frame_buffer->buf[i];
        const uint16_t current = current_frame[block_y][block_x];

        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;

#if DEBUG
    Serial.println("Current frame:");
    print_frame(current_frame);
    Serial.println("---------------");
#endif

    esp_camera_fb_return(frame_buffer);

    return true;
}*/


/**
 * Compute the number of different blocks
 * If there are enough, then motion happened
 */
/*bool motion_detect() {
    uint16_t changes = 0;
    const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float current = current_frame[y][x];
            float prev = prev_frame[y][x];
            float delta = abs(current - prev) / prev;

            if (delta >= BLOCK_DIFF_THRESHOLD) {
#if DEBUG
                Serial.print("diff\t");
                Serial.print(y);
                Serial.print('\t');
                Serial.println(x);
#endif

                changes += 1;
            }
        }
    }

    Serial.print("Changed ");
    Serial.print(changes);
    Serial.print(" out of ");
    Serial.println(blocks);

    return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
}


/**
 * Copy current frame to previous
 */
/*void update_frame() {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            prev_frame[y][x] = current_frame[y][x];
        }
    }
}

/**
 * For serial debugging
 * @param frame
 */
/*void print_frame(uint16_t frame[H][W]) {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }

        Serial.println();
    }
}*/

//#######******ftp*******
bool take_picture()
{
  char timestamp [20];
  Serial.println("Taking picture now");

  fb = esp_camera_fb_get();  
  if(!fb)
  {
    Serial.println("Camera capture failed");
    return false;
  }
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
  }
  //Serial.println(&timeinfo, "%d/%m/%Y %H:%M:%S");
  strftime(timestamp,20,"%d-%m-%Y-%H:%M:%S",&timeinfo);
  // Rename the picture with the time string
  pic_name += String( timestamp ) + String(WiFi.RSSI())+ ".jpg";
  Serial.print("Camera capture success, saved as:");
  Serial.print( pic_name );
  return true;
}

void FTP_upload()
{
  Serial.println("Uploading via FTP");
  ftp.OpenConnection();
  
  //Create a file and write the image data to it;
  ftp.InitFile("Type I");
  ftp.ChangeWorkDir("/home/uploads/"); // change it to reflect your directory
  const char *f_name = pic_name.c_str();
  ftp.NewFile( f_name );
  if(ftp.isConnected()) 
  {
    ftp.WriteData(fb->buf, fb->len);
    Serial.println("The FTP uploading completed");
  }
  else Serial.print("FTP connection Failed");
  ftp.CloseFile();
  ftp.CloseConnection();
  delay(100);
  esp_task_wdt_reset();
  esp_camera_fb_return(fb);
}