#include "esp_camera.h"
#include <WiFi.h>
#include <Servo.h>
#include "definations.h"



//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
#define MSGR 0b10101010
#define MSGL 0b11010101

#include "camera_pins.h"
HardwareSerial ServoSerial(115200);

const char* ssid = "Target";
const char* password = "09876543";



int pos = 0;
int pos2 = 0;

void startCameraServer();
void irdetector(camera_fb_t * fb, std::vector<Dot> &detectedDots);

void setup() {
  // uint32_t errors = 0;
  // uint32_t i;
  // bool ok = true;
  // uint32_t len ;
  // uint8_t *arr;
  // len = ESP.getFreeHeap();
  // arr = (uint8_t*)malloc(len);
  
  // for (i = 0; i < len; ++i)
  // {
  //   arr[i] = 0xff;
  //   if (arr[i] != 0xff)
  //     ++errors;
  //   // arr[i] = 0x00;
  //   // if (arr[i] != 0x00)
  //   //   ++errors;
  // }
  // free(arr);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  ServoSerial.begin(115200, SERIAL_8N1, 14, 15); // rx, tx
  // Serial.printf("len = %u, errors = %u", len, errors);
  camera_config_t config;
  // config.ledc_channel = LEDC_CHANNEL_0;
  // config.ledc_timer = LEDC_TIMER_0;
  
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
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QQVGA;
    // config.jpeg_quality = 10;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    // config.jpeg_quality = 12;
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
  Serial.printf("PID %i\n", s->id.PID);
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  // s->set_framesize(s, FRAMESIZE_VGA);
  // s->set_lenc(s, 1);
  // s->set_raw_gma(s, 1);
  // s->set_wpc(s, 1);
  // s->set_dcw(s, 1);
  // s->set_aec_value(s, 168);
  // s->set_awb_gain(s, 1);
  // s->set_vflip(s, 1);
  // s->set_brightness(s, 1);
  // s->set_saturation(s, -2);

  Serial.printf("lenc %i wpc %i brightness %i\n", s->status.lenc, s->status.wpc, s->status.brightness);


#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  
}



void loop() {
  // put your main code here, to run repeatedly:
  camera_fb_t * fb = NULL;
  int64_t fr_start = millis();
  char* buf;
  fb = esp_camera_fb_get();
  buf = (char*)malloc(fb->len);
  std::vector<Dot> dots;
  irdetector(fb, dots); 
  
  int averx = 0;
  int avery = 0;
  for (auto dot : dots){
    averx += dot.x + dot.w;
    avery += dot.y + dot.h;
  }
  if (dots.size() > 0){
    averx = averx / dots.size() - fb->width / 2;
    avery = (avery / dots.size() - fb->height / 2) * -1;
  }
  
  esp_camera_fb_return(fb);
  int64_t fr_end = millis();
  // Serial.printf("%ums (%.1ffps) cpuf = %u\n", (uint32_t)(fr_end - fr_start), 1000.0 / (uint32_t)(fr_end - fr_start), ESP.getCpuFreqMHz());
  // delay(10000);
  char msgR[2] = {MSGR, 0};
  char msgL[2] = {MSGL, 0};

  
  // ++pos;
  // if (pos > 180)
  //   pos = 0;
  msgR[1] = averx + 90;
  msgL[1] = avery + 90;
  // ServoSerial.write(msg, 2);
  Serial.write(msgR, 2);
  Serial.write(msgL, 2);

  // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    // myservo.write(pos);              // tell servo to go to position in variable 'pos'
    // delay(15);                       // waits 15ms for the servo to reach the position
  // }
  // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    // myservo.write(pos);              // tell servo to go to position in variable 'pos'
    // delay(15);                       // waits 15ms for the servo to reach the position
  // }
}