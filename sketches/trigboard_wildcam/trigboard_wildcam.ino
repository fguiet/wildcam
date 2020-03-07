/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
   - FTDI Converter : UOT = RX, UOR = TX, GND = GND, 5V = VCC

  Resistance R2 is 215kOhm....because ESP32 is slow to boot
  Circuit is ok if when turned on CAM is turning ON by TPL51111

  Reference :
    - ESP bare ship pinout : https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
  

  Consumption : 10.5uA when sleeping, 
                peak 180mA when taking picture

  Programmation via USB FTDI, GRND to GRND, U0R to TX, U0T to RX
  Careful : Pin 3 (U0R) is used in the program...so need to be unplugged before programming board 

*********/

//Light Mqtt library
#include <PubSubClient.h>
//Wifi library
#include <WiFi.h>
#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory
#include <ArduinoJson.h>

//Mqtt settings
#define MQTT_SERVER "192.168.1.25"

char message_buff[200];

// WiFi settings
const char* ssid = "DUMBLEDORE";
const char* password = "frederic";

#define SMS_TOPIC  "guiet/automationserver/smsservice"
#define MQTT_CLIENT_ID "TrigBoardWildCamSensor"
#define MAX_RETRY 100
#define FIRMWARE_VERSION "1.0"

WiFiClient espClient;
PubSubClient client(espClient);

// define the number of bytes you want to access
#define EEPROM_SIZE 1

// Pin definition for CAMERA_MODEL_AI_THINKER
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

int pictureNumber = 0;
float battery = 0;

#define DEBUG 1
#define DONE_PIN 3
#define EXTWAKE_PIN 14 //To check whether it is an external wake up or a tpl5111 timer wake up
#define LED_PIN 33 //ESP-12-E led pin
#define ADC_PIN 39 //To read battery voltage

void setup() {  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  //Pin Setup
  pinMode(EXTWAKE_PIN, INPUT); 

  //pinMode(DONE_PIN, INPUT_PULLUP);

  //Debug pin
  pinMode(LED_PIN, OUTPUT);    
  //digitalWrite(EXTWAKE_PIN, HIGH);

  //what wakes me up
  bool externalWakeUp = isExternalWakeUp();   
  //externalWakeUp = true;

  if (DEBUG) {
     //For debug purpose
     Serial.begin(115200);
     delay(100);
  }

  if (!externalWakeUp) {

     debug_message("TPL5111 woke me up! leave me alone...", true);
    
     makeLedBlink(3,200);      

      //Go to sleep immediatly
      weAreDone();

      return;
   }

   //Read voltage here otherwise it is not a good value that is read (due to camera, sd card)
   battery = readVoltage();

   debug_message("Time to take a picture !", true);     
}

//See : https://github.com/hallard/Battery-Voltage-Measure
// 672 is analog value when 4.06v baterry is plugged with voltage divider R1 = 47kOhm, R2 = 9kOhm
float readVoltage() {

  float analogValue  = analogRead(ADC_PIN);

  debug_message("Analog value : " + String(analogValue,2), true);
  
  float bv = (4.06/672)*analogValue;

  debug_message("Battery voltage : " + String(bv,2), true);

  return bv;
}

/*
 * EXTWAKE_PIN is HIGH when ESP is wake up by TPL5111 
 */
bool isExternalWakeUp() {
  if (digitalRead(EXTWAKE_PIN) == HIGH) {
    return false; 
  }
  else {
    return true;
  }
}


void loop() {

  //delay(500);

  takePicture();

  SendSMS();
  
  makeLedBlink(5,100); 
  
  weAreDone(); 
}

void takePicture() {

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
  config.pixel_format = PIXFORMAT_JPEG; 
  
 if(psramFound()) {
    debug_message("PSRAM was found and loaded", true);    
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {    
    debug_message("PSRAM NOT found", true); 
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  //debug_message("YAYA", true);
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {    
    if (DEBUG)
      Serial.printf("Camera init failed with error 0x%x", err);
    
    makeLedBlink(2,500);  
    weAreDone();
    return;
  }

  //debug_message("YOUOU", true);  
   
  if(!SD_MMC.begin()){
    debug_message("SD Card Mount Failed", true);     
    makeLedBlink(3,500);  
    weAreDone();
    return;
  } 

  //debug_message("YIYI", true);
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    debug_message("No SD Card attached", true);         
    makeLedBlink(4,500);  
    weAreDone();
    return;
  }

  //debug_message("TOTO", true);
    
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    debug_message("Camera capture failed", true);    
    makeLedBlink(5,500);  
    weAreDone();
    return;
  }

  //debug_message("PAPA", true);
  
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;  

  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";

  fs::FS &fs = SD_MMC; 

  if (DEBUG)
    Serial.printf("Picture file name: %s\n", path.c_str());

  //debug_message("PAPI", true);
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    debug_message("Failed to open file in writing mode", true);   
    makeLedBlink(6,500);  
    weAreDone();
    return;
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length

    if (DEBUG)
      Serial.printf("Saved file to path: %s\n", path.c_str());
          
    EEPROM.write(0, pictureNumber);    
    EEPROM.commit();    
  }
  
  file.close();
  esp_camera_fb_return(fb);   
  
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  //pinMode(4, OUTPUT);
  //digitalWrite(4, LOW);
  //rtc_gpio_hold_en(GPIO_NUM_4);

  debug_message("Picture taken", true);  
}

void makeLedBlink(int blinkTimes, int millisecond) {

  for (int x = 0; x < blinkTimes; x++) {
    digitalWrite(LED_PIN, HIGH);
    delay(millisecond);
    digitalWrite(LED_PIN, LOW);
    delay(millisecond);
  } 

  //Turn off at the end 
  digitalWrite(LED_PIN, HIGH);
}


void debug_message(String message, bool doReturnLine) {
  if (DEBUG) {
    if (doReturnLine)
      Serial.println(message);
    else
      Serial.print(message);
  }
}

void disconnectMqtt() {
  debug_message("Disconnecting from mqtt...", true);
  client.disconnect();
}

void disconnectWifi() {
  debug_message("Disconnecting from wifi...", true);
  WiFi.disconnect();
}

String ConvertToJSon(String message) {
    //Create JSon object
    DynamicJsonDocument  jsonBuffer(200);
    JsonObject root = jsonBuffer.to<JsonObject>(); 

    //Garage door still open guid
    String guid ="209bf64f-92f5-4540-b2a3-67329596c7b8";
    
    root["messageid"] = guid;
    root["messagetext"] = message;
           
    String result;
    serializeJson(root, result);

    return result;
}

void SendSMS() {
  if (WiFi.status() != WL_CONNECTED) {
    if (!connectToWifi())
      weAreDone();
  }  

  if (!client.connected()) {
    if (!connectToMqtt()) {
      weAreDone();
    }
  }

  String mess = ConvertToJSon("Wildcam : a picture has been taken ! (battery : "+String(battery,2)+" v)");  
  debug_message("JSON Sensor : " + mess + ", topic : " +SMS_TOPIC, true);
  mess.toCharArray(message_buff, mess.length()+1);
    
  client.publish(SMS_TOPIC,message_buff);

  disconnectMqtt();
  delay(100);
  disconnectWifi();
  delay(100);
}

boolean connectToMqtt() {

   client.setServer(MQTT_SERVER, 1883); 

  int retry = 0;
  // Loop until we're reconnected
  while (!client.connected() && retry < MAX_RETRY) {
    debug_message("Attempting MQTT connection...", true);
    
    if (client.connect(MQTT_CLIENT_ID)) {
      debug_message("connected to MQTT Broker...", true);
    } else {
      retry++;
      // Wait 5 seconds before retrying
      delay(500);
      //yield();
    }
  }

  if (retry >= MAX_RETRY) {
    debug_message("MQTT connection failed...", true);  
    return false;
    //goDeepSleep();
  }

  return true;
}

boolean connectToWifi() {

  // WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);  
  
  int retry = 0;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && retry < MAX_RETRY) {
    retry++;
    delay(500);
    debug_message(".", false);
  }

  if (WiFi.status() == WL_CONNECTED) {  
     debug_message("WiFi connected", true);  
     // Print the IP address
     if (DEBUG) {
      Serial.println(WiFi.localIP());
     }
     
     return true;
  } else {
    debug_message("WiFi connection failed...", true);   
    return false;
  }  
}

/* The DONE pin is driven by a μC to signal that the μC is working properly. The TPL5111 recognizes a valid
DONE signal as a low to high transition. */
void weAreDone() {

  //Needed otherwise if won't work!!
  pinMode(DONE_PIN, OUTPUT); 

  debug_message("Bye bye dude...", true);

  delay(500);
  
  for (int i=0;i<10;i++) {
    digitalWrite(DONE_PIN, HIGH);
    delay(100);
    digitalWrite(DONE_PIN, LOW);
    delay(100);
  }
}
