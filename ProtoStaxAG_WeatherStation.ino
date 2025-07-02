/*************************************************** 
  ProtoStaxAG Weather Station

  This is a Weather Station using Arduino Giga Display Shield,
  Arduino Giga R1 WIFI,
  BME280 on a prototying shield with I2C connected to D8 and D9
  and
  ProtoStax-AG Enclosure for Arduino Giga Display --> https://www.protostax.com/products/protostax-ag-enclosure-for-arduino-giga-display

  A lot of time and effort has gone into providing this and other code. 
  Please support ProtoStax by purchasing products from us!
 
  Written by Sridhar Rajagopal for ProtoStax
  BSD license. All text above must be included in any redistribution
 */

/* Define the following in your arduino_secrets.h file (create this file in this folder):
   #define SECRET_SSID "YOUR_SSID"
   #define SECRET_PASS "YOUR_WIFI_PASSWORD"
   #define SECRET_OWM_API_KEY "YOUR_OWM_API_KEY"
*/

#include "arduino_secrets.h"
#include "mbed.h"
#include <mbed_mktime.h>

#include "Arduino_H7_Video.h"
#include "Arduino_GigaDisplayTouch.h"
#include <Arduino_GigaDisplay.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h> // Required for parsing JSON

#include <lvgl.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

int timezone = -7; // Time zone - PDT - GMT - 7 
const char* location = "Mountain View, CA"; // Location - update this to what you want displayed

const char* server = "api.openweathermap.org";
const int port = 80;

// Lat and Long corresponding to Mountain View, CA
const String latitude = "37.3893889";
const String longitude = "-122.0832101";

#define BME280_ADDRESS 0x77  // Default I2C address

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

//Create rgb object
GigaDisplayRGB rgb;

Arduino_H7_Video          Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch  TouchDetector;

static lv_obj_t * time_label;

static int32_t indoor_temp_data = 35;
static lv_obj_t * indoor_temp_label;

static int32_t indoor_hum_data = 40;
static lv_obj_t * indoor_hum_label;

static int32_t indoor_pressure_data = 1013;
static lv_obj_t * indoor_pressure_label;

static int32_t outdoor_temp_data = 35;
static lv_obj_t * outdoor_temp_label;

static int32_t outdoor_hum_data = 40;
static lv_obj_t * outdoor_hum_label; 

static int32_t outdoor_pressure_data = 1013;
static lv_obj_t * outdoor_pressure_label;

// Declare the font to use (e.g., Montserrat 20)
LV_FONT_DECLARE(lv_font_montserrat_28); 


const int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

unsigned int localPort = 2390; // local port to listen for UDP packets

constexpr auto timeServer { "pool.ntp.org" };

byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

/* Slider red value changed event callback */
static void red_event_cb(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  int32_t value = lv_slider_get_value(slider);
  set_rgb_color(0, (uint8_t)value);
}

/* Slider green value changed event callback */
static void green_event_cb(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  int32_t value = lv_slider_get_value(slider);
  set_rgb_color(1, (uint8_t)value);
}

/* Slider blue value changed event callback */
static void blue_event_cb(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  int32_t value = lv_slider_get_value(slider);
  set_rgb_color(2, (uint8_t)value);
}

/* Convenience method to set the red, green, and blue values of the RGB LED individually while remembering the other values */
void set_rgb_color(int color, uint8_t val) {
  static uint8_t red = 0;
  static uint8_t green = 0;
  static uint8_t blue = 0;

  if (color == 0) {
    red = val;
  } else if (color == 1) {
    green = val;
  } else if (color == 2) {
    blue = val;
  }
  rgb.on(red, green, blue);
}

void set_label_font_size(lv_obj_t *label, const lv_font_t *font) {
  static lv_style_t style; // Static to avoid re-initialization on each call
  lv_style_init(&style);
  lv_style_set_text_font(&style, font);
  lv_obj_add_style(label, &style, 0);
}

void RTCset()  // Set cpu RTC
{    
  tm t;
            t.tm_sec = (0);       // 0-59
            t.tm_min = (52);        // 0-59
            t.tm_hour = (14);         // 0-23
            t.tm_mday = (18);   // 1-31
            t.tm_mon = (11);       // 0-11  "0" = Jan, -1 
            t.tm_year = ((22)+100);   // year since 1900,  current year + 100 + 1900 = correct year
            set_time(mktime(&t));       // set RTC clock                                 
}

String getLocaltime()
{
    char buffer[32];
    tm t;
    _rtc_localtime(time(NULL), &t, RTC_4_YEAR_LEAP_YEAR_SUPPORT);
    strftime(buffer, 32, "%Y-%m-%d %k:%M:%S", &t);
    return String(buffer);
}

void setNtpTime()
{
    Udp.begin(localPort);
    sendNTPpacket(timeServer);
    delay(1000);
    parseNtpPacket();
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(const char * address)
{
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0; // Stratum, or type of clock
    packetBuffer[2] = 6; // Polling Interval
    packetBuffer[3] = 0xEC; // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    Udp.beginPacket(address, 123); // NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
}

unsigned long parseNtpPacket()
{
    if (!Udp.parsePacket())
        return 0;

    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    const unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    const unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    const unsigned long secsSince1900 = highWord << 16 | lowWord;
    constexpr unsigned long seventyYears = 2208988800UL;
    const unsigned long epoch = secsSince1900 - seventyYears;
    const unsigned long new_epoch = epoch + (3600 * timezone); //multiply the timezone with 3600 (1 hour)

    set_time(new_epoch);    
    //set_time(epoch);

#if defined(VERBOSE)
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // print Unix time:
    Serial.println(epoch);

    // print the hour, minute and second:
    Serial.print("The UTC time is "); // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
        // In the first 10 minutes of each hour, we'll want a leading '0'
        Serial.print('0');
    }
    Serial.print((epoch % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
        // In the first 10 seconds of each minute, we'll want a leading '0'
        Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
#endif

    return epoch;
}

float kelvinToC(float value) {
  return (value - 273.15);
}

float kelvinToF(float value) {
  return (value - 273.15) * 9/5 + 32;
}

float CtoF(float value) {
  return (value * 9.0/5) + 32.0;
}

static void my_owm_timer(lv_timer_t * timer) {
  char buf[40];

  WiFiClient client;
  if (client.connect(server, port)) {

    // Use the Geocode API to get lat/long for your location by City or Zip
    // city would be something like "Mountain View,CA,US"
    // String geocodeUrl = "/geo/1.0/direct?q=" + city + "&appid=" + SECRET_OWM_API_KEY;

    String url = "/data/2.5/weather?lat=" + latitude + "&lon=" + longitude + "&appid=" + SECRET_OWM_API_KEY; // Example URL for current weather

    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + server + "\r\n" +
                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println(">>> Client Timeout !");
        client.stop();
        return;
      }
    }

    String response;
    while (client.available()) {
      char c = client.read();
      response += c;
      delay(1);
    }
    client.stop();

    // Find the index of the first '{' character (start of JSON)
    int startIndex = response.indexOf('{');
    if (startIndex != -1) {
      String jsonString = response.substring(startIndex);
      // Parse JSON
      StaticJsonDocument<2048> doc; // Adjust size as needed
      DeserializationError error = deserializeJson(doc, jsonString);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
      }

      // Extract data (example: temperature)
      float temperature = doc["main"]["temp"];
      sprintf( buf, "%6.2f \u00B0C", kelvinToC(temperature));
      lv_label_set_text(outdoor_temp_label, buf);     

      float humidity = doc["main"]["humidity"];
      sprintf( buf, "%6.2f %%", humidity);
      lv_label_set_text(outdoor_hum_label, buf);    

      float pressure = doc["main"]["pressure"];
      sprintf( buf, "%6.2f hPa", pressure);
      lv_label_set_text(outdoor_pressure_label, buf);         

    } else {
      Serial.print("No JSON found in the response: "); Serial.println(response);
    }
  } else {
    Serial.println("Connection to server failed.");
  }
}

static void my_timer(lv_timer_t * timer)
{
  /*Use the user_data*/
  // uint32_t * user_data = (uint32_t*) lv_timer_get_user_data(timer);
  char buf[40];

  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  sprintf( buf, "%6.2f \u00B0C", temp_event.temperature);
  lv_label_set_text(indoor_temp_label, buf);

  sprintf( buf, "%6.2f %%", humidity_event.relative_humidity);
  lv_label_set_text(indoor_hum_label, buf);

  sprintf( buf, "%6.2f hPa", pressure_event.pressure);
  lv_label_set_text(indoor_pressure_label, buf);

  /*Update label*/
  // sprintf( buf, "%ld", *user_data);
  // lv_label_set_text(indoor_temp_label, buf);
  lv_label_set_text(time_label, getLocaltime().c_str());
}

void setup() {
  Serial.begin(9600);

  // while (!Serial);
  Serial.println("ProtoStaxAG Weather Station");

  rgb.begin();

  Display.begin();
  TouchDetector.begin();

  DisplaySetup();

  if (!bme.begin(BME280_ADDRESS, &Wire2)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  Wire2.begin(); 

  WiFi.begin(SECRET_SSID, SECRET_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  setNtpTime();

  /*Create Task and have it update every 1000 ms*/
  lv_timer_t * timer = lv_timer_create(my_timer, 1000,  NULL);
  lv_timer_t * timer2 = lv_timer_create(my_owm_timer, 60000,  NULL);
  lv_timer_ready(timer2);
}


/* Set up LVGL Display */ 
void DisplaySetup() {
  /* Create a container with grid 2x2 */
  static lv_coord_t col_dsc[] = {375, 375, LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row_dsc[] = {60, 380, LV_GRID_TEMPLATE_LAST};
  lv_obj_t * cont = lv_obj_create(lv_scr_act());
  lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);
  lv_obj_set_size(cont, Display.width(), Display.height());
  lv_obj_set_style_bg_color(cont, lv_color_hex(0x03989e), LV_PART_MAIN);
  lv_obj_center(cont);

  lv_obj_t * obj;
  uint16_t idx = 0u;

  /* [0:0] - Banner */

  obj = lv_obj_create(cont);
  lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_STRETCH, 0, 2,
                        LV_GRID_ALIGN_STRETCH, 0, 1);
  lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE); 


  lv_obj_t * banner_label = lv_label_create(obj);
  lv_label_set_text(banner_label, location);
  lv_obj_set_style_text_align( banner_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( banner_label, LV_ALIGN_TOP_LEFT, 10, 0 );
  set_label_font_size(banner_label, &lv_font_montserrat_28);

  time_label = lv_label_create(obj);
  lv_label_set_text(time_label, "---");
  lv_obj_set_style_text_align( time_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( time_label, LV_ALIGN_TOP_RIGHT, 0, 0 );
  set_label_font_size(time_label, &lv_font_montserrat_28);

  /* [1:0] - Indoor Data */
  obj = lv_obj_create(cont);
  lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_STRETCH, 0, 1,
                        LV_GRID_ALIGN_STRETCH, 1, 1);

  lv_obj_t * label = lv_label_create(obj);
  lv_label_set_text(label, "Indoor");
  lv_obj_set_style_text_align( label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( label, LV_ALIGN_TOP_MID, 0, 0 );
  set_label_font_size(label, &lv_font_montserrat_28);



  LV_IMG_DECLARE(temperature);
  lv_obj_t * img2 = lv_img_create(obj);
  lv_img_set_src(img2, &temperature);
  lv_obj_align(img2, LV_ALIGN_TOP_LEFT, 10, 50);
  lv_obj_set_size(img2, 100, 100);

  indoor_temp_label = lv_label_create(obj);
  lv_label_set_text(indoor_temp_label, "---");
  lv_obj_set_style_text_align( indoor_temp_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( indoor_temp_label, LV_ALIGN_TOP_LEFT, 150, 75 );
  set_label_font_size(indoor_temp_label, &lv_font_montserrat_28);

  LV_IMG_DECLARE(humidity);
  lv_obj_t * img3 = lv_img_create(obj);
  lv_img_set_src(img3, &humidity);
  lv_obj_align(img3, LV_ALIGN_TOP_LEFT, 10, 160);
  lv_obj_set_size(img3, 100, 100);

  indoor_hum_label = lv_label_create(obj);
  lv_label_set_text(indoor_hum_label, "---");
  lv_obj_set_style_text_align( indoor_hum_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( indoor_hum_label, LV_ALIGN_TOP_LEFT, 150, 185 );
  set_label_font_size(indoor_hum_label, &lv_font_montserrat_28);

  LV_IMG_DECLARE(barometer);
  lv_obj_t * img4 = lv_img_create(obj);
  lv_img_set_src(img4, &barometer);
  lv_obj_align(img4, LV_ALIGN_TOP_LEFT, 10, 260);
  lv_obj_set_size(img4, 100, 100);

  indoor_pressure_label = lv_label_create(obj);
  lv_label_set_text(indoor_pressure_label, "---");
  lv_obj_set_style_text_align( indoor_pressure_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( indoor_pressure_label, LV_ALIGN_TOP_LEFT, 150, 285 );
  set_label_font_size(indoor_pressure_label, &lv_font_montserrat_28);

  /* [1:1] - Outdoor Data */
  obj = lv_obj_create(cont);
  lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_STRETCH, 1, 1,
                        LV_GRID_ALIGN_STRETCH, 1, 1);
  // lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_COLUMN);

  lv_obj_t * right_label = lv_label_create(obj);
  lv_label_set_text(right_label, "Outdoor");
  lv_obj_set_style_text_align( right_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( right_label, LV_ALIGN_TOP_MID, 0, 0 );
  set_label_font_size(right_label, &lv_font_montserrat_28);

  LV_IMG_DECLARE(temperature);
  lv_obj_t * img1_right = lv_img_create(obj);
  lv_img_set_src(img1_right, &temperature);
  lv_obj_align(img1_right, LV_ALIGN_TOP_LEFT, 10, 50);
  lv_obj_set_size(img1_right, 100, 100);

  outdoor_temp_label = lv_label_create(obj);
  lv_label_set_text(outdoor_temp_label, "---");
  lv_obj_set_style_text_align( outdoor_temp_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( outdoor_temp_label, LV_ALIGN_TOP_LEFT, 150, 75 );
  set_label_font_size(outdoor_temp_label, &lv_font_montserrat_28);  

  LV_IMG_DECLARE(humidity);
  lv_obj_t * img2_right = lv_img_create(obj);
  lv_img_set_src(img2_right, &humidity);
  lv_obj_align(img2_right, LV_ALIGN_TOP_LEFT, 10, 160);
  lv_obj_set_size(img2_right, 100, 100);

  outdoor_hum_label = lv_label_create(obj);
  lv_label_set_text(outdoor_hum_label, "---");
  lv_obj_set_style_text_align( outdoor_hum_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( outdoor_hum_label, LV_ALIGN_TOP_LEFT, 150, 185 );
  set_label_font_size(outdoor_hum_label, &lv_font_montserrat_28);

  LV_IMG_DECLARE(barometer);
  lv_obj_t * img3_right = lv_img_create(obj);
  lv_img_set_src(img3_right, &barometer);
  lv_obj_align(img3_right, LV_ALIGN_TOP_LEFT, 10, 260);
  lv_obj_set_size(img3_right, 100, 100);

  outdoor_pressure_label = lv_label_create(obj);
  lv_label_set_text(outdoor_pressure_label, "---");
  lv_obj_set_style_text_align( outdoor_pressure_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align( outdoor_pressure_label, LV_ALIGN_TOP_LEFT, 150, 285 );
  set_label_font_size(outdoor_pressure_label, &lv_font_montserrat_28);  

}



void loop() { 
  /* Feed LVGL engine */
  lv_timer_handler();

}




