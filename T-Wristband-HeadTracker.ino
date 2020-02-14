#include <TFT_eSPI.h>     // Graphics and font library for ST7735 driver chip (see https://github.com/Bodmer/TFT_eSPI.git)
#include <SPI.h>          // Serial Peripheral Interface (SPI) 
#include <Preferences.h>  // ESP32 Preferences / non-volatile storage

// local libs
#include "ttgo.h"
#include "wifi_setup.h"
#include "tftHelper.h"
#include "xMPU9250.h"
#include "openTrack.h"
#include "faceTrackNoIr.h"

// switches
#define ARDUINO_OTA_UPDATE      //! Enable this line OTA update
//#define CALIBRATE_MAGNETOMETER //! calibrate magnemoter -> move in a 8
//#define SAVE_MAGNETOMETER_CALIB_TO_EEPROM
#define LOAD_MAGNETOMETER_CALIB_FROM_EEPROM
//#define SHOW_HEADING            //! show current heading on display
//#define SEND_TO_OPENTRACK
#define SEND_TO_FACETRACK

// libs for OTA
#ifdef ARDUINO_OTA_UPDATE
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

// hardware adresses
#define TP_PIN_PIN          33
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define IMU_INT_PIN         39
#define RTC_INT_PIN         34
#define BATT_ADC_PIN        35
#define VBUS_PIN            36
#define TP_PWR_PIN          25
#define LED_PIN             4
#define CHARGE_PIN          32

// chips
xMPU9250        imu(Wire,0x69);
TFT_eSPI        tft = TFT_eSPI(); 
Preferences     pref;

// consts
const char* myName="BeatWatch";
const float R2D = 180.0f / PI;  // conversion radians to degrees 

// MPU9250 params
//
// Sample rate divider:
//
// The magnetometer is fixed to an output rate of:
// - 100 Hz for frequencies of 100 Hz or above (SRD less than or equal to 9)
// - 8 Hz for frequencies below 100 Hz (SRD greater than 9)
//
// When the data is read above the selected output rate, the read data will be stagnant. 
// For example, when the output rate is selected to 1000 Hz, the magnetometer data will be 
// the same for 10 sequential frames.
//
// -> let's use 8 Hz and there fore srd > 9
const uint8_t imuSrd = 10;

// poll rate for 8 Hz
uint32_t imuPollRateMs = 0.9 * (1000.0 / 8.0);

//! NOT USED FOR MAGNEMOTEMTER ONLY const MPU9250::DlpfBandwidth dlpfBandwidth = MPU9250::DLPF_BANDWIDTH_10HZ;


// vars
char buff[256];
bool otaStart = false;

bool pressed = false;
uint32_t targetTimeForSleep = 0;

// vars written by interrupt service routines / ISR
volatile bool imu_data_ready = false;

// output macros
#define DCLEAR() {tftClear(&tft);}
//#define DPRINT(...) { char b = tfdGetBuffer(); snprintf(b, sizeof(b), __VA_ARGS__); tfdPrint(b); }
#define DPRINT(...) { char* b = tfdGetBuffer(); snprintf(tftBuffer[tftCurrentLine], sizeof(tftBuffer[tftCurrentLine]), __VA_ARGS__); tftPrint(&tft, tftBuffer[tftCurrentLine]); Serial.println(tftBuffer[tftCurrentLine]);}
#define DWAIT() {delay(2000);}

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
{
    if (percentage == 0) {
        tft.fillRoundRect(x0, y0, w, h, 3, TFT_BLACK);
    }
    uint8_t margin = 2;
    uint16_t barHeight = h - 2 * margin;
    uint16_t barWidth = w - 2 * margin;
    tft.drawRoundRect(x0, y0, w, h, 3, frameColor);
    tft.fillRect(x0 + margin, y0 + margin, barWidth * percentage / 100.0, barHeight, barColor);
}

void setupWiFi()
{
#ifdef ARDUINO_OTA_UPDATE
    DCLEAR();
    DPRINT("Wifi setup");
    DPRINT("Connection to %s", ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        DPRINT("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    DPRINT("IP address: %s", WiFi.localIP().toString().c_str());

    DWAIT();
       
#endif
}

void setupOTA()
{
#ifdef ARDUINO_OTA_UPDATE
    DCLEAR();
    DPRINT("OTA setup");
    
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname("T-Wristband");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
        imu.disableDataReadyInterrupt();

#ifndef SHOW_HEADING
        // see https://forums.adafruit.com/viewtopic.php?f=22&t=39675
        tft.writecommand(ST7735_DISPON);
        tft.writecommand(ST7735_SLPIN);
        delay(150);
        tft.init();
#endif
  
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
        otaStart = true;
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Updating...", tft.width() / 2 - 20, 55 );
    })
    .onEnd([]() {
        Serial.println("\nEnd");
        delay(500);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
        // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        int percentage = (progress / (total / 100));
        tft.setTextDatum(TC_DATUM);
        tft.setTextPadding(tft.textWidth(" 888% "));
        tft.drawString(String(percentage) + "%", 145, 35);
        drawProgressBar(10, 30, 120, 15, percentage, TFT_WHITE, TFT_BLUE);
    })
    .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");

        tft.fillScreen(TFT_BLACK);
        tft.drawString("Update Failed", tft.width() / 2 - 20, 55 );
        delay(3000);
        otaStart = false;
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
    });

    ArduinoOTA.begin();

    DPRINT("..done");
    DWAIT();
#endif
}

// ISR to set data ready flag */
void IRAM_ATTR data_ready()
{
  imu_data_ready = true;
}

// macros to store calibration in esp32 preferences
#define PUT_TO_PREF(PREF)             \
  float old##PREF = imu.get##PREF();  \
  pref.putFloat(#PREF, old##PREF);    \
  DPRINT(#PREF "=%06f", old##PREF);

#ifdef SAVE_MAGNETOMETER_CALIB_TO_EEPROM
  #define SET_FROM_PREF(PREF_BIAS, PREF_SCALE, SET_TO) {                      \
    float_t bias = pref.getFloat(#PREF_BIAS);                                 \
    float_t scale = pref.getFloat(#PREF_SCALE);                               \
    if (bias == NAN) {                                                        \
      DPRINT(#PREF_BIAS "is NAN!!");                                          \
    }                                                                         \
    else if (scale == NAN) {                                                  \
      DPRINT(#PREF_SCALE "is NAN!!");                                         \
    }                                                                         \
    else if (old##PREF_BIAS != bias) {         \
      DPRINT(#PREF_BIAS "Different than write!!");                            \
      DPRINT("%06f / %06f", old##PREF_BIAS, bias);                            \
    }                                                                         \
    else if (old##PREF_SCALE != scale) {       \
      DPRINT(#PREF_SCALE "Different than write!!");                           \
      DPRINT("%06f / %06f", old##PREF_SCALE, scale);                          \
    }                                                                         \ 
    else {                                                                    \
      imu.set##SET_TO(bias, scale);                                           \
    }                                                                         \
  }
#else
  #define SET_FROM_PREF(PREF_BIAS, PREF_SCALE, SET_TO) {                      \
    float_t bias = pref.getFloat(#PREF_BIAS);                                 \
    float_t scale = pref.getFloat(#PREF_SCALE);                               \
    if (bias == NAN) {                                                        \
      DPRINT(#PREF_BIAS "is NAN!!");                                          \
    }                                                                         \
    else if (scale == NAN) {                                                  \
      DPRINT(#PREF_SCALE "is NAN!!");                                         \
    }                                                                         \
   else {                                                                     \
      DPRINT(#PREF_BIAS "=%+06.2f", bias);                                    \
      DPRINT(#PREF_SCALE "=%+06.2f", scale);                                  \
      imu.set##SET_TO(bias, scale);                                           \
    }                                                                         \
  }
#endif

void setupMpu9250() {
    // initialize IMU 
    DCLEAR();
    DPRINT("Initializing IMU / MPU9250");
    int status = imu.begin();
    DPRINT("status = %i", status);
    if (status < 0) {
      DPRINT("... ERROR");
    }
    else {
      // setting the accelerometer full scale range to +/-8G 
//!      imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
      // setting the gyroscope full scale range to +/-500 deg/s
//!      imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
      // setting DLPF bandwidth to 20 Hz
//!      imu.setDlpfBandwidth(dlpfBandwidth);
      // setting SRD to 19 for a 50 Hz update rate
      imu.setSrd(imuSrd);
      DPRINT("... done");
    }
    DWAIT();

#ifdef CALIBRATE_MAGNETOMETER
    // Calibrate magnetometer
    DCLEAR();
    DPRINT("Calibrating magnetometer,");
    DPRINT(".. please slowly move in a");
    DPRINT(".. figure 8 until complete...");
    imu.calibrateMag();
    DPRINT("Done!");
    DWAIT(); 
#endif

    pref.begin(myName, false);
    
#if defined(CALIBRATE_MAGNETOMETER) && defined(SAVE_MAGNETOMETER_CALIB_TO_EEPROM)
    // Save calibration to Preferences
    DCLEAR();
    DPRINT("Saving Calibration to ESP32 Preferences");
    PUT_TO_PREF(MagBiasX_uT);
    PUT_TO_PREF(MagBiasY_uT);
    PUT_TO_PREF(MagBiasZ_uT);
    PUT_TO_PREF(MagScaleFactorX);
    PUT_TO_PREF(MagScaleFactorY);
    PUT_TO_PREF(MagScaleFactorZ);
    DPRINT("Done!");
    DWAIT();
#endif

#ifdef LOAD_MAGNETOMETER_CALIB_FROM_EEPROM
    // Load calibration from Preferences
    DCLEAR();
    DPRINT("Loading Mag.Calib from ESP32");
    SET_FROM_PREF(MagBiasX_uT, MagScaleFactorX, MagCalX);
    SET_FROM_PREF(MagBiasY_uT, MagScaleFactorY, MagCalY);
    SET_FROM_PREF(MagBiasY_uT, MagScaleFactorY, MagCalY);
    DPRINT("Done!");
    DWAIT();
#endif  

    //  Attach the data ready interrupt to the data ready ISR
    imu.enableDataReadyInterrupt();
    pinMode(IMU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(IMU_INT_PIN, data_ready, RISING); 
}

void sleep() 
{
#ifndef SHOW_HEADING
  // see https://forums.adafruit.com/viewtopic.php?f=22&t=39675
  tft.writecommand(ST7735_DISPON);
  tft.writecommand(ST7735_SLPIN);
#endif
  
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
  imu.setSleepEnabled();
  Serial.println("Go to Sleep");
  delay(3000);
  
  // updated according https://github.com/Xinyuan-LilyGO/LilyGO-T-Wristband/issues/2
  tft.writecommand(ST7735_SWRESET);
  delay(100);
  tft.writecommand(ST7735_SLPIN);
  delay(150);
  tft.writecommand(ST7735_DISPOFF);
  
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

// calc heading with magnetometer only as described in https://github.com/bolderflight/MPU9250/issues/33
float calcHeading() {
    /* Read the MPU 9250 data */
    imu.readSensor();
    float hx = imu.getMagX_uT();
    float hy = imu.getMagY_uT();
    float hz = imu.getMagZ_uT();

    // Normalize magnetometer data 
    float h = sqrtf(hx * hx + hy * hy + hz * hz);
    hx /= h;
    hy /= h;
    hz /= h; 
    // Compute euler angles 
    float yaw_rad = atan2f(-hy, hx);
    float heading_rad = constrainAngle360(yaw_rad);
    return heading_rad * R2D;
}

/* Bound angle between 0 and 360 */
float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}

void setup()
{
    Serial.begin(115200);

    tft.init();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  160, 80, ttgo);
    delay(200);
    
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    DCLEAR();
    DPRINT("Build Timestamp:");
    DPRINT(__DATE__);
    DPRINT(__TIME__);
    DWAIT();

    // set hw components up
    setupWiFi();
    setupOTA();
    setupMpu9250();
    
    pinMode(TP_PIN_PIN, INPUT);
    //! Must be set to pull-up output mode in order to wake up in deep sleep mode
    pinMode(TP_PWR_PIN, PULLUP);
    digitalWrite(TP_PWR_PIN, HIGH);

    pinMode(LED_PIN, OUTPUT);
    pinMode(CHARGE_PIN, INPUT_PULLUP);

#ifdef SHOW_HEADING
    DCLEAR();
#else
    // see https://forums.adafruit.com/viewtopic.php?f=22&t=39675
    tft.writecommand(ST7735_DISPOFF);
    tft.writecommand(ST7735_SLPOUT);
    delay(150);
#endif

    imu.disableAccAndGyro();
}

void loop()
{
#ifdef ARDUINO_OTA_UPDATE
    ArduinoOTA.handle();
#endif

    //! If OTA starts, skip the following operation
    if (otaStart)
        return;

    // button handling
    if (digitalRead(TP_PIN_PIN) == HIGH) {
        if (!pressed) {
            targetTimeForSleep = millis() + 3000;

            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            pressed = true;
        } else {
            if (millis() >= targetTimeForSleep) {
                sleep();
            }
        }
    } else {
        pressed = false;
    }

    // use imu reading
    if (imu_data_ready) {
      imu_data_ready = false;
      
      imu.readSensor();
      float heading = calcHeading();

#ifdef SHOW_HEADING
      DPRINT("%+06.2f", heading);
#endif

#ifdef SEND_TO_OPENTRACK
      OpenTrackPackage otPack;
      otPack.yaw = heading;
      openTrackSend(otPack);
#endif
#ifdef SEND_TO_FACETRACK
      FaceTrackPackage ftPack;
      ftPack.yaw = heading;
      faceTrackSend(ftPack);
#endif

      // wait for next imu readings
      digitalWrite(LED_PIN, HIGH);
      delay(imuPollRateMs/10);
      digitalWrite(LED_PIN, LOW);
      delay(imuPollRateMs*9/10);
    }
}
