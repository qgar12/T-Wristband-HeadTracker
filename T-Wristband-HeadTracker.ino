#include <TFT_eSPI.h>     // Graphics and font library for ST7735 driver chip (see https://github.com/Bodmer/TFT_eSPI.git)
#include <SPI.h>          // Serial Peripheral Interface (SPI) 
#include <Wire.h>         // I2C library
#include <WiFi.h>

// local libs
#include "ttgo.h"
#include "wifi_setup.h"
#include "tftHelper.h"
#include "xMPU9250.h"

#define ARDUINO_OTA_UPDATE      //! Enable this line OTA update
//#define CALIBRATE_MAGNETOMETER //! calibrate magnemoter -> move in a 8
#define IMU_SHOW
#define SEND_TO_OPENTRACK

#ifdef ARDUINO_OTA_UPDATE
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

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
xMPU9250         IMU(Wire,0x69);
TFT_eSPI        tft = TFT_eSPI(); 

// Flag set to indicate MPU 9250 data is ready (will be set in interrupt service routine ISR)
volatile bool imu_data_ready = false;

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
        initial = 1;
        targetTime = millis() + 1000;
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
        omm = 99;
    });

    ArduinoOTA.begin();

    DPRINT("..done");
    DWAIT();
#endif
}

// ISR to set data ready flag */
void data_ready()
{
  imu_data_ready = true;
}

void setupMpu9250() {
    // initialize IMU 
    DCLEAR();
    DPRINT("Initializing IMU / MPU9250");
    int status = IMU.begin();
    DPRINT("status = %i", status);
    if (status < 0) {
      DPRINT("... ERROR");
    }
    else {
      // setting the accelerometer full scale range to +/-8G 
      IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
      // setting the gyroscope full scale range to +/-500 deg/s
      IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
      // setting DLPF bandwidth to 20 Hz
      IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
      // setting SRD to 19 for a 50 Hz update rate
      IMU.setSrd(19);
      DPRINT("... done");
    }
    DWAIT();

#ifdef CALIBRATE_MAGNETOMETER
    // Calibrate magnetometer
    DCLEAR();
    DPRINT("Calibrating magnetometer,");
    DPRINT(".. please slowly move in a");
    DPRINT(".. figure 8 until complete...");
    IMU.calibrateMag();
    DPRINT("Done!");
    DWAIT(); 
#endif

    //  Attach the data ready interrupt to the data ready ISR
    IMU.enableDataReadyInterrupt();
    pinMode(IMU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(IMU_INT_PIN, data_ready, RISING); 
}

void sleep() 
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
  IMU.setSleepEnabled();
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

void IMU_Show()
{
  if (imu_data_ready) {
    imu_data_ready = false;
    
    DCLEAR();
        
    DPRINT("--  ACC  GYR   MAG");
    tft.drawString(buff, 0, 0);
    DPRINT("x %+06.2f  %+06.2f  %+06.2f", IMU.getAccelX_mss(), IMU.getGyroX_rads(), IMU.getMagX_uT());
    tft.drawString(buff, 0, 16);
    DPRINT("y %+06.2f  %+06.2f  %+06.2f", IMU.getAccelY_mss(), IMU.getGyroY_rads(), IMU.getMagY_uT());
    tft.drawString(buff, 0, 32);
    DPRINT("z %+06.2f  %+06.2f  %+06.2f", IMU.getAccelZ_mss(), IMU.getGyroZ_rads(), IMU.getMagZ_uT());
    tft.drawString(buff, 0, 48);
    float heading = atan2(IMU.getMagY_uT(), IMU.getMagX_uT()) * 180 / PI;
    DPRINT("heading = %.2f", heading);

  }
} 

void setup()
{
    Serial.begin(115200);

    tft.init();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  160, 80, ttgo);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);


    setupWiFi();
    setupOTA();
    setupMpu9250();
    
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Note: the new fonts do not draw the background colour

    targetTime = millis() + 1000;

    pinMode(TP_PIN_PIN, INPUT);
    //! Must be set to pull-up output mode in order to wake up in deep sleep mode
    pinMode(TP_PWR_PIN, PULLUP);
    digitalWrite(TP_PWR_PIN, HIGH);

    pinMode(LED_PIN, OUTPUT);

    pinMode(CHARGE_PIN, INPUT_PULLUP);
    attachInterrupt(CHARGE_PIN, [] {
        charge_indication = true;
    }, CHANGE);

    if (digitalRead(CHARGE_PIN) == LOW) {
        charge_indication = true;
    }
}

void loop()
{
#ifdef ARDUINO_OTA_UPDATE
    ArduinoOTA.handle();
#endif

    //! If OTA starts, skip the following operation
    if (otaStart)
        return;

    if (digitalRead(TP_PIN_PIN) == HIGH) {
        if (!pressed) {
            initial = 1;
            targetTime = millis() + 1000;
            tft.fillScreen(TFT_BLACK);
            omm = 99;
            func_select = func_select + 1 > 2 ? 0 : func_select + 1;
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            pressed = true;
            pressedTime = millis();
        } else {
            if (millis() - pressedTime > 3000) {
 //             sleep();
            }
        }
    } else {
        pressed = false;
    }

    // read the IMU
    IMU.readSensor();

#ifdef IMU_SHOW
    IMU_ShowHeading();
    delay(200);
#endif

}