#define mySSID "REPLACEMEUSER"
#define myPASSWORD "REPLACEMEUSER"
#define thingspeak_server "api.thingspeak.com"
#define thingspeak_api_key "NOTNEEDED"
#define mosquittoUSER "REPLACEMEUSER"
#define mosquittoPASSWORD "REPLACEMEUSER"
#define mosquittoSERVER "192.168.178.44" //REPLACEMEUSER

#define rotary6


#ifdef rotary6
    // ESP32 Schreibtischlampe mit Lora
  #define mosquittoNAME "SCHREIBTISCHLAMPE"
  #define mqtt_client_on
  //#define ANZAHL_LEDS 36
  #define ANZAHL_LEDS 6//36
  #define RGB_LED_DATA_PIN 25
  
  #define OBEN_LED_DATA_PIN 26
  #define OBEN_LEDS 1
  
  //#define WS2812_LEDS
  #define SK6812_LEDS
  #define RXD2 35
  #define TXD2 33
  #define BMP280_CONNECTED
  #define MPU9250_CONNECTED
  #define BME680_CONNECTED
  #define LDR_CONNECTED
  //#define SGP30_CONNECTED
  #define ROTARY_ENCODER_A_PIN 35
  #define ROTARY_ENCODER_B_PIN 32
  #define ROTARY_ENCODER_BUTTON_PIN 34
  #define LED_PIN 22
  #define LDRPin 15
   #define sda2 21 ///* I2C Pin Definition */
   #define scl2 22
  
  #define BUTTON_DISABLED 0
  #define BUTTON_ENABLED 1
  #define timeSeconds 600
  #define updaterate_leds 2000
  #define updaterate_sensors 3000
  #define max_counter 255
  #define min_counter 0
#endif

#ifdef rotary7
    // ESP32 Regenmelder DEMOBOARD
  #define mosquittoNAME "REGENMELDER"
  #define mqtt_client_on
  //#define ANZAHL_LEDS 36
  #define ANZAHL_LEDS 6//36
  #define RGB_LED_DATA_PIN 25
  
  #define OBEN_LED_DATA_PIN 26
  #define OBEN_LEDS 1
  
  //#define WS2812_LEDS
  #define SK6812_LEDS
  #define RXD2 35
  #define TXD2 33
  //#define BMP280_CONNECTED
  //#define MPU9250_CONNECTED
  //#define BME680_CONNECTED
  #define LDR_CONNECTED
  //#define SGP30_CONNECTED
  #define ROTARY_ENCODER_A_PIN 35
  #define ROTARY_ENCODER_B_PIN 32
  #define ROTARY_ENCODER_BUTTON_PIN 25 //34
  #define LED_PIN 22
  #define LDRPin 27
   #define sda2 21 ///* I2C Pin Definition */
   #define scl2 22
  
  #define BUTTON_DISABLED 0
  #define BUTTON_ENABLED 1
  #define timeSeconds 600
  #define updaterate_leds 2000
  #define updaterate_sensors 3000
  #define max_counter 255
  #define min_counter 0
#endif
