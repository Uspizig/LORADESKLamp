
/*
 This sketch is a DEMO Sketch for LoRaLamp AIRQuality published on Hackaday.io(https://hackaday.io/project/180792-loralamp-airquality)
 
 
 
 WARNINGS
 1. Before you compile please ensure you modify all your credentials and used sensors in "credentials.h" which have been marked REPLACEMEUSER
 2. USE a Proper POWER SOURCE with at exactly 5 Volt(NOT 6V, not 4V)DC Voltage / The Power Supply should be capable of handling 2Ampere output
 3. To give You some more safety I set the default Amount of LEDS in the credentials.h to 6. 
 4. If you want to use more LEDS and have enough Power change ANZAHL_LEDS to 36
 This Source Code is provided "AS it is" . 
 If you use this code you agree that any harm, damage, burn or injuries are YOUR RESPONSIBILITY!
 The Author may not be held liable for any damage that might happen if you try this code out.
 
 
 Legal validity of this disclaimer
	This disclaimer is to be regarded as part of the internet publication which you were
	referred from. If sections or individual terms of this statement are not legal or correct,
	the content or validity of the other parts remain uninfluenced by this fact.

Referrals and links
	The author is not responsible for any contents linked or referred to from his pages -
	unless he has full knowledge of illegal contents and would be able to prevent the
	visitors of his site from viewing those pages. If any damage occurs by the use of
	information presented there, the author might not be liable.
	Furthermore the author is not liable for
	any postings or messages published by users of discussion boards, guest books or
	mailing lists provided.

 
 Also ensure you have installed the following libs:
 

Code Sources that influenced this Source Code and which libaries you may need:
- https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_auth/mqtt_auth.ino
- https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
- RGBW for SK6812: 
  - Original code by Jim Bumgardner (http://krazydad.com).
  - Modified by David Madison (http://partsnotincluded.com).
  - Extended by Christoph Wempe
  - https://gist.github.com/CWempe?direction=desc&sort=created

Adafruit BME/BMP280/SGP30 sketches/libs
- Please consider buying their products due to their great work in Arduino libs
- https://github.com/adafruit/Adafruit_SGP30
- https://github.com/adafruit/Adafruit_BMP280_Library

Bosch BME680 BSEC Lib
- https://github.com/BoschSensortec/BSEC-Arduino-library

Andreas Spiess
- OTA Sketch for OTA Update of the ESP32 over Wifie
- https://github.com/SensorsIot/ESP32-OTA

Hideakitai MPU9250 Sketch
- https://github.com/hideakitai/MPU9250

knolleary pubsubclient
- /pubsubclient/blob/master/examples/mqtt_auth/mqtt_auth
- https://github.com/knolleary/pubsubclient

Fastled
- https://github.com/FastLED/FastLED

/*
Goal:
Project Goal Build a Desktop Lamp that can measure the Air Quality and warn the user if certain levels are reached.


What is working:
 - Update with OTA over Wifi
 - Switch on the upper LED under the rotary encoder
 - Switch on the lower LED
 - Detect a turn of the LED with the MPU9250 Gyro
 - Measure Temperature with the BMP280
 - Measure Brghtness with the LDR 
 - Wifi connectivity
 - Publish and recieve Data over MQTT
 - Measure eCo2, TVOC Data from BME680
 - Gyro Calibration on startup
 - Code is working with SK6812 Diodes and WS2812

Where to improve:
- Action performed on Gyro Measurements
- FailSafe methods if Wifi or MQTT Server becomes unavailible

Things that needs to be implemented:
 - AdvANCED Methods that are performed whenever the lamp is turned
 - Current measurement to protect about over current and detect shortcuts to GND
 - Change or enable Light when AirQuality becomes over a specific value
 - Update all warning limits of the sensors(Temp, CO2, Humidity,Light ..)  over MQTT
 - Update Wifi Credentials, MQTT Server over Wifi
 - Integrate Demo Lora Sketch to this source
 - NTP:Current Time
 - Quiet Time: Times when the Dekslamp shall go to sleep
 - Adaptaions vor PVB V2 to disable quiscent current of each LED during off Phase
 - Low Power Mode(Sleep Mode of all sensors)
 - Ultra Low Power Mode: Let the Processor sleep


Basic Operation:
- Switch On the Desklamp with a Button PRESS [OK]
- Change Brightness or collor by rotating the knob [OK]
- Nice LED by LED Dimming for nice shutdown of the light[OK}

Advanced:
- Save the most used Brightness level by Longpress the button (5 Sec) [not yet implemented]
- Set the color temperature[Partly OK]
- Measure the Temperature/Humidity/CO2/Brightness[OK]
- Aktivate a Timer Function . Lamp Blinks once the timer has been reached: Aktivate by Turn the lamp head to 90° [Not yet implemented]
- Set the Color via MQTT[OK]

Advanced Phase 2:
- Set the Timerlength via MQTT [Not done yet]
- Detect Bluetooth Devices[needs to be implemented to this sketch]
- Transmit Data over LORA [OK but not yet implemented to this sketch]

  LORA:
  - Trasnmit Data if LED is switched ON/OFF= PORT1
  - Transmit detected Bluetooth Devices, Transmit detected Users with Corona Warn APP = PORT2
  - Transmit Air Quality Sensors = PORT3

*/
#include "OTA.h"
#include "credentials.h"
#include <FastLED.h>
#include <Adafruit_BMP280.h> // to include wire.h if nothing BMP280 is not selected

// Only include libaries and variables if Feature is activated in credentials.h

//LDR Brigntess Sensor
#ifdef LDR_CONNECTED
  #include <soc/sens_reg.h> //für ADC2 Betrieb während WIFI Betrieb
  static RTC_NOINIT_ATTR int reg_b; 
  int brightness = 0, lastBrightness = 0;
  int SENSITIVITY          = 100;
  int DARKNESS_THRESHOLD   = 800;
  int BRIGHTNESS_THRESHOLD = 3500;
#endif

//Sensirion SGP30 eCo2 Sensor
#ifdef SGP30_CONNECTED
  #include "Adafruit_SGP30.h"
  Adafruit_SGP30 sgp;
  int counter_co2messungen = 0;
#endif

//MPU9250 Gyro Sensor
#ifdef MPU9250_CONNECTED
  #include "MPU9250.h"
  MPU9250 mpu; 
        int yaw;
        int pitch;
        int roll;
        int avgyaw;
        int avgpitch;
        int avgroll;
        int previousyaw;
        int previouspitch;
        int previousroll;
#endif

//Bosch BMP280 Temperature and pressure Sensor
#ifdef BMP280_CONNECTED
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp; // I2C
  #define BMP_ADRESS 0x76
#endif 

//MQTT Client: I use a Mosquitto on Raspberry PI
#ifdef mqtt_client_on
  #include <PubSubClient.h>
  const char* mqtt_server = mosquittoSERVER; //"192.168.178.44";
  WiFiClient espClient;
  PubSubClient client(espClient);
  long lastMsg = 0;
  char msg[50];
  int value = 0;
#endif

//Bosch BME680 Temperature, eCo2,... and Pressure Sensor
#ifdef BME680_CONNECTED
  #include "bsec.h"
  Bsec iaqSensor;
  int bme680_co2 = 440;
  int bme680_iaq_value_measured = 0;
  int bme680_iaq_acc_measured = 0;
  int bme680_temp_measured = 0;
  int bme680_humidity_measured = 0;
  int bme680_pressure_measured = 0;
#endif

   
//Led Settings
  
  FASTLED_USING_NAMESPACE
  #define MAX_POWER_MILLIAMPS 500
  #ifdef WS2812_LEDS
    CRGB leds[ANZAHL_LEDS];
  #endif
  #ifdef SK6812_LEDS
    #include "FastLED_RGBW.h"
    CRGBW leds[ANZAHL_LEDS];
    CRGB *ledsRGB = (CRGB *) &leds[0];

    //Beleuchtung des Drehreglers
    CRGBW leds2[OBEN_LEDS];
    CRGB *ledsRGB2 = (CRGB *) &leds2[0];
  #endif  
  static uint8_t gHue = 0;


//Global Variables
    uint8_t hue, hue2 = 0;
    boolean zielwert_erreicht =0;
    uint8_t Aktuelle_LED=0;
    uint8_t schleifenzaehler = 0;
    int vorher_message =0;
    int counter = 0;
    int lastcounter=0;
    int counter_last=0;
    int drehregler=0;
    long lastTrigger = 0;
    long lastTrigger_LEDS=0;
    long lastTrigger_SENSOR=0;
    boolean ROTARY_A_NOW = 0;
    boolean ROTARY_B_NOW = 0;
    boolean ROTARY_A_LAST = 0;
    boolean BUTTON_CLICK = 0;
    boolean BUTTON_CLICK_ALT = 0;
    



/* Only check if Rotary PIN A or B has been changed first
 *  nur prüfen welcher Pin zuerst verändert wurde A oder B
 *  If A first -> Clockwise : Wenn A zuerst dann Clockwise
 *  If B first -> CounterClockwise : Bei B zuerst dann Counter Clockwise
*/
/*ISR ROUTINEN Interrupt Routines */
/*Checks if Rotary encoder was turned */
void IRAM_ATTR rotary_movement() {
  ROTARY_A_NOW = digitalRead(ROTARY_ENCODER_A_PIN);
  ROTARY_B_NOW = digitalRead(ROTARY_ENCODER_B_PIN);
  if (ROTARY_A_NOW != ROTARY_B_NOW){// nur beim ersten voreilenden Wechsel erfassen... den zweiten Wechsel ignorieren
    if (ROTARY_A_NOW != ROTARY_A_LAST){
      //A hat sich verändert Im Uhrzeigersinn
      if (counter < max_counter)counter++;
    }
    else{
      //B hat sich zuerst verändert Gegnuhrzeigersinn
      if (counter > min_counter)counter--;
    }
  }
    
}
/*Checks if Rotary encoder was pressed */
void IRAM_ATTR rotary_button() {
  BUTTON_CLICK = 1;  
}

/*ISR ROUTINEN ENDE */

/*
 * Prüft ob Button gedrückt wurde
 * Prüft ob Button lange gedrückt wurde
 * Zustände: 
 * 1. Button wurde nicht gedrückt -> Alt Zustand auf 0 setzen
 * 2a. Button wurde 2 mal Kurz gedrückt
 * 2B. Button wurde 1 mal Kurz gedrückt
 */
void rotary_onButtonClick() {
    if (BUTTON_CLICK != 0){ 
      blinky(2, 50);
      counter = 0;
      lastcounter=0;
      //Prüfung auf LONG / SHORT Press des Buttons
      if (BUTTON_CLICK_ALT == BUTTON_CLICK){  //Zustand 2a
        Serial.println("DoubleKlick Button erkannt"); 
        einzel();
        //fade_white(50);
        //LEDS_gruen();
        BUTTON_CLICK_ALT = BUTTON_CLICK;
        BUTTON_CLICK = 0; 
      }
      else{//Zustand 2B
        Serial.println("Kurzes Druecken des Button erkannt"); 
        //LEDS_weiss();
        LEDS_aus();
        BUTTON_CLICK_ALT = BUTTON_CLICK; //Speichern damit im nächsten Run der Longpress erkannt wird.
        BUTTON_CLICK = 0; 
      }
    }
    else{//Zustand 1
      BUTTON_CLICK_ALT = 0; //damit aus dem Longpress wieder rausgekommen wird.
      //LEDS_aus();
    }
}

/*
 * Prüft ob sich Rotary Encoder gedreht hat
 * Prüft in welche Richtung sich das Rad gedreht hat
 */
void rotary_loop() {
  if (lastcounter!=counter){
    if (counter > lastcounter){
      Serial.print("Clockwise:"); 
      LEDS_rot();
      blinky(2, 100);
    }
    if (counter < lastcounter){
      Serial.print("andersRum:"); 
      blinky(2, 100);
      LEDS_blau();
    }
    lastcounter = counter;
    Serial2.println(counter);
  }
}

//Some stupid Blinky Routine to light a LED
void blinky(int b, int time){
  for (int i = 0; i <= b; i++){
    digitalWrite(LED_PIN, HIGH);
    delay(time);
    digitalWrite(LED_PIN, LOW);
    delay(time);
  }
}

void rotary_encoder_setup(){ 
  
  #ifdef rotary6
    pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT); //on the Input a HW PULLUP is availible on rotary 6
    pinMode(ROTARY_ENCODER_A_PIN, INPUT); //on the Input a HW PULLUP is availible on rotary 6
    pinMode(ROTARY_ENCODER_B_PIN, INPUT);//on the Input a HW PULLUP is availible on rotary 6
  #else
    pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP); //use if on the Input NO HW PULLUP is availible and you may want to use SW PULLUP of ESP32
    pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP); //use if on the Input NO HW PULLUP is availible and you may want to use SW PULLUP of ESP32
    pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP); //use if on the Input NO HW PULLUP is availible and you may want to use SW PULLUP of ESP32
  #endif
    pinMode(LED_PIN, OUTPUT);
  
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_A_PIN), rotary_movement, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_B_PIN), rotary_movement, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_BUTTON_PIN), rotary_button, CHANGE);
}

void setup() {
  pinMode(TXD2, OUTPUT); //Debugging interface
  Serial.begin(115200);  Serial.println("Booting");
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  Serial2.println("Booting");
  Wire.begin(sda2, scl2);
  
  #ifdef LDR_CONNECTED
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.print("reason ");Serial.println(reason);
    if ((reason != ESP_RST_DEEPSLEEP)) {// get reg_b if reset not from deep sleep --- Necessary WORKAROUN due to ADC2 and Wifi are colliding on ESP32
      reg_b = READ_PERI_REG(SENS_SAR_READ_CTRL2_REG);
      Serial.println("Reading reg b.....");
    }
    analogSetPinAttenuation(LDRPin, ADC_11db); // Set the ADC Range on the ESP32
    adcAttachPin(LDRPin);
  #endif
  
  #ifdef SGP30_CONNECTED
    delay(2000);
    setup_sgp30();
    delay(2000); //give the CO2 Sensor a little bit of Time after COLD Start to warm up
    co2_messen();
  #endif

  
  #ifdef BME680_CONNECTED
    iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    checkIaqSensorStatus();
  
    bsec_virtual_sensor_t sensorList[10] = {//Check which sensor you may need
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };
  
    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();
  #endif
  
  
  setupOTA(mosquittoNAME);// Switch on OTA WIFI Update (Credits to A.Spiess)
  
  rotary_encoder_setup(); // Setup the Interrups for the Rotary encoder
  lastTrigger = millis();

  #ifdef WS2812_LEDS
    LEDS.addLeds<WS2812,RGB_LED_DATA_PIN,GRB>(leds, ANZAHL_LEDS);
    LEDS.setBrightness(154);
  #endif
  #ifdef SK6812_LEDS
    FastLED.addLeds<WS2812B, RGB_LED_DATA_PIN, RGB>(ledsRGB, getRGBWsize(ANZAHL_LEDS));
    FastLED.addLeds<WS2812B, OBEN_LED_DATA_PIN, RGB>(ledsRGB2, getRGBWsize(OBEN_LEDS));
    FastLED.setBrightness(154);
  #endif

  #ifdef MPU9250_CONNECTED  
    if (!mpu.setup(0x68)) {  // change to your I2C address if your breakout board uses a differnet pinout
            Serial.println("MPU connection to MPU9250 GYRO failed. Please check your connection.");
            delay(5000); // Provide here some clever workaround to annouce that GYRO may not be used
    }
    else{
      MPU_startup(); // Only startup if MPU has been detected
    }    
  #endif
  #ifdef  BMP280_CONNECTED
      BMP_Start(); 
    #endif  
  //einzel(); //Attention: Switches all your LED - Only if your Power supply can handle 2.6 Amps - NOT if connected to PC
  //delay(1000);
  //rainbow(); //Attention: Switches all your LED - Only if your Power supply can handle 2.6 Amps - NOT if connected to PC
  //LEDS_rot(); //Attention: Switches all your LED - Only if your Power supply can handle 2.6 Amps - NOT if connected to PC
  //delay(500);
  LEDS_gruen_oben(); 
  //LEDS_blau();
  delay(500);
  //LEDS_gruen();
  LEDS_blau_oben();
  delay(500);
  //LEDS_weiss();
  //einzel();
  #ifdef SK6812_LEDS
    //delay(5000);
    //rainbowLoop();
    LEDS_aus();
    //delay(1500);
    
  #endif
  
  #ifdef mqtt_client_on //Connect to an MQTT Server that has been defined in credentials.h
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    if (client.connect("ESP32RotaryEncoder", "admin", "admin")) {
      client.publish("ROTARYENCODER/counter","0");
      client.publish("ROTARYENCODER/button","0");
      /*client.publish("ROTARYENCODER/ldr","0");
      client.publish("ROTARYENCODER/co2","0");
      client.publish("ROTARYENCODER/iaq","0");
      client.publish("ROTARYENCODER/iaq_acc","0");
      client.publish("ROTARYENCODER/temp","0");
      client.publish("ROTARYENCODER/humidity","0");
      client.publish("ROTARYENCODER/pressure","0");*/
      client.subscribe("inTopic");
      client.subscribe("LIGHTSWITCH/values");
    }
  #endif
}


void loop() {
  ArduinoOTA.handle(); //needs to be called on a regular basis to keep the wifi Update running
  #ifdef mqtt_client_on
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  #else
  #endif
  long now = millis();
  if(now - lastTrigger > (timeSeconds)){
    //thingspeak_send();
    //dimmen();
     
    rotary_loop();
    rotary_onButtonClick();
    lastTrigger = millis();
    
                              
  } 
  if(now - lastTrigger_LEDS > (updaterate_leds)){
     //LEDS_weiss();
     lastTrigger_LEDS = millis();
     //Serial.println("Led loop - Millis: ");Serial.print(lastTrigger_LEDS/1000);
     
  }
  if(now - lastTrigger_SENSOR > (updaterate_sensors)){
     //You may adapt the measurementrate and type to your needs
     lastTrigger_SENSOR = millis();
    /*#ifdef MPU9250_CONNECTED
      MPU9250_ABFRAGE();
    #endif
    #ifdef BMP280_CONNECTED
      BMP_Test();
    #endif
    #ifdef SGP30_CONNECTED
      co2_messen();
    #endif*/
    #ifdef LDR_CONNECTED
      ldr_messung();
    #endif 
    #ifdef BME680_CONNECTED
      BME680_ouptut();
    #endif 
    #ifdef mqtt_client_on
      mqtt_checker();
    #else
    #endif
    
  }
}

    #ifdef mqtt_client_on
      void mqtt_checker(void){
       if (lastcounter!=counter){ 
        char counterString[8]; dtostrf(counter, 1, 2, counterString);
        client.publish("ROTARYENCODER/counter", counterString);
       }
      if (BUTTON_CLICK != 0){
        char buttonString[8]; dtostrf(counter, 1, 2, buttonString);
        client.publish("ROTARYENCODER/button", buttonString);
      }
		  #ifdef LDR_CONNECTED
			  int LDR_UNTERSCHIED = brightness - lastBrightness;
			  LDR_UNTERSCHIED = abs(LDR_UNTERSCHIED);
			  //if (brightness != lastBrightness){
			  if (LDR_UNTERSCHIED > SENSITIVITY){
			  char ldrString[8]; dtostrf(brightness, 1, 2, ldrString);
				client.publish("ROTARYENCODER/ldr", ldrString);
			  }
		  #endif
		  #ifdef BME680_CONNECTED
			 char co2String[8]; dtostrf(bme680_co2, 1, 2, co2String);
			 client.publish("ROTARYENCODER/co2", co2String);
			 char iaQString[8]; dtostrf(bme680_iaq_value_measured, 1, 2, iaQString);
			 client.publish("ROTARYENCODER/iaq", iaQString);
			 char iaQACCString[8]; dtostrf(bme680_iaq_acc_measured, 1, 2, iaQACCString);
			 client.publish("ROTARYENCODER/iaq_acc", iaQACCString);
			 char tempString[8]; dtostrf(bme680_temp_measured, 1, 2, tempString);
			 client.publish("ROTARYENCODER/temp", tempString);
			 char humidityString[8]; dtostrf(bme680_humidity_measured, 1, 2, humidityString);
			 client.publish("ROTARYENCODER/humidity", humidityString);
			 char pressureString[8]; dtostrf(bme680_pressure_measured, 1, 2, pressureString);
			 client.publish("ROTARYENCODER/pressure", pressureString);
		  #endif
		}

  
    void reconnect() {
      // Loop until we're reconnected. TODO: This needs some workaround to avoid looping forever if MQTT Server is down
      while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect(mosquittoNAME, mosquittoUSER, mosquittoPASSWORD)) {
          Serial.println("connected");
          // Subscribe to a Topic
          client.subscribe("esp32/output");
        } 
		else {
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
          // Wait 5 seconds before retrying
          LEDS_rot();
          delay(5000);
        }
      }
    }
    void callback(char* topic, byte* message, unsigned int length) {
        Serial.print("Message arrived on topic: ");
        Serial.print(topic);
        Serial.print(". Message: ");
        String messageTemp;
        
        for (int i = 0; i < length; i++) {
          Serial.print((char)message[i]);
          messageTemp += (char)message[i];
        }
        Serial.println();
      
        if (String(topic) == "LIGHTSWITCH/values") {
          Serial.print("Changing output to ");
          if(messageTemp == "1"){
            Serial.println("on");
            //digitalWrite(LED_PIN, HIGH);
            LEDS_weiss();
          }
          else if(messageTemp == "0"){
            Serial.println("off");
            //digitalWrite(LED_PIN, LOW);
            LEDS_aus();
          }
          else{
            Serial.print("Wert: ");Serial.println(messageTemp);
            dimmen(messageTemp.toInt(), vorher_message);
            vorher_message = messageTemp.toInt();
          }
        }
      }
#endif



void fadeall() { for(int i = 0; i < ANZAHL_LEDS; i++) { leds[i].nscale8(250); } }


#ifdef WS2812_LEDS
  void rainbow()
  {
    // FastLED's built-in rainbow generator
    fill_rainbow( leds, ANZAHL_LEDS, gHue, 7);
    FastLED.show();  
  }
  void einzel(void){
      for(int i = 0; i < ANZAHL_LEDS; i++) {
        // Set the i'th led to red 
        if (counter < 20)      leds[i] = CRGB::Green;
        else if (counter < 30) leds[i] = CRGB::Yellow;
        else                      leds[i] = CRGB::Red;
        // Show the leds
        FastLED.show(); 
      }
  }
#endif

#ifdef SK6812_LEDS
  void rainbow(){
    static uint8_t hue;
   
    for(int i = 0; i < ANZAHL_LEDS; i++){
      leds[i] = CHSV((i * 256 / ANZAHL_LEDS) + hue, 255, 255);
    }
    FastLED.show();
    hue++;
  }
  
  void rainbowLoop(){
    long millisIn = millis();
    long loopTime = 5000; // 5 seconds
   
    while(millis() < millisIn + loopTime){
      rainbow();
      delay(5);
    }
  }
  //Schaltet Alle Leds nacheinander an /mit delay
  void einzel(void){
    for(int i = 0; i < ANZAHL_LEDS; i++){
      leds[i] = CRGBW(0, 0, 0, 75);
      FastLED.show();
      delay(250);
    }
  }
  //Schaltet Einzelne Leds gezielt an
  void einzel_led(uint8_t led_nummer, uint8_t red, uint8_t  green, uint8_t blue, uint8_t white){
    leds[led_nummer] = CRGBW(red, green, blue, white);
    FastLED.show();
  }

    //Fährt gleichzeitig alle LEDs an und bringt die Helligkeit auf den Zielwert
  void fade_white(uint8_t zielwert){
      Serial.print("Millis: ");           Serial.print(millis());
      Serial.print(" Aktueller Wert: ");   Serial.print(hue);
      Serial.print(" Zielwert:");         Serial.println(zielwert);
      if(hue < zielwert){
        hue++;  
      }
      else if (hue == zielwert){
        zielwert_erreicht = 1;
        Serial.println("Zielwert erreicht");
      }
      else{
        hue--;
      }
      fill_solid(leds, ANZAHL_LEDS, CRGBW(0,0,0,hue));
      FastLED.show();
  }
  
  //fährt Nacheinander alle Leds in einen bestimmten Wert
  //Dazu muss der Zielwert erreicht sein
  void fade_white_single(uint8_t startwert, uint8_t zielwert, int x){
      int dimmen_um_wert=10;
      Serial.print("Millis: ");          Serial.print(millis());
      Serial.print(" Diode:");          Serial.print(x);
      Serial.print(" Aktueller Wert: "); Serial.print(hue2);
      Serial.print(" Zielwert:");       Serial.println(zielwert);
      
      if(hue2 < zielwert){
        hue2+=dimmen_um_wert;  
        leds[x] = CRGBW(0, 0, 0, hue2);
      }
      else if (hue2 == zielwert){
        zielwert_erreicht = 1;
        hue2 = startwert; //rücksetzen des aktuellen Licht Werts für nächste LED
        Serial.println("Zielwert erreicht");
        Aktuelle_LED++;
      }
      else{
        hue2-=dimmen_um_wert;
        leds[x] = CRGBW(0, 0, 0, hue2);
      }    
      FastLED.show();
  }

  
#endif

/*******Dimms the led to the rotay Encoder Vaue****/
void dimmen(void){
  //fill_gradient(leds, 0, CHSV(lastcounter, 255,255), ANZAHL_LEDS, CHSV(counter,255,255), SHORTEST_HUES);    // up to 4 CHSV values
  fill_gradient(leds, 0, CHSV(lastcounter, 100,100), ANZAHL_LEDS, CHSV(counter,100,100), SHORTEST_HUES);    // up to 4 CHSV values
  FastLED.show();
}

/*******Dimms the led from a value to a value****/
void dimmen(int farbe_jetzt, int farbe_vorher){
  //fill_gradient(leds, 0, CHSV(farbe_jetzt, 255,255), ANZAHL_LEDS, CHSV(farbe_vorher,255,255), SHORTEST_HUES);    // up to 4 CHSV values
  fill_gradient(leds, 0, CHSV(farbe_jetzt, 100,100), ANZAHL_LEDS, CHSV(farbe_vorher,100,100), SHORTEST_HUES);    // up to 4 CHSV values
  FastLED.show();
}

/*******Turn all LED immediatly white ****/
void LEDS_weiss(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(55, 55, 55);  
    //CRGB color = CRGB(255, 255, 255);  // Only enable if you have a proper POWER SUPPLY connected   
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,0,0, 25);   
    //CRGBW color = CRGBW(0,0,0, 255); // Only enable if you have a proper POWER SUPPLY connected   
  #endif
  fill_solid(leds, ANZAHL_LEDS, color);
  FastLED.show();
}


/*******Turn all LED immediatly OFF ****/
void LEDS_aus(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 0, 0);  
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,0,0,0);
  #endif
  fill_solid(leds, ANZAHL_LEDS, color);
  FastLED.show();
}

/*******Turn all LED immediatly RED ****/
void LEDS_rot(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(20, 0, 0);  
    //CRGB color = CRGB(255, 0, 0);  // Only enable if you have a proper POWER SUPPLY connected   
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(25,0,0, 0);   
    //CRGBW color = CRGBW(255,0,0, 0);   
  #endif
  fill_solid(leds, ANZAHL_LEDS, color);// Only enable if you have a proper POWER SUPPLY connected   
  FastLED.show();
}

/*******Turn all LED immediatly BLUE ****/
void LEDS_blau(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 0, 20);  
    //CRGB color = CRGB(0, 0, 255); // Only enable if you have a proper POWER SUPPLY connected    
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,0,25, 0);   
    //CRGBW color = CRGBW(0,0,255, 0);// Only enable if you have a proper POWER SUPPLY connected    
  #endif
  fill_solid(leds, ANZAHL_LEDS, color);
  FastLED.show();
}

/*******Turn all LED immediatly GREEN ****/
void LEDS_gruen(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 20, 0);  
    //CRGB color = CRGB(0, 255, 0);  // Only enable if you have a proper POWER SUPPLY connected    
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,25,0, 0);   
    //CRGBW color = CRGBW(0,255,0, 0);   // Only enable if you have a proper POWER SUPPLY connected    
  #endif
  fill_solid(leds, ANZAHL_LEDS, color);
  FastLED.show();
}

/*******Turns the UPPER LED immediatly GREEN ****/
void LEDS_gruen_oben(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 20, 0);  
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,25,0, 0);   
  #endif
  fill_solid(leds2, OBEN_LEDS, color);
  FastLED.show();
}

/*******Turns the UPPER LED immediatly BLUE ****/
void LEDS_blau_oben(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 20, 0);  
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0, 0, 25, 0);   
  #endif
  fill_solid(leds2, OBEN_LEDS, color);
  FastLED.show();
 }

#ifdef MPU9250_CONNECTED
  void MPU_startup(void){
      // calibrate anytime you want to
      Serial.println("Accel Gyro calibration will start in 5sec.");
      Serial.println("Please leave the device still on the flat plane.");
      mpu.verbose(true);
      //delay(5000);
      mpu.calibrateAccelGyro();
      //print_calibration(); 
  }
  void MPU9250_ABFRAGE(void){
    if (mpu.update()) {
        yaw = mpu.getYaw();
        pitch = mpu.getPitch();
        roll = mpu.getRoll();
        avgyaw = (previousyaw + yaw)/2;
        avgpitch = (previouspitch + pitch)/2;
        avgroll = (previousroll+ roll)/2;
        previousyaw = yaw;
        previouspitch = pitch;
        previousroll = roll;
        Serial.print("YAW: "); Serial.print(avgyaw); Serial.print(", ");
        Serial.print("PITCH: ");Serial.print(avgpitch); Serial.print(", ");
        Serial.print("ROLL: ");Serial.print(avgroll);
    }
  }
  void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
  }

#endif    

#ifdef BMP280_CONNECTED
  void BMP_Start(void){
      bool status;
        status = bmp.begin(BMP_ADRESS,0x58); //alternative BMP Adress
        Serial.print("BMP check: "); Serial.print(status);
        if (!status) {
            Serial.println("No BMP280 I2C sensor, check wiring!");
        }
        else{
          Serial.println(" BMP280 gefunden");
          /* Default settings from datasheet. */
          bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        }
    }
    
    void BMP_Test(void){
          int temp =0;
          int pressure =0;             
          temp = bmp.readTemperature();
          pressure = bmp.readPressure()/ 100.0F;
          Serial.print(" Temp: ");
          Serial.print(bmp.readTemperature());
          Serial.print(" Druck: ");
          Serial.print(bmp.readPressure() / 100.0F);
    }

#endif



#ifdef SGP30_CONNECTED
  uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
  }

  void setup_sgp30(void){
    if (!sgp.begin()){
      Serial.println(" CO2 Sensor not found :(");
    }
    else{
      Serial.print("Found SGP30 serial #");
      Serial.print(sgp.serialnumber[0], HEX);
      Serial.print(sgp.serialnumber[1], HEX);
      Serial.println(sgp.serialnumber[2], HEX);
    }
  }

  void co2_messen(void){
    if (! sgp.IAQmeasure()) {
      Serial.println("CO2 Measurement failed");
    }
    else{
      Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
      Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");
    }
    

    if (! sgp.IAQmeasureRaw()) {
      Serial.println("CO2 Raw Measurement failed");
    }
    else{
      Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print(" \t");
      Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");
    }
    
  
    counter_co2messungen++;
    if (counter_co2messungen == 30) {
      counter_co2messungen = 0;
  
      uint16_t TVOC_base, eCO2_base;
      if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
        Serial.println("Failed to get baseline readings");
      }
      else{
        Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
        Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
      }
    }
  }
#endif

#ifdef LDR_CONNECTED
  void ldr_messung(void){
    WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b); // Workaround to USE ADC2 and Wifi at the same time
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV);// Workaround to USE ADC2 and Wifi at the same time
    lastBrightness = brightness;
    brightness = analogRead(LDRPin);
    Serial.print(", Brightness: " + String(brightness));
    /*if (abs(brightness - lastBrightness) >= SENSITIVITY) {
          lastBrightness = brightness;
          Serial.println("Brightness change to: " + String(brightness));
      }
  
      if (brightness < DARKNESS_THRESHOLD) {
          Serial.println("   Lights ON");
      }
      if (brightness > BRIGHTNESS_THRESHOLD){
          Serial.println("   Lights OFF");
      }*/
  }
#endif

#ifdef BME680_CONNECTED
  void checkIaqSensorStatus(void)
  {
    String output;
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } 
    else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      }
    }
  }
  void BME680_ouptut(void){
    String output;
    if (iaqSensor.run()) { // If new data is available
      output = ", BME680_Temp:" + String(iaqSensor.temperature);
      output += ", Feuchte: " + String(iaqSensor.humidity);
      output += ", Druck: " + String(iaqSensor.pressure/ 100.0F);
      output += ", IAQ: " + String(iaqSensor.staticIaq);
      output += ", CO2: " + String(iaqSensor.co2Equivalent);
      //output += ", Feuchte;" + String(iaqSensor.rawHumidity);
      //output += ", " + String(iaqSensor.gasResistance);
      output += ", IAq: " + String(iaqSensor.iaq);
      output += ", IAQ Genauigkeit: " + String(iaqSensor.iaqAccuracy);    
      output += ", VoC: " + String(iaqSensor.breathVocEquivalent);
      Serial.println(output);
      
      bme680_co2 = iaqSensor.co2Equivalent;
      bme680_iaq_value_measured = iaqSensor.iaq;
      bme680_iaq_acc_measured = int(iaqSensor.iaqAccuracy);
      bme680_temp_measured = iaqSensor.temperature;
      bme680_humidity_measured = int(iaqSensor.humidity);
      bme680_pressure_measured = int(iaqSensor.pressure/ 100.0F);
    } 
    else {
      checkIaqSensorStatus();
    }
  }
  
  void errLeds(void)
  {
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  }
#endif
