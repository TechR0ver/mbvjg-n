#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 
#include "DHT.h"  

#define Relay1            D2
#define Relay2            D3
#define Relay3            D4
#define Relay4            D5

#define DHTPIN            D1

#define PIR               D8
//Selection pins for multiplexer module to switch between different sensors and give data on a single analog pin.
#define S0                D6
#define S1                D7


//Analog pin to read the incoming analog value from different sensors.
#define analogpin         A0

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com" //Adafruit Server
#define AIO_SERVERPORT  1883                   
#define AIO_USERNAME    "zackight"            // Username
#define AIO_KEY         "9dd74feebc7a4f6f9edd6762c1b3bcad"   // Auth Key

//WIFI CLIENT
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);


// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish CO2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gas");
Adafruit_MQTT_Publish Sound = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sound");
Adafruit_MQTT_Publish Motion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/motion");


Adafruit_MQTT_Subscribe Light1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/relay1"); // Feeds name should be same everywhere
Adafruit_MQTT_Subscribe Light2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay2");
Adafruit_MQTT_Subscribe Light3 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay3");
Adafruit_MQTT_Subscribe Light4 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay4");
Adafruit_MQTT_Subscribe Everything = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/everything");


/************ Necessary declaration for DHT11 ******************/
#define DHTTYPE           DHT11     // DHT 11 

DHT dht(DHTPIN, DHTTYPE);
uint32_t delayMS;


void MQTT_connect();

void setup() {
  Serial.begin(115200);
  WiFiManager wifiManager;
  wifiManager.autoConnect("TechroverAP");
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay3, HIGH); 
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(PIR , INPUT);
  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());
  dht.begin();
  mqtt.subscribe(&Light1);
  mqtt.subscribe(&Light3);
  mqtt.subscribe(&Light2);
  mqtt.subscribe(&Light4);
  mqtt.subscribe(&Everything);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay3, HIGH); 

  
}

void loop() {

  Serial.println("check point start loop OK!");
  MQTT_connect();
  
  Serial.println("check point end mqconnect OK!");
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(20000))) {
    if (subscription == &Light1) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light1.lastread);
      int Light1_State = atoi((char *)Light1.lastread);
      digitalWrite(Relay1, Light1_State);
      
    }
    if (subscription == &Light2) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light2.lastread);
      int Light2_State = atoi((char *)Light2.lastread);
      digitalWrite(Relay2, Light2_State);
    }
    if (subscription == &Light3) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light3.lastread);
      int Light3_State = atoi((char *)Light3.lastread);
      digitalWrite(Relay3, Light3_State);
    }
    if (subscription == &Light4) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light4.lastread);
      int Light4_State = atoi((char *)Light4.lastread);
      digitalWrite(Relay4, Light4_State);
      
    }
      if (subscription == &Everything) {
      Serial.print(F("Got: "));
      Serial.println((char *)Everything.lastread);
      int Everything_State = atoi((char *)Everything.lastread);
      digitalWrite(Relay1, Everything_State);
      digitalWrite(Relay2, Everything_State);
      digitalWrite(Relay3, Everything_State);
      digitalWrite(Relay4, Everything_State);            
    }
  }
    // Now we can publish stuff!
//  delay(300);
  int p = digitalRead(PIR);
    if (! Motion.publish(p)) {
    Serial.println(F("Motion Failed"));
  } else {
    Serial.println(F("Motion OK!"));
  }
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  Serial.print("Motion "); 
  Serial.print("...");
//  delay(300);
  int Value = analogRead(analogpin);
  Serial.print("analog data");
  Serial.print(Value);
////  if(Value>400)
////  Value=1;
////  else
////  Value=0;
  if (! Sound.publish(Value)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
//  delay(300);
//
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  Serial.print("C02 "); 
  Serial.print("...");
  int gValue = analogRead(analogpin);
  if (! CO2.publish(gValue)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
//
// delay(300);

//  digitalWrite(S0, HIGH);
//  digitalWrite(S1, HIGH);
//  Serial.print("Light "); Serial.println(analogRead(analogpin));
//  Serial.print("...");
//  int raw_light = analogRead(analogpin);
//  Value = map(raw_light,1024,0,0,100);
//  if (! Light.publish(Value)) {
//    Serial.println(F("Failed"));
//  } else {
//    Serial.println(F("OK!"));
//  }


  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  Serial.println("check point 1 OK!");
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  if (! Humidity.publish(h)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  if (! Temperature.publish(t)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  Serial.println("check point 2 OK!");
}

void MQTT_connect() {
  int8_t ret;
  Serial.println("check point 3 OK!");
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000); 
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  
}
