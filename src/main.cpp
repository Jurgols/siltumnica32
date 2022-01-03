#include <Arduino.h>
#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
// #include <TelnetStream.h>
#include <ESPmDNS.h>
#include "Wire.h"
#include "time.h"
#include <AM232X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define lightPin 19
#define heatPin 18
#define fanOutPin 5
#define fainInPin 17
#define soilPin 34
#define levelPin 35
#define capPin 32
#define mistPin 16
#define waterPin 4
#define lightInterupt 2

WiFiMulti wifiMulti;
AM232X AM2320;
Adafruit_BME280 bme;
// Volumetric water content calculation parameters
float slope = 2.48; // slope from linear fit
float intercept = -0.72;

//InfuxDB device name
#define DEVICE "Siltumica_ESP32"
// InfluxDB  server url. Don't use localhost, always server name or ip address.
#define INFLUXDB_URL "https://europe-west1-1.gcp.cloud2.influxdata.com"
// InfluxDB 2 server or cloud API authentication token
#define INFLUXDB_TOKEN ***REMOVED***
// InfluxDB 2 organization id
#define INFLUXDB_ORG ***REMOVED***
// InfluxDB 2 bucket name
#define INFLUXDB_BUCKET "siltumnica"
//Riga time
#define TZ_INFO "EET-2EEST,M3.5.0/3,M10.5.0/4"
// InfluxDB client instance
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
// InfluxDB client instance for InfluxDB 1
//InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);
// Data point
Point sensor("microclimate");
char hostString[16] = {0};

const char* mqtt_server = "192.168.88.192";
//char mqtt_server[];
//mqtt message buffer size
#define MSG_BUFFER_SIZE  (50)
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
//init WiFi
WiFiClient espClient;
// init Mqtt client
PubSubClient mqtt_client(espClient);
//Deconstruct ctime struct
struct tm * timeinfo;


//Saturation vapor pressure
float calculate_svp(float temp){
  float leaf_temp = temp - 2.0;
  return (610.7*pow(10, ((7.5*leaf_temp)/(237.3+leaf_temp))))/1000;
}

//Air vapor pressure
float calculate_avp(float temp, float humidity){
  return calculate_svp(temp)*(humidity/100);
}

//Vapor pressure deficit
float calculate_vpd(float temp, float humidity){
  return calculate_svp(temp)-calculate_avp(temp, humidity);
}

// Soil sensor mesure voltage
float soil_voltage(uint8_t pin){
  int soil_adc = analogRead(pin);
  float voltage = (soil_adc / 4095.0) * 3.3;
  return voltage;
}
// vwc calculate precentage from voltage
float soil_vwc(float voltage){
  float soil_percent = constrain((((1.0 / voltage) * slope) + intercept) * 100, 0, 100);
  return soil_percent;
}

void reconnect();
void callback(char*, byte*, unsigned int);

void browseService(const char * service, const char * proto){
    Serial.printf("Browsing for service _%s._%s.local. ... ", service, proto);
    int n = MDNS.queryService(service, proto);
    if (n == 0) {
        Serial.println("no services found");
    } else {
        Serial.print(n);
        Serial.println(" service(s) found");
        for (int i = 0; i < n; ++i) {
            // Print details for each service found
            Serial.print("  ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(MDNS.hostname(i));
            Serial.print(" (");
            Serial.print(MDNS.IP(i));
            Serial.print(":");
            Serial.print(MDNS.port(i));
            Serial.println(")");
        }
    }
    Serial.println();
}


class InfluxData
{
  long interval;
  unsigned long previousMillis;
  public:
  InfluxData(int min_interval){
    previousMillis = 0;
    interval = min_interval * 60000; //convert minutes to milis
  }
  void Upload()
  {
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= interval)
    {
      AM2320.begin();
      timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
      sensor.clearFields();
      // Report RSSI of currently connected network
      sensor.addField("rssi", WiFi.RSSI());
      // Print what are we exactly writing
      Serial.print("Writing: ");
      Serial.println(client.pointToLineProtocol(sensor));
      sensor.addField("rssi", WiFi.RSSI());
      sensor.addField("air_temperature", AM2320.getTemperature());
      sensor.addField("air_humidity", AM2320.getHumidity());
      sensor.addField("VPD", calculate_vpd(AM2320.getTemperature(), AM2320.getHumidity()));
      sensor.addField("air_temperature_out", bme.readTemperature());
      sensor.addField("air_humidity_out", bme.readHumidity());
      //TelnetStream.print(sht.getHumidity());
      // sensor.addField("SVP", calculate_svp(AM2320.getTemperature()));
      // sensor.addField("AVP", calculate_avp(AM2320.getTemperature(), AM2320.getHumidity()));
      sensor.addField("soil_vwc", soil_vwc(soil_voltage(soilPin)));
      sensor.addField("soil_moisture_voltage", soil_voltage(soilPin));
      sensor.addField("mister_water_volume", map(analogRead(35),3847,2358,0, 100));
      // Write point
      if (!client.writePoint(sensor)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
      }
      previousMillis = currentMillis;
    }
  }

};
class Lights
{
  int end_hour;
  char light_pin;
  int start_hour;
  public:
  Lights(char out_pin, int start_h, int end_h)
  {
    light_pin = out_pin;
    start_hour = start_h;
    end_hour = end_h;
  }
  void Update()
  {
    
    if(start_hour >= end_hour)
    {
      if((timeinfo->tm_hour >= start_hour) || (timeinfo->tm_hour < end_hour))
      {
        digitalWrite(light_pin, HIGH);
        digitalWrite(fanOutPin, HIGH);
        digitalWrite(fainInPin, HIGH);
        digitalWrite(mistPin, HIGH);
      }
      else
      {
        //Turn on light when RPi is taking picture
        if (digitalRead(lightInterupt))
        {
          digitalWrite(lightPin, HIGH);
        }
        else
        {
          digitalWrite(lightPin, LOW);
          digitalWrite(fanOutPin, LOW);
          digitalWrite(fainInPin, LOW);
          digitalWrite(mistPin, LOW);
        }
    //Turn on at predefined interval
      }
    }
    else
    {
      if((timeinfo->tm_hour >=start_hour) && (timeinfo->tm_hour < end_hour))
      {
        digitalWrite(light_pin, HIGH);
        digitalWrite(fanOutPin, HIGH);
        digitalWrite(fainInPin, HIGH);
        digitalWrite(mistPin, HIGH);
      }
      else
      {
        //Turn on light when RPi is taking picture
        if (digitalRead(lightInterupt))
        {
          digitalWrite(lightPin, HIGH);
        }
        else
        {
          digitalWrite(lightPin, LOW);
          digitalWrite(fanOutPin, LOW);
          digitalWrite(fainInPin, LOW);
          digitalWrite(mistPin, LOW);
        }
      }

    }
  }
};

Lights growLights(lightPin, 7, 1);
InfluxData sensorData(5);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  //TelnetStream.begin();
  //Default pin states
  pinMode(lightPin, OUTPUT);
  pinMode(heatPin, OUTPUT);
  pinMode(fanOutPin, OUTPUT);
  pinMode(fainInPin, OUTPUT);
  pinMode(soilPin, INPUT);
  pinMode(mistPin, OUTPUT);
  pinMode(waterPin, OUTPUT);
  pinMode(lightInterupt, INPUT);
  pinMode(levelPin, INPUT);
  pinMode(capPin, OUTPUT);
  digitalWrite(capPin, HIGH);
  digitalWrite(lightPin, LOW);
  digitalWrite(heatPin, LOW);
  digitalWrite(fanOutPin, HIGH);
  digitalWrite(fainInPin, HIGH);
  digitalWrite(mistPin, LOW);
  digitalWrite(waterPin, LOW);
  //setup bme280
  if (! bme.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

  // Connect WiFi
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(***REMOVED***, ***REMOVED***);
  wifiMulti.addAP(***REMOVED***, ***REMOVED***);
  wifiMulti.addAP(***REMOVED***, ***REMOVED***);
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

   // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("siltumnica32");

  // No authentication by default
  ArduinoOTA.setPassword("esp8266");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (!MDNS.begin(hostString)) {
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.println("mDNS responder started");
  //Sync time with ntp
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
  // Add constant tags - only once
  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setCallback(callback);

}

unsigned long long recconectMillis = 0;
int reconnectInterval = 600000;
unsigned long long previousMillis = 0;

void loop() {
  unsigned long long currentMillis = millis();

  ArduinoOTA.handle();
  // browseService("mqtt", "tcp");
  
  if ((!mqtt_client.connected()) && (currentMillis - previousMillis >= reconnectInterval)) 
  {
    reconnect();
    previousMillis = currentMillis;
  }
  mqtt_client.loop();
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }
  time_t now;
  time(&now);
  if (digitalRead(lightInterupt))
  {
    Serial.print("Interupted");
  }
  timeinfo = localtime(&now);
  growLights.Update();
  sensorData.Upload();

  
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  if (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt_client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
    }
  }
}