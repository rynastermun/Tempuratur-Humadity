#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Arduino_MQTT_Client.h>
#include <Wire.h>
#include <WiFi.h>
#include <DHT20.h>
#include <ThingsBoard.h>

#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12


constexpr char WIFI_SSID[] = "Mun";
constexpr char WIFI_PASSWORD[] = "manh0110";

constexpr char TOKEN[] = "exe1w9un02cyz8c3588c";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

volatile bool attributesChanged = false;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;

// void TaskTemperature_Humidity(void *pvParameters){
//   Wire.begin(GPIO_NUM_11, GPIO_NUM_12);
//   dht20.begin();
//   while(1){
//     dht20.read();

//     double temperature = dht20.getTemperature();
//     double humidity = dht20.getHumidity();

//     Serial.print("Temp: "); Serial.print(temperature); Serial.print(" *C ");
//     Serial.print(" Humidity: "); Serial.print(humidity); Serial.print(" %");
//     Serial.println();
    
//     tb.sendTelemetryData("temperature", temperature);
//     tb.sendTelemetryData("humidity", humidity);

//     vTaskDelay(5000);
//   }
//}

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();  
}

void loop() {
  delay(10);

  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }

    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

    // Serial.println("Subscribing for RPC...");
    // if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
    //   Serial.println("Failed to subscribe for RPC");
    //   return;
    // }

    // if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
    //   Serial.println("Failed to subscribe for shared attribute updates");
    //   return;
    // }

    Serial.println("Subscribe done");

  //   if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
  //     Serial.println("Failed to request for shared attributes");
  //     return;
  //   }
  // }

  // if (attributesChanged) {
  //   attributesChanged = false;
  //   tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
  // }

    if (millis() - previousDataSend > telemetrySendInterval) {
      previousDataSend = millis();

      dht20.read();
      
      float temperature = dht20.getTemperature();
      float humidity = dht20.getHumidity();

      if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT20 sensor!");
      } else {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");

        tb.sendTelemetryData("temperature", temperature);
        tb.sendTelemetryData("humidity", humidity);
      }

      tb.sendAttributeData("rssi", WiFi.RSSI());
      tb.sendAttributeData("channel", WiFi.channel());
      tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
      tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
      tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    }
  }
  tb.loop();
}
