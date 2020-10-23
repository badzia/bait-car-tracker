#include <stdarg.h>
#include "BLOG.h"
#include "MQTT.h"

#define DEBUG_MQTT_SERIAL_OUTPUT 1

String mqtt_host = "mqtt.eclipse.org";
uint16_t mqtt_port = 1883;
String mqtt_user = "tracker";
String mqtt_password = "BaitCarTracker";

String deviceId = "";

uint32_t lastMillisMS = 0;
uint32_t publishIntervalMS = 0;
uint32_t mqtt_connectRetryMS = 5000;

MQTT mqttClient("server_name", 1883, mqtt_callback);
BLOG blog(BLOG::LOG_LEVEL_TRACE);

//#define LOGT(format, ...) blog.logtt(format, __VA_ARGS__)
//#define LOGI(format, ...) blog.logi(String::format(format, args))
//#define LOGE(format, ...) blog.loge(String::format(format, args))
void LOGT(const char* fmt, ...) {
  char buf[MAX_LOG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  blog.logt(buf);
}
void LOGI(const char* fmt, ...) {
  char buf[MAX_LOG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  blog.logi(buf);
}
void LOGE(const char* fmt, ...) {
  char buf[MAX_LOG_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  blog.loge(buf);
}

void setup() {
  Particle.variable("mqttServer", mqtt_host);
  Particle.function("setMQTTServer", mqtt_setServer);
  Particle.function("initMQTT", mqtt_init);
  Particle.function("restartMQTT", mqtt_restart);

  deviceId = System.deviceID();
  blog.setLogTo(BLOG::LOG_TO_CONSOLE);
  blog.setLogLevel(BLOG::LOG_LEVEL_INFO);
}

void loop() {
  if (Particle.connected()) {

    if (!mqttClient.isConnected()) {
        mqtt_init("");
        mqtt_reconnect();
      }
      mqttClient.loop();

      if (millis() - lastMillisMS >= publishIntervalMS) {
          lastMillisMS = millis();
//          Particle.publish("gps", pubbuf, PRIVATE);
//          snprintf(buf, sizeof(buf), "%f", gps.location.lat());
//sprintf(resultstr, "{\"temp1\":%0.1f,\"temp2\":%0.1f}", temp1, temp2);
      }
  }
}

void mqtt_reconnect() {
  LOGT("mqtt_reconnect()");
  while (!mqttClient.isConnected()) {
    LOGI("MQTT connect to %s as %s:%s", mqtt_host.c_str(),  mqtt_user.c_str(), mqtt_password.c_str());
    String id = "tracker_" + String(Time.now());
    if (mqttClient.connect(id.c_str(), mqtt_user.c_str(), mqtt_password.c_str())) {
      LOGI("connected to %s:%u", mqtt_host.c_str(), mqtt_port);
      mqttClient.publish("outTopic","hello world");
      mqttClient.subscribe("inTopic");
    } else {
      LOGI("MQTT connect failed. Retry in %d seconds", mqtt_connectRetryMS);
      delay(mqtt_connectRetryMS);
    }
  }
}

int mqtt_setServer(String host) {
  LOGT("mqtt_setServer(%s)", host);
  mqtt_host = host;
  return 0;
}

int mqtt_init(String command) {
  mqtt_user = "tracker_" + deviceId;
  char host[20];
  snprintf(host, sizeof(host), "%s", mqtt_host);
  mqttClient.setBroker(host, mqtt_port);
  return 0;
}

int mqtt_restart(String command) {

  return 0;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL; 

}


