#include <stdarg.h>
#include "BLOG.h"
#include "MQTT.h"

#define DEBUG_MQTT_SERIAL_OUTPUT

String mqtt_host = "mqtt.eclipse.org";
uint16_t mqtt_port = 1883;
String mqtt_user = "tracker";
String mqtt_password = "baitcar_tracker";
const char *mqtt_topics[] = { "mqtt/data", "mqtt/+/data", "baitcartracker/command" };

String deviceId = "";

uint32_t lastMillisMS = 0;
uint32_t publishIntervalMS = 60000;
uint32_t mqtt_lastConnectMS = 0;
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
  Particle.function("arm", arm);
  Particle.function("disarm", disarm);
  Particle.function("setOutput1", setOutput1);
  Particle.function("setOutput2", setOutput2);
  Particle.function("setOutput3", setOutput3);
  Particle.function("setOutput4", setOutput4);
  Particle.function("setOutput5", setOutput5);
  Particle.function("setOutput6", setOutput6);
  Particle.function("setOutput7", setOutput7);
  Particle.function("setOutput8", setOutput8);
  Particle.function("info", info);
  Particle.function("test", test);

  deviceId = System.deviceID();
  blog.setLogTo(BLOG::LOG_TO_CONSOLE);
  blog.setLogLevel(BLOG::LOG_LEVEL_ALL);
}

void loop() {
  if (Particle.connected()) {
    if (!mqttClient.isConnected() && (millis() - mqtt_lastConnectMS >= mqtt_connectRetryMS)) {
        mqtt_lastConnectMS = millis();
        mqtt_init("");
        mqtt_reconnect();
      }
      mqttClient.loop();

      if (millis() - lastMillisMS >= publishIntervalMS) {
        lastMillisMS = millis();
        Particle.publish("I", "BaitCarTracker", PRIVATE);
//          Particle.publish("gps", pubbuf, PRIVATE);
//          snprintf(buf, sizeof(buf), "%f", gps.location.lat());
//sprintf(resultstr, "{\"temp1\":%0.1f,\"temp2\":%0.1f}", temp1, temp2);
      }
  }
}

void mqtt_reconnect() {
  LOGT("mqtt_reconnect()");
  if(!mqttClient.isConnected()) {
    LOGI("MQTT connecting to %s as %s:%s", mqtt_host.c_str(),  mqtt_user.c_str(), mqtt_password.c_str());
    String id = "tracker_" + String(Time.now());
    if (mqttClient.connect(id.c_str(), mqtt_user.c_str(), mqtt_password.c_str())) {
      LOGI("connected to %s:%u", mqtt_host.c_str(), mqtt_port);
      mqtt_subscribe();
    } else {
      LOGI("MQTT connect failed. Retry in %d seconds", mqtt_connectRetryMS);
      //delay(mqtt_connectRetryMS);
    }
  }
}

int mqtt_setServer(String host) {
  LOGT("mqtt_setServer(%s)", host.c_str());
  mqtt_host = host;
  char newhost[100];
  mqtt_host.toCharArray(newhost, mqtt_host.length()+1);
  mqttClient.setBroker(newhost, mqtt_port);
  return 0;
}

int mqtt_init(String command) {
  //mqtt_user = "tracker_" + deviceId;
  mqtt_setServer(mqtt_host);
  return 0;
}

int mqtt_restart(String command) {
  LOGT("mqtt_restart()");
  if(mqttClient.isConnected())
    mqttClient.disconnect();
  mqtt_reconnect();
  return 0;
}

void mqtt_subscribe() {
  LOGT("mqtt_subscribe()");
  for (u_int i = 0; i < sizeof(mqtt_topics)/sizeof(mqtt_topics[0]); i++) {
    LOGT("subscribe to %s", mqtt_topics[i]);
    mqttClient.subscribe(mqtt_topics[i]);
  }
}
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  LOGT("mqtt_callback(%s, payload, %d)", topic, length);
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL; 
  LOGI("mqtt_callback command=%s", p);

  //if(strcmp(topic, "mqtt/data") != 0) return;

  if(strcmp(p, "arm") == 0) {
    arm("");
  } else
  if(strcmp(p, "disarm") == 0) {
    disarm("");
  } else
  if(strstr(p, "setOutput")) {
    char * pch;
    pch = strtok (p, ",");
    if(pch != NULL) {
      byte n = pch[strlen(pch)-1] - '0';
      pch = strtok(NULL, ",");
      byte v = pch[0] - '0';
      setOutput(n, v);
    }
  } else
  if(strcmp(p, "test") == 0) {
    test("");
  }
}

int arm(String command) {
  LOGT("arm()");

  return 0;
}

int disarm(String command) {
  LOGT("disarm()");

  return 0;
}

int setOutput(byte number, byte value) {
  LOGT("setOutput(%d, %d)", number, value);

  return 0;
}

int setOutput1(String value) {
  LOGT("setOutput1(%s)", value.c_str());
  setOutput(1, atoi(value));
  return 0;
}

int setOutput2(String value) {
  LOGT("setOutput2(%s)", value.c_str());
  setOutput(2, atoi(value));
  return 0;
}

int setOutput3(String value) {
  LOGT("setOutput3(%s)", value.c_str());
  setOutput(3, atoi(value));
  return 0;
}

int setOutput4(String value) {
  LOGT("setOutput4(%s)", value.c_str());
  setOutput(4, atoi(value));
  return 0;
}

int setOutput5(String value) {
  LOGT("setOutput5(%s)", value.c_str());
  setOutput(5, atoi(value));
  return 0;
}

int setOutput6(String value) {
  LOGT("setOutput6(%s)", value.c_str());
  setOutput(6, atoi(value));
  return 0;
}

int setOutput7(String value) {
  LOGT("setOutput7(%s)", value.c_str());
  setOutput(7, atoi(value));
  return 0;
}

int setOutput8(String value) {
  LOGT("setOutput8(%s)", value.c_str());
  setOutput(8, atoi(value));
  return 0;
}

int info(String value) {
  LOGT("info()");

  return 0;
}

int test(String value) {
  LOGT("test()");
  Serial.println("TEST OK");
  Particle.publish("I", "TEST OK");
  mqttClient.publish("baitcartracker/"+deviceId, "TEST OK");
  LOGI("TEST OK");
  return 0;
}
