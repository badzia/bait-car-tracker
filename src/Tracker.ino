#include <stdarg.h>
#include "spark_wiring_arduino_constants.h"
#include "BLOG.h"
#include "MQTT.h"
#include "AssetTracker.h"
#include "ICM_20948.h"

#define RELEASENUMBER 2
#define VERSION "2.0"
#define APP "BaitCarTracker"
#define SPI_PORT SPI
#define CS_PIN 2

String ver = VERSION;
String app = APP;
String firmware = app+ " " + ver;
char deviceName[32] = "";
char deviceId[32] = "";

char mqtt_host[32] = "mqtt.eclipse.org";
uint16_t mqtt_port = 1883;
char mqtt_user[32] = "tracker";
char mqtt_password[32] = "baitcar_tracker";
char mqtt_server[50] = "";
const char *mqtt_topics[] = { "mqtt/data", "mqtt/+/data", "baitcartracker/command" };

const int IOpins[] = { D1, D2, D3, D4, D5, D6, D7, D8 };
float lastLat = 0;
float lastLon = 0;

uint32_t lastMillisMS = 0;
uint32_t loop_IntervalMS = 60000;
uint32_t mqtt_lastConnectMS = 0;
uint32_t mqtt_retryIntervalMS = 5000;
uint32_t arm_lastMillisMS = 0;
uint32_t arm_IntervalMS = 1000;
uint32_t workCounter = 0;
bool isArm = false;
bool isPendingDisarm = false;

MQTT mqttClient("mqtt.eclipse.org", 1883, mqtt_callback, 1024);
BLOG blog(BLOG::LOG_LEVEL_TRACE);
AssetTracker g = AssetTracker();
FuelGauge fuelGauge;
ICM_20948_SPI icm;

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
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  Particle.variable("version", ver);
  Particle.variable("firmware", firmware);
  Particle.variable("mqttServer", mqtt_server);
  Particle.variable("deviceName", deviceName);
  Particle.variable("deviceId", deviceId);
  Particle.variable("armed", isArm);
  Particle.variable("armInterval", arm_IntervalMS);
  Particle.variable("loopInterval", loop_IntervalMS);
  Particle.variable("mqttRetryInterval", mqtt_retryIntervalMS);
  Particle.variable("workCounter", workCounter);

  Particle.function("setMQTTServer", mqtt_setServer);
  Particle.function("setMQTTPort", mqtt_setPort);
  Particle.function("initMQTT", mqtt_init);
  Particle.function("restartMQTT", mqtt_restart);
  Particle.function("setLoopInterval", setLoopInterval);
  Particle.function("setArmInterval", setArmInterval);
  Particle.function("setMQTTRetryInterval", setMqttRetryInterval);
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
  Particle.function("load", load);
  Particle.function("save", save);
  Particle.function("test", test);

  String s = System.deviceID();
  s.toCharArray(deviceId, s.length()+1);

  blog.setLogTo(BLOG::LOG_TO_CONSOLE);
  blog.setLogLevel(BLOG::LOG_LEVEL_ALL);

  g.begin();
  g.gpsOn();

  SPI_PORT.begin();
  icm.begin( CS_PIN, SPI_PORT );
  LOGI("ICM status: %s", icm.statusString() );
// For future use
//  Particle.subscribe("particle/device/name", handlerDeviceName, MY_DEVICES);
//  waitFor(Particle.connected, 300000);
//  for (uint32_t ms = millis(); millis() - ms < 3000; Particle.process());
//  Particle.publish("particle/device/name");
  
  workCounter = 0;

  load("");
}
// For future use
//void handlerDeviceName(const char *topic, const char *data) {
//  LOGT("handlerDeviceName(%s, %s)", topic, data);
//  strncpy(deviceName, data, sizeof(deviceName)-1);
//}

void loop() {
  workCounter = workCounter + 1;
  //if (Particle.connected()) {
    g.updateGPS();
    if (!mqttClient.isConnected() && (millis() - mqtt_lastConnectMS >= mqtt_retryIntervalMS)) {
      mqtt_lastConnectMS = millis();
      LOGI("Loop reconnect");
      mqtt_init("");
      mqtt_reconnect();
    }
    mqttClient.loop();

    if (millis() - lastMillisMS >= loop_IntervalMS) {
      lastMillisMS = millis();
      LOGI("Loop publish %d", workCounter);
      Particle.publish("I", "BaitCarTracker", PRIVATE);
    }
    if ((isArm || isPendingDisarm) && (millis() - arm_lastMillisMS >= arm_IntervalMS)) {
      arm_lastMillisMS = millis();
      LOGI("Arm publish");
      info("");
      isPendingDisarm = false;
    }
  //}
}

void mqtt_reconnect() {
  LOGT("mqtt_reconnect()");
  if(!mqttClient.isConnected()) {
    LOGI("MQTT connecting to %s as %s:%s", mqtt_host,  mqtt_user, mqtt_password);
    String id = "tracker_" + String(Time.now());
    if (mqttClient.connect(id.c_str(), mqtt_user, mqtt_password)) {
      LOGI("connected to %s:%d", mqtt_host, mqtt_port);
      save("");
      mqtt_subscribe();
    } else {
      LOGI("MQTT connect failed. Retry in %d seconds", mqtt_retryIntervalMS);
    }
  }
}

int mqtt_setHost(String host) {
  LOGT("mqtt_setHost(%s)", host.c_str());
  host.toCharArray(mqtt_host, host.length()+1);
  mqttClient.setBroker(mqtt_host, mqtt_port);
  return 0;
}

int mqtt_setPort(String port) {
  LOGT("mqtt_setPort(%s)", port.c_str());
  mqtt_port = atoi(port.c_str());
  mqttClient.setBroker(mqtt_host, mqtt_port);
  return 0;
}

int mqtt_setServer(String server) {
  LOGT("mqtt_setServer(%s)", server.c_str());

  //tymczasowa kompatybilność z wersją 1
  return mqtt_setHost(server);
/*  
  char s[100];
  server.toCharArray(s, server.length()+1);
  
  char * pch;
  pch = strtok(s, ":");
  if(pch != NULL) {
      strncpy(mqtt_host, pch, sizeof pch);
      pch = strtok(NULL, ":");
      if(pch != NULL) {
        mqtt_port = atoi(pch);
      } else return -1;
  } else return -1;
  return 0;
*/  
}

int mqtt_init(String command) {
  LOGT("mqtt_init()");
  //zmienić user
  //mqtt_user = "tracker_" + deviceId;

  if (command.length() > 0) {
    char buf[50];
    command.toCharArray(buf, sizeof(buf));
    char * pch;
    pch = strtok(buf, "|");
    if(pch != NULL) {
      strncpy(mqtt_host, pch, sizeof(mqtt_host));
      pch = strtok(NULL, "/");
      if(pch != NULL) {
        strncpy(deviceName, pch, sizeof(deviceName));
      }
      LOGI("mqtt_init host: %s , deviceName: %s", mqtt_host, deviceName);
    }
  }
  mqtt_setServer(mqtt_host);

  //set server address as "host:port"
  memset(mqtt_server, 0, sizeof mqtt_server);
  strcat(mqtt_server, mqtt_host);
  strcat(mqtt_server, ":");
  char p[5];
  itoa(mqtt_port, p, 10);
  strcat(mqtt_server, p);

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
  
  char topic2[50] = "mqtt/";
  strcat(topic2, deviceName);
  strcat(topic2, "/data");

  if(strcmp(topic, topic2) != 0) {
    LOGI("mqtt_callback wrong topic");
    return;
  }

  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL; 
  
  LOGI("mqtt_callback command=%s", p);

  if(strcmp(p, "arm") == 0) {
    arm("");
  } else
  if(strcmp(p, "disarm") == 0) {
    disarm("");
  } else
  if(strstr(p, "setOutput")) {
    char * pch;
    pch = strtok (p, "/");
    if(pch != NULL) {
      pch = strtok(NULL, "/");
      byte n = pch[0] - '0';
      pch = strtok(NULL, "/");
      byte v = pch[0] - '0';
      setOutput(n, v);
    }
  } else
  if(strcmp(p, "info") == 0) {
    info("");
  } else
  if(strcmp(p, "test") == 0) {
    test("");
  }
}

int setLoopInterval(String value) {
  loop_IntervalMS = atoi(value);
  save("");
  return 0;
}

int setArmInterval(String value) {
  arm_IntervalMS = atoi(value);
  save("");
  return 0;
}

int setMqttRetryInterval(String value) {
  mqtt_retryIntervalMS = atoi(value);
  save("");
  return 0;
}

int arm(String command) {
  LOGT("arm()");
  isArm = true;
  LOGI("armed");
  save("");
  return 0;
}

int disarm(String command) {
  LOGT("disarm()");
  isPendingDisarm = true;
  isArm = false;
  LOGI("disarmed");
  save("");
  return 0;
}

int setOutput(byte number, byte value) {
  LOGT("setOutput(%d, %d)", number, value);
  pinMode(IOpins[number], INPUT);
  digitalWrite(IOpins[number], (value==0)?LOW:HIGH);
  LOGI("setOutput%d to %d", number, value);
  pinMode(IOpins[number], OUTPUT);
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
  String battery = getBattery();
  String signal = getSignal();
  String temperature = getTemperature();
  String state = getState();
  String light = getLight();
  String motion = getMotion();
  String inputs = getInputs();
  String gps = getGps();
  String data = String::format("{%s,%s,%s,%s,%s,%s,%s,%s}", gps.c_str(), battery.c_str(), signal.c_str(), temperature.c_str(), state.c_str(), inputs.c_str(), light.c_str(), motion.c_str());

  char infoTopic[50] = "mqtt/";
//zmienić na deviceId
  strcat(infoTopic, deviceName);
  strcat(infoTopic, "/info");

  LOGI("info() publish to %s", infoTopic);
  LOGI("info(): %s", data.c_str());

  mqttClient.publish(infoTopic, data.c_str());
  return 0;
}

String getBattery() {
  LOGT("getBattery()");
  String s = String::format("\"bat\":{\"voltage\":%.2f,\"charge\":%.2f}", fuelGauge.getVCell(), System.batteryCharge());
  LOGT("getBattery(): %s", s.c_str());
  return s;
}

String getSignal() {
  LOGT("getSignal()");
  CellularSignal sig = Cellular.RSSI();  
  String s = String::format("\"sig\":{\"radio\":%d,\"strengthV\":%.2f,\"sthrengthP\":%.2f,\"qualityV\":%.2f,\"qualityP\":%.2f}", sig.getAccessTechnology(), sig.getStrengthValue(), sig.getStrength(), sig.getQualityValue(), sig.getQuality());
  LOGT("getSignal(): %s", s.c_str());
  return s;
}

String getTemperature() {
  LOGT("getTemperature()");
  float temp = 0;
  if(icm.dataReady()){
    LOGI("icm data ready");
    icm.getAGMT();
    temp = icm.temp();
  } else {
    LOGI("icm data not ready yet");
  }
  String s = String::format("\"temp\":{\"cpu\":%.2f,\"board\":%.2f}", temp, 0);
  LOGT("getTemperature(): %s", s.c_str());
  return s;
}

String getState() {
  LOGT("getState()");
  String s = String::format("\"state\":{\"firmware\":\"%s\",\"arm\":%s,\"send\":%s,\"interval\":%d}", firmware.c_str(), (isArm?"true":"false"), "true", arm_IntervalMS);
  LOGT("getState(): %s", s.c_str());
  return s;
}

String getLight() {
  LOGT("getLight()");
  float ch0 = 0;
  float ch1 = 0;
  float lux = 0;
  String s = String::format("\"light\":{\"ch0\":%d,\"ch1\":%d,\"lux\":%.2f}", ch0, ch1, lux);
  LOGT("getLight(): %s", s.c_str());
  return s;
}

//!!!!!!!!!!!!!!!
String getMotion() {
  LOGT("getMotion()");
  float accx = 0;
  float accy = 0;
  float accz = 0;
  float gyrx = 0;
  float gyry = 0;
  float gyrz = 0;
  float magx = 0;
  float magy = 0;
  float magz = 0;

  if(icm.dataReady()){
    LOGI("icm data ready");
    icm.getAGMT();
    accx = icm.accX();
    accx = icm.accY();
    accx = icm.accZ();
    gyrx = icm.gyrX();
    gyry = icm.gyrY();
    gyrz = icm.gyrZ();
    magx = icm.magX();
    magy = icm.magY();
    magz = icm.magZ();
  } else {
    LOGI("icm data not ready yet");
  }
  String s = String::format("\"motion\":{\"accx\":%.2f,\"accy\":%.2f,\"accz\":%.2f,\"gyrox\":%.2f,\"gyroy\":%.2f,\"gyroz\":%.2f,\"magx\":%.2f,\"magy\":%.2f,\"magz\":%.2f}", accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz);
  LOGT("get(): %s", s.c_str());
  return s;
}

String getInputs() {
  LOGT("getInputs()");
  int32_t d1 = digitalRead(D1);
  int32_t d2 = digitalRead(D2);
  int32_t d3 = digitalRead(D3);
  int32_t d4 = digitalRead(D4);
  int32_t d5 = digitalRead(D5);
  int32_t d6 = digitalRead(D6);
  int32_t d7 = digitalRead(D7);
  int32_t d8 = digitalRead(D8);
  int32_t a1 = analogRead(A1);
  int32_t a2 = analogRead(A2);
  int32_t a3 = analogRead(A3);
  int32_t a4 = analogRead(A4);
  int32_t a5 = analogRead(A5);

  String s = String::format("\"inputs\":{\"d1\":%d,\"d2\":%d,\"d3\":%d,\"d4\":%d,\"d5\":%d,\"d6\":%d,\"d7\":%d,\"d8\":%d,\"a1\":%d,\"a2\":%d,\"a3\":%d,\"a4\":%d,\"a5\":%d}", d1, d2, d3, d4, d5, d6, d7, d8, a1, a2, a3, a4, a5);
  LOGT("getInputs(): %s", s.c_str());
  return s;
}

String getGps() {
  LOGT("getGps()");
  float lat = g.readLatDeg();
  float lon = g.readLonDeg();
  float speed = g.getSpeed();
  uint32_t timestamp = g.getGpsTimestamp();
  float head = calculateBearing(lat, lon, lastLat, lastLon);
  lastLat = lat;
  lastLon = lon;

  String s = String::format("\"gps\":{\"lat\":%f,\"lon\":%f,\"head\":%f,\"speed\":%f,\"timestamp\":%d}", lat, lon, head, speed, Time.now());
  LOGT("getGps(): %s", s.c_str());
  return s;
}

int test(String value) {
  LOGT("test()");
  Serial.println("TEST OK");
  Particle.publish("I", "TEST OK", PRIVATE);
  mqttClient.publish(String::format("baitcartracker/%s",deviceId), "TEST OK");
  LOGI("TEST OK");
  return 0;
}

float calculateBearing(float lat,float lon,float lat2,float lon2) {
  float teta1 = radians(lat);
  float teta2 = radians(lat2);
  float delta1 = radians(lat2-lat);
  float delta2 = radians(lon2-lon);

  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
  float brng = atan2(y,x);
  brng = degrees(brng);// radians to degrees
  brng = ( ((int)brng + 360) % 360 ); 

  return brng;
}

struct Config {
  uint8_t number;
  char version[5];
  char host[32];
  uint16_t port;
  uint8_t arm;
  uint32_t armInterval;
  uint32_t loopInterval;
  uint32_t mqttRetryInterval;
};


int save(String value) {
  LOGT("save()");
  int addr = 0;
  Config config = { RELEASENUMBER, "", "", mqtt_port, (isArm?1:0), arm_IntervalMS, loop_IntervalMS, mqtt_retryIntervalMS };
  strncpy(config.version, ver.c_str(), sizeof(config.version));
  strncpy(config.host, mqtt_host, sizeof(config.host));
  EEPROM.put(addr, config);
}

int load(String value) {
  LOGT("load()");
  int addr = 0;
  Config config;
  EEPROM.get(addr, config);

  if(config.number == RELEASENUMBER) {
    strncpy(mqtt_host, config.host, sizeof(config.host));
    mqtt_port = config.port;
    arm_IntervalMS = config.armInterval;
    loop_IntervalMS = config.loopInterval;
    mqtt_retryIntervalMS = config.mqttRetryInterval;
    isArm = (config.arm==1?true:false);
  } else {
    save("");
  }
}

