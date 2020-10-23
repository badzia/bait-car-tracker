#ifndef __BLOG_H_
#define __BLOG_H_

#include "MQTT.h"

#define MAX_LOG_SIZE 1024

class BLOG {

public:
typedef enum {
    LOG_LEVEL_ALL = 0,
    LOG_LEVEL_TRACE = 10,
    LOG_LEVEL_INFO = 20,
    LOG_LEVEL_ERROR = 30,
    LOG_LEVEL_NONE = 40,
} BLOG_LEVEL;
typedef enum {
    LOG_TO_CONSOLE = 0,
    LOG_TO_PARTICLE = 10,
    LOG_TO_MQTT = 20,
} BLOG_TO;

private:
    BLOG_LEVEL logLevel = LOG_LEVEL_NONE;
    bool logConsole = false;
    bool logMqtt = false;
    bool logParticle = false;
    String topicParticle = "BaitCarTracker/blog";
    String topicMqtt = "BaitCarTracker/blog";
    MQTT mqttClient;
    void logTo(BLOG_TO logTo);
public:
    BLOG();
    BLOG(BLOG_LEVEL logLevel);
    BLOG(BLOG_LEVEL logLevel, BLOG_TO logTo);
    ~BLOG();
    void setLogLevel(BLOG_LEVEL logLevel);
    void setLogTo(BLOG_TO logTo);
    void setParticleTopic(String topic);
    void setMqttTopic(String topic);
    void setMqttClient(MQTT client);
    void logt(String log);
    void logi(String log);
    void loge(String log);
    void logt(BLOG_TO logTo, String log);
    void logi(BLOG_TO logTo, String log);
    void loge(BLOG_TO logTo, String log);
    void log(BLOG_LEVEL logLevel, String log);
    void log(BLOG_LEVEL logLevel, BLOG_TO logTo, String log);
    const char* logLevel2String(BLOG_LEVEL logLevel) {
        static const char * enumLogLevel[] = { "ALL", "TRACE", "INFO", "ERROR", "NONE" };
        return logLevel < sizeof(enumLogLevel) ? enumLogLevel[logLevel] : "unknown";
    }
};

#endif  // __BLOG_H_
