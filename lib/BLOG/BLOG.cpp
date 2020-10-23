#include "BLOG.h"
#include <stdarg.h>

BLOG::BLOG() { BLOG(LOG_LEVEL_ALL); }

BLOG::BLOG(BLOG_LEVEL logLevel) { BLOG(logLevel, LOG_TO_CONSOLE); }

BLOG::BLOG(BLOG_LEVEL logLevel, BLOG_TO logTo) {
    this->logLevel = logLevel;
    this->logTo(logTo);
}

BLOG::~BLOG() {}

void BLOG::logTo(BLOG_TO logTo) {
    this->logConsole = (logTo & LOG_TO_CONSOLE);
    this->logParticle = (logTo & LOG_TO_PARTICLE);
    this->logMqtt = (logTo & LOG_TO_MQTT);
}

void BLOG::setLogLevel(BLOG_LEVEL logLevel) { this->logLevel = logLevel; }

void BLOG::setLogTo(BLOG_TO logTo) { this->logTo(logTo); }
    
void BLOG::setParticleTopic(String topic) { this->topicParticle = topic; }

void BLOG::setMqttTopic(String topic) { this->topicMqtt = topic; }

void BLOG::setMqttClient(MQTT client) { this->mqttClient = client; }


void BLOG::logt(String log) { this->log(LOG_LEVEL_TRACE, log); }
void BLOG::logi(String log) { this->log(LOG_LEVEL_INFO, log); }
void BLOG::loge(String log) { this->log(LOG_LEVEL_ERROR, log); }
void BLOG::logt(BLOG_TO logTo, String log) { this->log(LOG_LEVEL_TRACE, logTo, log); }
void BLOG::logi(BLOG_TO logTo, String log) { this->log(LOG_LEVEL_INFO, logTo, log); }
void BLOG::loge(BLOG_TO logTo, String log) { this->log(LOG_LEVEL_ERROR, logTo, log); }
void BLOG::log(BLOG_LEVEL logLevel, BLOG_TO logTo, String log) { 
    this->logTo(logTo); 
    this->log(logLevel, log); 
}
void BLOG::log(BLOG_LEVEL logLevel, String log) {
    char line[MAX_LOG_SIZE];
    String level = logLevel2String(logLevel);
    snprintf(line, sizeof(line), "%s [%lu] %s: %s", Time.format("%y/%m/%d %H:%M:%S").c_str(), millis(), level.c_str(), log.c_str());
    if(this->logConsole)  Serial.println(line);
    if(this->logParticle) Particle.publish(topicParticle, line, PRIVATE, NO_ACK);
    if(this->logMqtt) mqttClient.publish(topicMqtt.c_str(), line);
}


void BLOG::logtt(const char* log, ...) {
    char buf[MAX_LOG_SIZE];
    va_list args;
    va_start(args, log);
    snprintf(buf, sizeof(buf), log, args);
    va_end(args);
    this->logt(buf);
}