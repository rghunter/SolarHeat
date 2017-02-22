#include "statsd.h"
#include "string.h"

UDP statsd;

StatsD::StatsD(IPAddress ip, int port) {
  _ip = ip;
  _port = port;
}

void StatsD::begin(int localPort) {
    waitUntil(Particle.connected);
    statsd.begin(localPort);
}

void StatsD::increment(const char *metric) {
  StatsD::_send(metric, 1, "c");
}

void StatsD::decrement(const char *metric) {
  StatsD::_send(metric, -1, "c");
}

void StatsD::timing(const char *metric, int ms) {
  StatsD::_send(metric, ms, "ms");
}

void StatsD::gauge(const char *metric, int gaugeValue) {
  StatsD::_send(metric, gaugeValue, "g");
}

void StatsD::gauge(const char *metric, float gaugeValue) {
  StatsD::_send(metric, String(gaugeValue), "g");
}
void StatsD::gauge(const char *metric, double gaugeValue) {
  StatsD::_send(metric, String(gaugeValue), "g");
}

void StatsD::sets(const char *metric, int setsValue) {
  StatsD::_send(metric, setsValue, "s");
}
void StatsD::_send(const char *metric, int value, const char *cmd){
    char valueString[12];
    itoa(value, valueString,10);
    StatsD::_send(metric, valueString, cmd);
}

void StatsD::_send(const char *metric, String value, const char *cmd) {

  // Concatenate the parts of the final string
  char buffer[strlen(metric) + strlen(value) + strlen(cmd) + 2 + 1]; // +2 for : and | and add +1 for null
  strcpy(buffer, metric);
  strcat(buffer, ":");
  strcat(buffer, value);
  strcat(buffer, "|");
  strcat(buffer, cmd);

  const uint8_t* msg = (uint8_t*)buffer;

  //char txbuffer[] = "baseTemp.test:34|c";
  //statsd.sendPacket(txbuffer, sizeof(txbuffer), _ip, _port);

  statsd.beginPacket(_ip, _port);
  //statsd.write();
  statsd.write(msg, strlen(buffer));
  statsd.endPacket();
}
