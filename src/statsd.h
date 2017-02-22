#ifndef Statsd_h
#define Statsd_h

#include <Particle.h>
//#include "Arduino.h"
//#include "IPAddress.h"
//#include "Ethernet.h"
//#include "EthernetUDP.h"

class StatsD
{
public:
  StatsD(IPAddress ip, int port);
  void begin(int localPort=8888);
  void increment(const char *metric);
  void decrement(const char *metric);
  void timing(const char *metric, int ms);
  void gauge(const char *metric, int gaugeValue);
  void gauge(const char *metric, float gaugeValue);
  void gauge(const char *metric, double gaugeValue);
  void sets(const char *metric, int setsValue);
private:
  void _send(const char *metric, int value, const char *cmd);
  void _send(const char *metric, String value, const char *cmd);
  IPAddress _ip;
  int _port;
};

#endif
