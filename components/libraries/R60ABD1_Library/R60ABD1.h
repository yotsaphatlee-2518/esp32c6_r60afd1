#ifndef R60ABD1_H
#define R60ABD1_H

#include <Arduino.h>
#include <HardwareSerial.h>

class R60ABD1 {
  public:
    R60ABD1(HardwareSerial &serial);
    void begin();
    void update();

    int getPresenceStatus();
    int getMotionStatus();
    int getHeartbeatRate();
    int getRespiratoryRate();
    int getSleepQuality();
    int getBedStatus();

  private:
    HardwareSerial *sensorSerial;
    int presenceStatus;
    int motionStatus;
    int heartbeatRate;
    int respiratoryRate;
    int sleepQuality;
    int bedStatus;

    void processPacket(const uint8_t *data, int length);
};

#endif
