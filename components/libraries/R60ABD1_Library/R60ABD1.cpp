#include "R60ABD1.h"

// คำสั่งเริ่มต้นสำหรับเซ็นเซอร์
const uint8_t CMD_PRESENCE_STATUS[] = {0x53, 0x59, 0x80, 0x81, 0x00, 0x01, 0x0F, 0xBD, 0x54, 0x43};

R60ABD1::R60ABD1(HardwareSerial &serial) {
  sensorSerial = &serial;
}

void R60ABD1::begin() {
  // ตั้งค่า UART เรียบร้อยในไฟล์ .ino
  delay(100);
}

void R60ABD1::update() {
  uint8_t buffer[128];
  int index = 0;

  while (sensorSerial->available()) {
    buffer[index++] = sensorSerial->read();
    Serial.print(buffer[index - 1], HEX);
    Serial.print(" ");

    if (index >= 7 && buffer[index - 2] == 0x54 && buffer[index - 1] == 0x43) {
      processPacket(buffer, index);
      index = 0;
    }
  }
  Serial.println();
}

void R60ABD1::processPacket(const uint8_t *data, int length) {
  if (length < 7) return;
  if (data[2] == 0x80 && data[3] == 0x01) { // Presence Status
    presenceStatus = data[6];
  } else if (data[2] == 0x80 && data[3] == 0x02) { // Motion Status
    motionStatus = data[6];
  } else if (data[2] == 0x85 && data[3] == 0x02) { // Heartbeat Rate
    heartbeatRate = data[6];
  } else if (data[2] == 0x81 && data[3] == 0x02) { // Respiratory Rate
    respiratoryRate = data[6];
  } else if (data[2] == 0x84 && data[3] == 0x06) { // Sleep Quality
    sleepQuality = data[6];
  } else if (data[2] == 0x84 && data[3] == 0x01) { // Bed Status
    bedStatus = data[6];
  }
}

// void R60ABD1::processPacketOriginal(const uint8_t *data, int length) {
//   if (length < 7) return;

//   if (data[2] == 0x80 && data[3] == 0x01) { // Presence Status
//     presenceStatus = (data[6] == 0) ? "Clear" : "Detected";
//   } else if (data[2] == 0x80 && data[3] == 0x02) { // Motion Status
//     motionStatus = (data[6] == 0) ? "No Motion" : (data[6] == 1) ? "Motion Detected" : "Strong Motion";
//   } else if (data[2] == 0x85 && data[3] == 0x02) { // Heartbeat Rate
//     heartbeatRate = data[6];
//   } else if (data[2] == 0x81 && data[3] == 0x02) { // Respiratory Rate
//     respiratoryRate = data[6];
//   } else if (data[2] == 0x84 && data[3] == 0x06) { // Sleep Quality
//     sleepQuality = (data[6] == 0) ? "Poor" : (data[6] == 1) ? "Fair" : "Good";
//   } else if (data[2] == 0x84 && data[3] == 0x01) { // Bed Status
//     bedStatus = (data[6] == 0) ? "Out Bed" : "In Bed";
//   }
// }

int R60ABD1::getPresenceStatus() {
  return presenceStatus;
}

int R60ABD1::getMotionStatus() {
  return motionStatus;
}

int R60ABD1::getHeartbeatRate() {
  return heartbeatRate;
}

int R60ABD1::getRespiratoryRate() {
  return respiratoryRate;
}

int R60ABD1::getSleepQuality() {
  return sleepQuality;
}

int R60ABD1::getBedStatus() {
  return bedStatus;
}
