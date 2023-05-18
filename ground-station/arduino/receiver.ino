#include <SPI.h>
#include <LoRa.h>
byte readByte;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  while (!LoRa.begin(915E6)) {
    Serial.write(0xFA);
  }
}


void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // read packet
    for (int i = 0; i < packetSize; i++) {
      Serial.write(LoRa.read());
    }
  }

  if (Serial.available() > 0) {
    int indicator = Serial.read();
    if (indicator == 0) {
      Serial.write(0x66);
      return;
    } else {
      size_t packet_length = 0;
      uint8_t packet[1024];
      while (Serial.available()) {
        // Read data into buffer
        readByte = Serial.read();
        packet[packet_length] = readByte;
        packet_length++;
      }
      // Transmit buffer
      LoRa.beginPacket();
      LoRa.write(packet, packet_length);
      LoRa.endPacket(false);
      LoRa.receive();
    }
  }

// Used to test receive and transmit
//  if (millis() % 1000) {
//    uint8_t packet[] = {0x04, 0xa7, 0xab, 0x00, 0xd4 };
//    LoRa.beginPacket();
//    LoRa.write(packet, sizeof(packet));
//    LoRa.endPacket(false);
//    LoRa.receive();
//  }
}



    // size_t packet_length = 0;
    // uint8_t packet[1024];

    // if (indicator == 1) {
    //   float angle_x = Serial.read() << 24;
    //   angle_x = ((unsigned long) angle_x) | (Serial.Read() << 16);
    //   angle_x = ((unsigned long) angle_x) | (Serial.Read() << 8);
    //   angle_x = ((unsigned long) angle_x) | Serial.Read();

    //   float angle_y = Serial.read() << 24;
    //   angle_y = ((unsigned long) angle_y) | (Serial.Read() << 16);
    //   angle_y = ((unsigned long) angle_y) | (Serial.Read() << 8);
    //   angle_y = ((unsigned long) angle_y) | Serial.Read();
    // } else if (indicator == 2) {

    // }