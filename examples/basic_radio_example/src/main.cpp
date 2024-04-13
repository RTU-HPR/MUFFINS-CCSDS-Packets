#include <Arduino.h>
#include "MUFFINS_CCSDS_Packets.h"
#include "MUFFINS_Radio.h"

const int SENSOR_POWER_ENABLE_PIN = 17;
const int SPI0_RX = 4;
const int SPI0_TX = 3;
const int SPI0_SCK = 2;

Radio radio;

Radio::Config radio_config{
    .frequency = 434.0,
    .cs = 5,
    .dio0 = 8,
    .dio1 = 9,
    .reset = 7,
    .sync_word = 0xF4,
    .tx_power = 22,
    .spreading = 11,
    .coding_rate = 8,
    .signal_bw = 62.5,
    .frequency_correction = false,
    .spi_bus = &SPI,
};

// Actual CCSDS packet from previous flight
byte packet[46] = {0x08, 0xc8, 0xc2, 0x1e, 0x00, 0x18, 0x66, 0x09, 0x3b, 0x04, 0x00, 0x00, 0x42, 0x64, 0x21, 0x4e, 0x41, 0xcc, 0xe9, 0x01, 0x46, 0x03, 0xfe, 0x3d, 0x46, 0x13, 0xb4, 0x71, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0xc2, 0xd6, 0x00, 0x00, 0x40, 0x10, 0x00, 0x00, 0x3a, 0x55};

void setup()
{
  pinMode(SENSOR_POWER_ENABLE_PIN, OUTPUT_12MA);
  digitalWrite(SENSOR_POWER_ENABLE_PIN, HIGH);

  if (SPI.setRX(SPI0_RX) && SPI.setTX(SPI0_TX) && SPI.setSCK(SPI0_SCK))
  {
    SPI.begin();
  }

  if (!radio.begin(radio_config))
  {
    while (1)
      ;
  };
}

void loop()
{
  Serial.println("Sending packet!");
  radio.transmit_bytes((uint8_t*)packet, sizeof(packet));
  delay(5000);
}