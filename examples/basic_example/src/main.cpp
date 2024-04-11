#include <Arduino.h>
#include "MUFFINS_CCSDS_Packets.h"

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1000);
  }

  // Create a CCSDS packet
  uint16_t apid = 100;
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  uint16_t data_values_count = 4;
  Converter data_values[data_values_count] = {{.f = 1.23}, {.i8 = 123}, {.i16 = 12345}, {.i32 = 1234567890}};
  String data_format[data_values_count] = {"float", "uint8", "uint16", "uint32"};
  uint16_t data_length = 0;

  byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

  Serial.println("CCSDS Packet: ");
  for (int i = 0; i < 12 + data_length; i++) // 12 bytes for primary and secondary headers and data field length
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();

  // Create a CCSDS string packet
  String data_string = "Hello, World!";
  data_length = 0;

  byte *string_packet = create_ccsds_string_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_string, data_length);

  Serial.println("CCSDS String Packet: ");
  for (int i = 0; i < 12 + data_length; i++)
  {
    Serial.print(string_packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();

  // Read a CCSDS packet
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  byte *read_data = new byte[244];
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  Serial.println("Read CCSDS Packet: ");
  Serial.println("APID: " + String(read_apid));
  Serial.println("Sequence Count: " + String(read_sequence_count));
  Serial.println("Epoch Time: " + String(read_epoch_time));
  Serial.println("Subseconds: " + String(read_subseconds));
  Serial.println("Data: ");
  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();

  // Read a CCSDS string packet
  parse_ccsds_telemetry(string_packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  Serial.println("Read CCSDS String Packet: ");
  Serial.println("APID: " + String(read_apid));
  Serial.println("Sequence Count: " + String(read_sequence_count));
  Serial.println("Epoch Time: " + String(read_epoch_time));
  Serial.println("Subseconds: " + String(read_subseconds));
  Serial.println("Data: ");
  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print((char)read_data[i]);
  }
  Serial.println();
  Serial.println();

  // Read an actual CCSDS packet from previous flight
  byte actual_packet[46] = {0x08, 0xc8, 0xc2, 0x1e, 0x00, 0x18, 0x66, 0x09, 0x3b, 0x04, 0x00, 0x00, 0x42, 0x64, 0x21, 0x4e, 0x41, 0xcc, 0xe9, 0x01, 0x46, 0x03, 0xfe, 0x3d, 0x46, 0x13, 0xb4, 0x71, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0xc2, 0xd6, 0x00, 0x00, 0x40, 0x10, 0x00, 0x00, 0x3a, 0x55};

  parse_ccsds_telemetry(actual_packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  Serial.println("Read Actual CCSDS Packet: ");
  Serial.println("APID: " + String(read_apid));
  Serial.println("Sequence Count: " + String(read_sequence_count));
  Serial.println("Epoch Time: " + String(read_epoch_time));
  Serial.println("Subseconds: " + String(read_subseconds));
  Serial.println("Data: ");
  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();

  // Get values from actual packet data field
  uint16_t actual_data_values_count = 8;
  Converter *actual_data_values = new Converter[actual_data_values_count];
  String actual_data_format[actual_data_values_count] = {"float", "float", "float", "float", "uint32", "uint32", "float", "float"};
  extract_ccsds_data_values(read_data, actual_data_values_count, actual_data_format, actual_data_values);

  Serial.println("Extracted Data Values: ");
  for (int i = 0; i < actual_data_values_count; i++)
  {
    if (actual_data_format[i] == "float")
    {
      Serial.println(actual_data_values[i].f);
    }
    else if (actual_data_format[i] == "uint32")
    {
      Serial.println(actual_data_values[i].i32);
    }
  }
  Serial.println();

  Serial.println("Done");

  // Clean up
  delete[] packet;
  delete[] string_packet;
  delete[] read_data;
  delete[] actual_data_values;
}

void loop()
{
}