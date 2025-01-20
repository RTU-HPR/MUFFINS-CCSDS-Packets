#include "MUFFINS_CCSDS_Packets.h"

byte *create_ccsds_primary_header(const uint16_t &apid, const uint16_t &sequence_count, const uint16_t &data_length)
{
  // Packet version number - 3 bits total
  byte PACKET_VERSION_NUMBER = 0;

  // Packet identification field - 13 bits total
  // Packet type (0 is telemetry, 1 is telecommand)- 1 bit
  // Secondary header flag (Always 1, as we will use it) - 1 bit
  // APID (Application Process Identifier) - 11 bits
  byte PACKET_TYPE = 0;
  byte SECONDARY_HEADER_FLAG = 1;
  int apid_binary = apid & 0x7FF; // keep only 11 bits
  int packet_identification_field = (PACKET_TYPE << 12) | (SECONDARY_HEADER_FLAG << 11) | apid_binary;

  // Packet Sequence Control - 16 bits total
  // Sequence flags (Always 11, as we are sending a single undivided packet) - 2 bits
  // Packet Sequence Count (Packet index)- 14 bits
  byte PACKET_SEQUENCE_FLAG = 3;                       // binary 11 is 3 in decimal
  int packet_sequence_count = sequence_count & 0x3FFF; // keep only 14 bits
  int packet_sequence_control = (PACKET_SEQUENCE_FLAG << 14) | packet_sequence_count;

  // Packet data length - 16 bits total
  // 16-bit field contains a length count that equals one fewer than the length of the data field
  byte *primary_header = new byte[6];
  primary_header[0] = (PACKET_VERSION_NUMBER << 5) | ((packet_identification_field >> 8) & 0x1F);
  primary_header[1] = packet_identification_field & 0xFF;
  primary_header[2] = (packet_sequence_control >> 8) & 0xFF;
  primary_header[3] = packet_sequence_control & 0xFF;
  primary_header[4] = (data_length >> 8) & 0xFF;
  primary_header[5] = data_length & 0xFF;

  return primary_header;
}

byte *create_ccsds_secondary_header(const uint32_t &epoch_time, const uint16_t &subseconds)
{
  // GPS epoch time - 4 bytes
  // Subseconds - 2 bytes

  // Create the secondary header
  byte *secondary_header = new byte[6];
  secondary_header[0] = (epoch_time >> 24) & 0xFF;
  secondary_header[1] = (epoch_time >> 16) & 0xFF;
  secondary_header[2] = (epoch_time >> 8) & 0xFF;
  secondary_header[3] = epoch_time & 0xFF;
  secondary_header[4] = (subseconds >> 8) & 0xFF;
  secondary_header[5] = subseconds & 0xFF;

  return secondary_header;
}

byte *create_ccsds_telemetry_packet(const uint16_t &apid, const uint16_t &sequence_count, const uint32_t &epoch_time, const uint16_t &subseconds, const uint16_t &data_values_count, const String *data_format, const Converter *data_values, uint16_t &data_length)
{
  byte packet_data[244]; // LoRa max packet length is 256 bytes, but taking out 12 bytes for primary and secondary headers

  data_length = 0;

  for (int i = 0; i < data_values_count; i++)
  {
    // Convert value to byte array
    if (data_format[i] == "uint8" || data_format[i] == "int8")
    {
      packet_data[data_length++] = data_values[i].b[0];
    }
    else if (data_format[i] == "uint16" || data_format[i] == "int16")
    {
      packet_data[data_length++] = data_values[i].b[1];
      packet_data[data_length++] = data_values[i].b[0];
    }
    else if (data_format[i] == "uint32" || data_format[i] == "int32" || data_format[i] == "float")
    {
      packet_data[data_length++] = data_values[i].b[3];
      packet_data[data_length++] = data_values[i].b[2];
      packet_data[data_length++] = data_values[i].b[1];
      packet_data[data_length++] = data_values[i].b[0];
    }
  }

  // Create full packet
  byte *primary_header = create_ccsds_primary_header(apid, sequence_count, data_length);
  byte *secondary_header = create_ccsds_secondary_header(epoch_time, subseconds);

  byte *packet = new byte[12 + data_length];

  // Add primary header, secondary header, and data to packet
  memcpy(packet, primary_header, 6);
  memcpy(packet + 6, secondary_header, 6);
  memcpy(packet + 12, packet_data, data_length);

  // Clean up
  delete[] primary_header;
  delete[] secondary_header;

  return packet;
}

byte *create_ccsds_string_telemetry_packet(const uint16_t &apid, const uint16_t &sequence_count, const uint32_t &epoch_time, const uint16_t &subseconds, const String &data_string, uint16_t &data_length)
{
  byte packet_data[244]; // LoRa max packet length is 256 bytes, but taking out 12 bytes for primary and secondary headers

  data_length = 0;

  for (int i = 0; i < data_string.length(); i++)
  {
    packet_data[data_length++] = data_string[i];
  }

  // Create full packet
  byte *primary_header = create_ccsds_primary_header(apid, sequence_count, data_length);
  byte *secondary_header = create_ccsds_secondary_header(epoch_time, subseconds);

  byte *packet = new byte[12 + data_length];

  // Add primary header, secondary header, and data to packet
  memcpy(packet, primary_header, 6);
  memcpy(packet + 6, secondary_header, 6);
  memcpy(packet + 12, packet_data, data_length);

  // Clean up
  delete[] primary_header;
  delete[] secondary_header;

  return packet;
}

void parse_ccsds_telemetry(const byte *packet, uint16_t &apid, uint16_t &sequence_count, uint32_t &epoch_time, uint16_t &subseconds, byte *&data, uint16_t &data_length)
{
  // Read primary header (6 bytes)
  byte packet_version_number = (packet[0] >> 5) & 0x07;
  byte packet_identification_field[2] = {static_cast<byte>(packet[0] & 0x1F), packet[1]};
  byte packet_sequence_control[2] = {packet[2], packet[3]};
  byte packet_data_length[2] = {packet[4], packet[5]};

  // Read secondary header (6 bytes)
  byte epoch_time_bytes[4] = {packet[6], packet[7], packet[8], packet[9]};
  byte subseconds_bytes[2] = {packet[10], packet[11]};

  // Convert bytes to integers
  apid = ((packet_identification_field[0] & 0x07) << 8) | packet_identification_field[1];
  sequence_count = ((packet_sequence_control[0] << 8) | packet_sequence_control[1]) & 0x3FFF;
  data_length = (packet_data_length[0] << 8) | packet_data_length[1];
  epoch_time = (epoch_time_bytes[0] << 24) | (epoch_time_bytes[1] << 16) | (epoch_time_bytes[2] << 8) | epoch_time_bytes[3];
  subseconds = (subseconds_bytes[0] << 8) | subseconds_bytes[1];

  // Read data
  // Data starts at byte 12 until the end of the packet
  if (data_length == 0)
  {
    data = nullptr;
    return;
  }
  data = new byte[data_length];
  memcpy(data, packet + 12, data_length);
}

void parse_ccsds_telecommand(const byte *packet, uint16_t &apid, uint16_t &sequence_count, uint16_t &packet_id, byte *&data, uint16_t &data_length)
{
  // Read primary header (6 bytes)
  byte packet_version_number = (packet[0] >> 5) & 0x07;
  byte packet_identification_field[2] = {static_cast<byte>(packet[0] & 0x1F), packet[1]};
  byte packet_sequence_control[2] = {packet[2], packet[3]};
  byte packet_data_length[2] = {packet[4], packet[5]};

  // Read packet id (2 bytes)
  byte packet_id_bytes[2] = {packet[6], packet[7]};

  // Convert bytes to integers
  apid = ((packet_identification_field[0] & 0x07) << 8) | packet_identification_field[1];
  sequence_count = ((packet_sequence_control[0] << 8) | packet_sequence_control[1]) & 0x3FFF;
  data_length = (packet_data_length[0] << 8) | packet_data_length[1];
  packet_id = (packet_id_bytes[0] << 8) | packet_id_bytes[1];

  // Read data
  // Data starts at byte 8 until the end of the packet
  if (data_length == 0)
  {
    data = NULL;
    return;
  }
  data = new byte[data_length];
  memcpy(data, packet + 8, data_length);
}

void extract_ccsds_data_values(const byte *data, const uint16_t &data_values_count, const String *data_format, Converter *data_values)
{
  uint16_t value_index = 0;
  uint16_t parsed_bytes = 0;
  for (int i = 0; i < data_values_count; i++)
  {
    // Read value from packet (MSB first)
    if (data_format[i] == "uint8" || data_format[i] == "int8")
    {
      data_values[value_index].b[0] = data[parsed_bytes++];
    }
    else if (data_format[i] == "uint16" || data_format[i] == "int16")
    {
      data_values[value_index].b[1] = data[parsed_bytes++];
      data_values[value_index].b[0] = data[parsed_bytes++];
    }
    else if (data_format[i] == "uint32" || data_format[i] == "int32" || data_format[i] == "float")
    {
      data_values[value_index].b[3] = data[parsed_bytes++];
      data_values[value_index].b[2] = data[parsed_bytes++];
      data_values[value_index].b[1] = data[parsed_bytes++];
      data_values[value_index].b[0] = data[parsed_bytes++];
    }
    value_index++;
  }
}