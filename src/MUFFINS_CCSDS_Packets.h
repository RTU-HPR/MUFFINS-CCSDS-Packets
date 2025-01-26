#pragma once
#include <Arduino.h>

/**
 * @brief Union for converting between signed and unsigned integers, floats, and bytes
 */
union Converter
{
  int32_t i32;
  int16_t i16;
  int8_t i8;
  uint32_t ui32;
  uint16_t ui16;
  uint8_t ui8;
  float f;
  byte b[4];
};

float float_from_bytes(const byte byte1, const byte byte2, const byte byte3, const byte byte4);
uint8_t uint8_from_bytes(const byte byte1);
uint16_t uint16_from_bytes(const byte byte1, const byte byte2);
uint32_t uint32_from_bytes(const byte byte1, const byte byte2, const byte byte3, const byte byte4);
int8_t int8_from_bytes(const byte byte1);
int16_t int16_from_bytes(const byte byte1, const byte byte2);
int32_t int32_from_bytes(const byte byte1, const byte byte2, const byte byte3, const byte byte4);

/**
 * @brief Create a CCSDS primary header
 * @param apid Application ID
 * @param sequence_count Sequence count
 * @param data_length Length of data in packet
 * @return Pointer to primary header byte array
 * @note The primary header must be deleted after use
 */
byte *create_ccsds_primary_header(const uint16_t &apid, const uint16_t &sequence_count, const uint16_t &data_length);

/**
 * @brief Create a CCSDS secondary header
 * @param epoch_time Epoch time
 * @param subseconds Subseconds
 * @return Pointer to secondary header byte array
 * @note The secondary header must be deleted after use
 */
byte *create_ccsds_secondary_header(const uint32_t &epoch_time, const uint16_t &subseconds);

/**
 * @brief Create a full CCSDS telemetry packet with no checksum
 * @param apid Application ID
 * @param sequence_count Sequence count
 * @param epoch_time Epoch time
 * @param subseconds Subseconds (0-65535 fraction of a second)
 * @param data_values_count Number of data values
 * @param data_format Array of string of data types. Example: {"float", "uint8", "uint16", "uint32"}
 * @param data_values Pointer to data values array
 * @param data_length Length of data field
 * @note The CCSDS packet must be deleted after use
 */
byte *create_ccsds_telemetry_packet(const uint16_t &apid, const uint16_t &sequence_count, const uint32_t &epoch_time, const uint16_t &subseconds, const uint16_t &data_values_count, const String *data_format, const Converter *data_values, uint16_t &data_length);


/**
 * @brief Create a full CCSDS telecommand packet with no checksum
 * 
 * @param apid Application ID
 * @param sequence_count Sequence count
 * @param packet_id Packet ID
 * @param data_values_count Number of data values
 * @param data_format Array of string of data types. Example: {"float", "uint8", "uint16", "uint32"}
 * @param data_values Pointer to data values array
 * @param data_length Length of data field
 * @return Pointer to telecommand packet
 * @note The CCSDS packet must be deleted after use
 */
byte *create_ccsds_telecommand_packet(const uint16_t &apid, const uint16_t &sequence_count, const uint16_t &packet_id, const uint16_t &data_values_count, const String *data_format, const Converter *data_values, uint16_t &data_length);

/**
 * @brief Create a full CCSDS telecommand packet with no checksum
 * @param apid Application ID
 * @param sequence_count Sequence count
 * @param epoch_time Epoch time
 * @param subseconds Subseconds (0-65535 fraction of a second)
 * @param data_string Data string
 * @param data_length Length of data field
 */
byte *create_ccsds_string_telemetry_packet(const uint16_t &apid, const uint16_t &sequence_count, const uint32_t &epoch_time, const uint16_t &subseconds, const String &data_string, uint16_t &data_length);

/**
 * @brief Parse a CCSDS packet, extracting the primary header, secondary header, and data
 * @param packet Pointer to CCSDS packet byte array
 * @param apid Application ID
 * @param sequence_count Sequence count
 * @param epoch_time Epoch time
 * @param subseconds Subseconds
 * @param data Pointer to data byte array
 * @param data_length Length of data field
 * @note The data must be deleted after use
 */
void parse_ccsds_telemetry(const byte *packet, uint16_t &apid, uint16_t &sequence_count, uint32_t &epoch_time, uint16_t &subseconds, byte *&data, uint16_t &data_length);

/**
 * @brief Parse a CCSDS telecommand packet, extracting the primary header, secondary header, and data
 * @param packet Pointer to CCSDS packet byte array
 * @param apid Application ID
 * @param sequence_count Sequence count
 * @param packet_id Packet ID
 * @param data Pointer to data byte array
 * @param data_length Length of data in packet
 * @note The data must be deleted after use
 */
void parse_ccsds_telecommand(const byte *packet, uint16_t &apid, uint16_t &sequence_count, uint16_t &packet_id, byte *&data, uint16_t &data_length);

/**
 * @brief Read a CCSDS packet data, using the data format to convert the data to the correct data type
 * @param data Pointer to data byte array
 * @param data_values_count Number of data values
 * @param data_format Array of string of data types. Example: {"float", "uint8", "uint16", "uint32"}
 * @param data_values Pointer to data values array
 */
void extract_ccsds_data_values(const byte *data, const uint16_t &data_values_count, const String *data_format, Converter *data_values);