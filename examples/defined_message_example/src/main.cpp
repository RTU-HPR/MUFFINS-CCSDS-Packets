#include <Arduino.h>
#include "MUFFINS_CCSDS_Packets.h"
#include "MUFFINS_CCSDS_All_Messages.h"

void test_balloon_telecommand_acknowledgement()
{
  // Create Balloon telecommand acknowledgement
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  uint16_t received_packet_id = 123;

  uint8_t packet_length;
  byte *packet = BalloonTelemetry::TelecommandAckowledgement::create(packet_length, sequence_count, epoch_time, subseconds, received_packet_id);

  Serial.println("Created balloon Telecommand Acknowledgement Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon telecommand acknowledgement packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  uint16_t read_received_packet_id;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BalloonTelemetry::TelecommandAckowledgement::get_values(read_data, read_received_packet_id);

  Serial.println("Read Balloon Telecommand Acknowledgement Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BALLOON_TELECOMMAND_ACKNOWLEDGEMENT) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BALLOON_TELECOMMAND_ACKNOWLEDGEMENT == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Received Packet ID: " + String(received_packet_id) + " Read Received Packet ID: " + String(read_received_packet_id) + " Match: " + String(received_packet_id == read_received_packet_id));
  Serial.println("");
}

void test_balloon_system_status()
{
  // Create Balloon system status
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  uint16_t uptime = 123;
  float battery_voltage = 12.3;
  uint8_t uplink_rssi = 123;
  uint16_t error_binary_string = 0b1111000011110000;
  uint8_t flight_mode = 1;
  uint8_t gps_lock = 1;
  uint8_t uplink_status = 1;

  uint8_t packet_length;
  byte *packet = BalloonTelemetry::SystemStatus::create(packet_length, sequence_count, epoch_time, subseconds, uptime, battery_voltage, uplink_rssi, error_binary_string, flight_mode, gps_lock, uplink_status);

  Serial.println("Created balloon System Status Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon system status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  uint16_t read_uptime;
  float read_battery_voltage;
  uint8_t read_uplink_rssi;
  uint16_t read_error_binary_string;
  uint8_t read_flight_mode;
  uint8_t read_gps_lock;
  uint8_t read_uplink_status;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BalloonTelemetry::SystemStatus::get_values(read_data, read_uptime, read_battery_voltage, read_uplink_rssi, read_error_binary_string, read_flight_mode, read_gps_lock, read_uplink_status);

  Serial.println("Read Balloon System Status Packet: ");
  
  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BALLOON_SYSTEM_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BALLOON_SYSTEM_STATUS == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Uptime: " + String(uptime) + " Read Uptime: " + String(read_uptime) + " Match: " + String(uptime == read_uptime));
  Serial.println("Original Battery Voltage: " + String(battery_voltage) + " Read Battery Voltage: " + String(read_battery_voltage) + " Match: " + String(battery_voltage == read_battery_voltage));
  Serial.println("Original Uplink RSSI: " + String(uplink_rssi) + " Read Uplink RSSI: " + String(read_uplink_rssi) + " Match: " + String(uplink_rssi == read_uplink_rssi));
  Serial.println("Original Error Binary String: " + String(error_binary_string, BIN) + " Read Error Binary String: " + String(read_error_binary_string, BIN) + " Match: " + String(error_binary_string == read_error_binary_string));
  Serial.println("Original Flight Mode: " + String(flight_mode) + " Read Flight Mode: " + String(read_flight_mode) + " Match: " + String(flight_mode == read_flight_mode));
  Serial.println("Original GPS Lock: " + String(gps_lock) + " Read GPS Lock: " + String(read_gps_lock) + " Match: " + String(gps_lock == read_gps_lock));
  Serial.println("Original Uplink Status: " + String(uplink_status) + " Read Uplink Status: " + String(read_uplink_status) + " Match: " + String(uplink_status == read_uplink_status));
  Serial.println("");
}

void test_balloon_configuration()
{
  // Create Balloon configuration
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float lora_frequency = 123.456;
  CCSDS_Enums::LoRa_TX_Power lora_tx_power = CCSDS_Enums::LoRa_TX_Power::TX_POWER_20;
  CCSDS_Enums::LoRa_Spreading_Factor lora_spreading_factor = CCSDS_Enums::LoRa_Spreading_Factor::SF_7;
  CCSDS_Enums::LoRa_Bandwidth lora_bandwidth = CCSDS_Enums::LoRa_Bandwidth::BW_125;
  CCSDS_Enums::LoRa_Coding_Rate lora_coding_rate = CCSDS_Enums::LoRa_Coding_Rate::CR_4_5;
  float barometer_reference_pressure = 1013.25;

  uint8_t packet_length;
  byte *packet = BalloonTelemetry::Configuration::create(packet_length, sequence_count, epoch_time, subseconds, lora_frequency, lora_tx_power, lora_spreading_factor, lora_bandwidth, lora_coding_rate, barometer_reference_pressure);

  Serial.println("Created balloon Configuration Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_lora_frequency;
  CCSDS_Enums::LoRa_TX_Power read_lora_tx_power;
  CCSDS_Enums::LoRa_Spreading_Factor read_lora_spreading_factor;
  CCSDS_Enums::LoRa_Bandwidth read_lora_bandwidth;
  CCSDS_Enums::LoRa_Coding_Rate read_lora_coding_rate;
  float read_barometer_reference_pressure;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BalloonTelemetry::Configuration::get_values(read_data, read_lora_frequency, read_lora_tx_power, read_lora_spreading_factor, read_lora_bandwidth, read_lora_coding_rate, read_barometer_reference_pressure);

  Serial.println("Read Balloon Configuration Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BALLOON_CONFIGURATION) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BALLOON_CONFIGURATION == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original LoRa Frequency: " + String(lora_frequency) + " Read LoRa Frequency: " + String(read_lora_frequency) + " Match: " + String(lora_frequency == read_lora_frequency));
  Serial.println("Original LoRa TX Power: " + String(static_cast<uint8_t>(lora_tx_power)) + " Read LoRa TX Power: " + String(static_cast<uint8_t>(read_lora_tx_power)) + " Match: " + String(static_cast<uint8_t>(lora_tx_power) == static_cast<uint8_t>(read_lora_tx_power)));
  Serial.println("Original LoRa Spreading Factor: " + String(static_cast<uint8_t>(lora_spreading_factor)) + " Read LoRa Spreading Factor: " + String(static_cast<uint8_t>(read_lora_spreading_factor)) + " Match: " + String(static_cast<uint8_t>(lora_spreading_factor) == static_cast<uint8_t>(read_lora_spreading_factor)));
  Serial.println("Original LoRa Bandwidth: " + String(static_cast<uint8_t>(lora_bandwidth)) + " Read LoRa Bandwidth: " + String(static_cast<uint8_t>(read_lora_bandwidth)) + " Match: " + String(static_cast<uint8_t>(lora_bandwidth) == static_cast<uint8_t>(read_lora_bandwidth)));
  Serial.println("Original LoRa Coding Rate: " + String(static_cast<uint8_t>(lora_coding_rate)) + " Read LoRa Coding Rate: " + String(static_cast<uint8_t>(read_lora_coding_rate)) + " Match: " + String(static_cast<uint8_t>(lora_coding_rate) == static_cast<uint8_t>(read_lora_coding_rate)));
  Serial.println("Original Barometer Reference Pressure: " + String(barometer_reference_pressure) + " Read Barometer Reference Pressure: " + String(read_barometer_reference_pressure) + " Match: " + String(barometer_reference_pressure == read_barometer_reference_pressure));
  Serial.println("");
}

void test_balloon_location()
{
  // Create Balloon location
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float latitude = 12.345;
  float longitude = 123.45;
  float gps_altitude = 1234.5;
  float barometer_altitude = 1234.5;
  uint8_t satellites = 12;

  uint8_t packet_length;
  byte *packet = BalloonTelemetry::Location::create(packet_length, sequence_count, epoch_time, subseconds, latitude, longitude, gps_altitude, barometer_altitude, satellites);

  Serial.println("Created balloon Location Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon location packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_latitude;
  float read_longitude;
  float read_gps_altitude;
  float read_barometer_altitude;
  uint8_t read_satellites;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BalloonTelemetry::Location::get_values(read_data, read_latitude, read_longitude, read_gps_altitude, read_barometer_altitude, read_satellites);

  Serial.println("Read Balloon Location Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BALLOON_LOCATION) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BALLOON_LOCATION == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Latitude: " + String(latitude) + " Read Latitude: " + String(read_latitude) + " Match: " + String(latitude == read_latitude));
  Serial.println("Original Longitude: " + String(longitude) + " Read Longitude: " + String(read_longitude) + " Match: " + String(longitude == read_longitude));
  Serial.println("Original GPS Altitude: " + String(gps_altitude) + " Read GPS Altitude: " + String(read_gps_altitude) + " Match: " + String(gps_altitude == read_gps_altitude));
  Serial.println("Original Barometer Altitude: " + String(barometer_altitude) + " Read Barometer Altitude: " + String(read_barometer_altitude) + " Match: " + String(barometer_altitude == read_barometer_altitude));
  Serial.println("Original Satellites: " + String(satellites) + " Read Satellites: " + String(read_satellites) + " Match: " + String(satellites == read_satellites));
  Serial.println("");
}

void test_balloon_rwc_status()
{
  // Create Balloon RWC status
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float heading = 123.45;
  int16_t angular_velocity_x = 123;
  int16_t angular_velocity_y = 123;
  int16_t angular_velocity_z = 123;
  float motor_angular_velocity = 123.45;
  float rwc_battery_voltage = 12.3;
  int8_t rwc_temperature = 12;

  uint8_t packet_length;
  byte *packet = BalloonTelemetry::RwcStatus::create(packet_length, sequence_count, epoch_time, subseconds, heading, angular_velocity_x, angular_velocity_y, angular_velocity_z, motor_angular_velocity, rwc_battery_voltage, rwc_temperature);

  Serial.println("Created balloon RWC Status Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon RWC status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_heading;
  int16_t read_angular_velocity_x;
  int16_t read_angular_velocity_y;
  int16_t read_angular_velocity_z;
  float read_motor_angular_velocity;
  float read_rwc_battery_voltage;
  int8_t read_rwc_temperature;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BalloonTelemetry::RwcStatus::get_values(read_data, read_heading, read_angular_velocity_x, read_angular_velocity_y, read_angular_velocity_z, read_motor_angular_velocity, read_rwc_battery_voltage, read_rwc_temperature);

  Serial.println("Read Balloon RWC Status Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BALLOON_SUBSYSTEM_1_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BALLOON_SUBSYSTEM_1_STATUS == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Heading: " + String(heading) + " Read Heading: " + String(read_heading) + " Match: " + String(heading == read_heading));
  Serial.println("Original Angular Velocity X: " + String(angular_velocity_x) + " Read Angular Velocity X: " + String(read_angular_velocity_x) + " Match: " + String(angular_velocity_x == read_angular_velocity_x));
  Serial.println("Original Angular Velocity Y: " + String(angular_velocity_y) + " Read Angular Velocity Y: " + String(read_angular_velocity_y) + " Match: " + String(angular_velocity_y == read_angular_velocity_y));
  Serial.println("Original Angular Velocity Z: " + String(angular_velocity_z) + " Read Angular Velocity Z: " + String(read_angular_velocity_z) + " Match: " + String(angular_velocity_z == read_angular_velocity_z));
  Serial.println("Original Motor Angular Velocity: " + String(motor_angular_velocity) + " Read Motor Angular Velocity: " + String(read_motor_angular_velocity) + " Match: " + String(motor_angular_velocity == read_motor_angular_velocity));
  Serial.println("Original RWC Battery Voltage: " + String(rwc_battery_voltage) + " Read RWC Battery Voltage: " + String(read_rwc_battery_voltage) + " Match: " + String(rwc_battery_voltage == read_rwc_battery_voltage));
  Serial.println("Original RWC Temperature: " + String(rwc_temperature) + " Read RWC Temperature: " + String(read_rwc_temperature) + " Match: " + String(rwc_temperature == read_rwc_temperature));
  Serial.println("");
}

void test_payload_telecommand_acknowledgement()
{
  // Create Payload telecommand acknowledgement
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  uint16_t received_packet_id = 123;

  uint8_t packet_length;
  byte *packet = PayloadTelemetry::TelecommandAckowledgement::create(packet_length, sequence_count, epoch_time, subseconds, received_packet_id);

  Serial.println("Created payload Telecommand Acknowledgement Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Payload telecommand acknowledgement packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  uint16_t read_received_packet_id;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  PayloadTelemetry::TelecommandAckowledgement::get_values(read_data, read_received_packet_id);

  Serial.println("Read Payload Telecommand Acknowledgement Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::PAYLOAD_TELECOMMAND_ACKNOWLEDGEMENT) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::PAYLOAD_TELECOMMAND_ACKNOWLEDGEMENT == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Received Packet ID: " + String(received_packet_id) + " Read Received Packet ID: " + String(read_received_packet_id) + " Match: " + String(received_packet_id == read_received_packet_id));
  Serial.println("");
}

void test_payload_system_status()
{
  // Create Payload system status
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  uint16_t uptime = 123;
  float battery_voltage = 12.3;
  uint8_t uplink_rssi = 123;
  uint16_t error_binary_string = 0b1111000011110000;
  uint8_t flight_mode = 1;
  uint8_t gps_lock = 1;
  uint8_t uplink_status = 1;

  uint8_t packet_length;
  byte *packet = PayloadTelemetry::SystemStatus::create(packet_length, sequence_count, epoch_time, subseconds, uptime, battery_voltage, uplink_rssi, error_binary_string, flight_mode, gps_lock, uplink_status);

  Serial.println("Created payload System Status Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon system status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  uint16_t read_uptime;
  float read_battery_voltage;
  uint8_t read_uplink_rssi;
  uint16_t read_error_binary_string;
  uint8_t read_flight_mode;
  uint8_t read_gps_lock;
  uint8_t read_uplink_status;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  PayloadTelemetry::SystemStatus::get_values(read_data, read_uptime, read_battery_voltage, read_uplink_rssi, read_error_binary_string, read_flight_mode, read_gps_lock, read_uplink_status);

  Serial.println("Read Payload System Status Packet: ");
  
  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::PAYLOAD_SYSTEM_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::PAYLOAD_SYSTEM_STATUS == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Uptime: " + String(uptime) + " Read Uptime: " + String(read_uptime) + " Match: " + String(uptime == read_uptime));
  Serial.println("Original Battery Voltage: " + String(battery_voltage) + " Read Battery Voltage: " + String(read_battery_voltage) + " Match: " + String(battery_voltage == read_battery_voltage));
  Serial.println("Original Uplink RSSI: " + String(uplink_rssi) + " Read Uplink RSSI: " + String(read_uplink_rssi) + " Match: " + String(uplink_rssi == read_uplink_rssi));
  Serial.println("Original Error Binary String: " + String(error_binary_string, BIN) + " Read Error Binary String: " + String(read_error_binary_string, BIN) + " Match: " + String(error_binary_string == read_error_binary_string));
  Serial.println("Original Flight Mode: " + String(flight_mode) + " Read Flight Mode: " + String(read_flight_mode) + " Match: " + String(flight_mode == read_flight_mode));
  Serial.println("Original GPS Lock: " + String(gps_lock) + " Read GPS Lock: " + String(read_gps_lock) + " Match: " + String(gps_lock == read_gps_lock));
  Serial.println("Original Uplink Status: " + String(uplink_status) + " Read Uplink Status: " + String(read_uplink_status) + " Match: " + String(uplink_status == read_uplink_status));
  Serial.println("");
}

void test_payload_configuration()
{
  // Create Payload configuration
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float lora_frequency = 123.456;
  CCSDS_Enums::LoRa_TX_Power lora_tx_power = CCSDS_Enums::LoRa_TX_Power::TX_POWER_20;
  CCSDS_Enums::LoRa_Spreading_Factor lora_spreading_factor = CCSDS_Enums::LoRa_Spreading_Factor::SF_7;
  CCSDS_Enums::LoRa_Bandwidth lora_bandwidth = CCSDS_Enums::LoRa_Bandwidth::BW_125;
  CCSDS_Enums::LoRa_Coding_Rate lora_coding_rate = CCSDS_Enums::LoRa_Coding_Rate::CR_4_5;
  float barometer_reference_pressure = 1013.25;

  uint8_t packet_length;
  byte *packet = PayloadTelemetry::Configuration::create(packet_length, sequence_count, epoch_time, subseconds, lora_frequency, lora_tx_power, lora_spreading_factor, lora_bandwidth, lora_coding_rate, barometer_reference_pressure);

  Serial.println("Created payload Configuration Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Payload configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_lora_frequency;
  CCSDS_Enums::LoRa_TX_Power read_lora_tx_power;
  CCSDS_Enums::LoRa_Spreading_Factor read_lora_spreading_factor;
  CCSDS_Enums::LoRa_Bandwidth read_lora_bandwidth;
  CCSDS_Enums::LoRa_Coding_Rate read_lora_coding_rate;
  float read_barometer_reference_pressure;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  PayloadTelemetry::Configuration::get_values(read_data, read_lora_frequency, read_lora_tx_power, read_lora_spreading_factor, read_lora_bandwidth, read_lora_coding_rate, read_barometer_reference_pressure);

  Serial.println("Read Payload Configuration Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::PAYLOAD_CONFIGURATION) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::PAYLOAD_CONFIGURATION == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original LoRa Frequency: " + String(lora_frequency) + " Read LoRa Frequency: " + String(read_lora_frequency) + " Match: " + String(lora_frequency == read_lora_frequency));
  Serial.println("Original LoRa TX Power: " + String(static_cast<uint8_t>(lora_tx_power)) + " Read LoRa TX Power: " + String(static_cast<uint8_t>(read_lora_tx_power)) + " Match: " + String(static_cast<uint8_t>(lora_tx_power) == static_cast<uint8_t>(read_lora_tx_power)));
  Serial.println("Original LoRa Spreading Factor: " + String(static_cast<uint8_t>(lora_spreading_factor)) + " Read LoRa Spreading Factor: " + String(static_cast<uint8_t>(read_lora_spreading_factor)) + " Match: " + String(static_cast<uint8_t>(lora_spreading_factor) == static_cast<uint8_t>(read_lora_spreading_factor)));
  Serial.println("Original LoRa Bandwidth: " + String(static_cast<uint8_t>(lora_bandwidth)) + " Read LoRa Bandwidth: " + String(static_cast<uint8_t>(read_lora_bandwidth)) + " Match: " + String(static_cast<uint8_t>(lora_bandwidth) == static_cast<uint8_t>(read_lora_bandwidth)));
  Serial.println("Original LoRa Coding Rate: " + String(static_cast<uint8_t>(lora_coding_rate)) + " Read LoRa Coding Rate: " + String(static_cast<uint8_t>(read_lora_coding_rate)) + " Match: " + String(static_cast<uint8_t>(lora_coding_rate) == static_cast<uint8_t>(read_lora_coding_rate)));
  Serial.println("Original Barometer Reference Pressure: " + String(barometer_reference_pressure) + " Read Barometer Reference Pressure: " + String(read_barometer_reference_pressure) + " Match: " + String(barometer_reference_pressure == read_barometer_reference_pressure));
  Serial.println("");
}

void test_payload_location()
{
  // Create Payload location
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float latitude = 12.345;
  float longitude = 123.45;
  float gps_altitude = 1234.5;
  float barometer_altitude = 1234.5;
  uint8_t satellites = 12;

  uint8_t packet_length;
  byte *packet = PayloadTelemetry::Location::create(packet_length, sequence_count, epoch_time, subseconds, latitude, longitude, gps_altitude, barometer_altitude, satellites);

  Serial.println("Created payload Location Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Payload location packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_latitude;
  float read_longitude;
  float read_gps_altitude;
  float read_barometer_altitude;
  uint8_t read_satellites;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  PayloadTelemetry::Location::get_values(read_data, read_latitude, read_longitude, read_gps_altitude, read_barometer_altitude, read_satellites);

  Serial.println("Read Payload Location Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::PAYLOAD_LOCATION) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::PAYLOAD_LOCATION == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Latitude: " + String(latitude) + " Read Latitude: " + String(read_latitude) + " Match: " + String(latitude == read_latitude));
  Serial.println("Original Longitude: " + String(longitude) + " Read Longitude: " + String(read_longitude) + " Match: " + String(longitude == read_longitude));
  Serial.println("Original GPS Altitude: " + String(gps_altitude) + " Read GPS Altitude: " + String(read_gps_altitude) + " Match: " + String(gps_altitude == read_gps_altitude));
  Serial.println("Original Barometer Altitude: " + String(barometer_altitude) + " Read Barometer Altitude: " + String(read_barometer_altitude) + " Match: " + String(barometer_altitude == read_barometer_altitude));
  Serial.println("Original Satellites: " + String(satellites) + " Read Satellites: " + String(read_satellites) + " Match: " + String(satellites == read_satellites));
  Serial.println("");
}

void test_payload_heater_container_status()
{
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float air_temperature = 12.3;
  int8_t barometer_temperature = 12;
  int8_t heatsink_temperature = 12;
  uint32_t pressure = 1234;
  int16_t proportional_value = 123;
  int16_t integral_value = 123;
  uint8_t duty_cycle = 123;
  uint8_t packet_length;

  byte *packet = PayloadTelemetry::HeatedContainerStatus::create(packet_length, sequence_count, epoch_time, subseconds, air_temperature, barometer_temperature, heatsink_temperature, pressure, proportional_value, integral_value, duty_cycle);

  Serial.println("Created payload Heated Container Status Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Payload heated container status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_air_temperature;
  int8_t read_barometer_temperature;
  int8_t read_heatsink_temperature;
  uint32_t read_pressure;
  int16_t read_proportional_value;
  int16_t read_integral_value;
  uint8_t read_duty_cycle;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  PayloadTelemetry::HeatedContainerStatus::get_values(read_data, read_air_temperature, read_barometer_temperature, read_heatsink_temperature, read_pressure, read_proportional_value, read_integral_value, read_duty_cycle);

  Serial.println("Read Payload Heated Container Status Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::PAYLOAD_SUBSYSTEM_1_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::PAYLOAD_SUBSYSTEM_1_STATUS == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Air Temperature: " + String(air_temperature) + " Read Air Temperature: " + String(read_air_temperature) + " Match: " + String(air_temperature == read_air_temperature));
  Serial.println("Original Barometer Temperature: " + String(barometer_temperature) + " Read Barometer Temperature: " + String(read_barometer_temperature) + " Match: " + String(barometer_temperature == read_barometer_temperature));
  Serial.println("Original Heatsink Temperature: " + String(heatsink_temperature) + " Read Heatsink Temperature: " + String(read_heatsink_temperature) + " Match: " + String(heatsink_temperature == read_heatsink_temperature));
  Serial.println("Original Pressure: " + String(pressure) + " Read Pressure: " + String(read_pressure) + " Match: " + String(pressure == read_pressure));
  Serial.println("Original Proportional Value: " + String(proportional_value) + " Read Proportional Value: " + String(read_proportional_value) + " Match: " + String(proportional_value == read_proportional_value));
  Serial.println("Original Integral Value: " + String(integral_value) + " Read Integral Value: " + String(read_integral_value) + " Match: " + String(integral_value == read_integral_value));
  Serial.println("Original Duty Cycle: " + String(duty_cycle) + " Read Duty Cycle: " + String(read_duty_cycle) + " Match: " + String(duty_cycle == read_duty_cycle));
  Serial.println("");
}

void test_base_telecommand_acknowledgement()
{
  // Create Base telecommand acknowledgement
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  uint16_t received_packet_id = 123;

  uint8_t packet_length;
  byte *packet = BaseStation1Telemetry::TelecommandAckowledgement::create(packet_length, sequence_count, epoch_time, subseconds, received_packet_id);

  Serial.println("Created base Telecommand Acknowledgement Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base telecommand acknowledgement packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  uint16_t read_received_packet_id;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BaseStation1Telemetry::TelecommandAckowledgement::get_values(read_data, read_received_packet_id);

  Serial.println("Read Base Telecommand Acknowledgement Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_TELECOMMAND_ACKNOWLEDGEMENT) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_TELECOMMAND_ACKNOWLEDGEMENT == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Received Packet ID: " + String(received_packet_id) + " Read Received Packet ID: " + String(read_received_packet_id) + " Match: " + String(received_packet_id == read_received_packet_id));
  Serial.println("");
}

void test_base_system_status()
{
  // Create Base system status
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  uint16_t uptime = 123;
  float battery_voltage = 12.3;
  uint8_t wifi_rssi = 123;
  uint16_t error_binary_string = 0b1111000011110000;
  uint8_t transmitting_allowed = 1;
  uint8_t packet_length;

  byte *packet = BaseStation1Telemetry::SystemStatus::create(packet_length, sequence_count, epoch_time, subseconds, uptime, battery_voltage, wifi_rssi, error_binary_string, transmitting_allowed);

  Serial.println("Created base System Status Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base system status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  uint16_t read_uptime;
  float read_battery_voltage;
  uint8_t read_wifi_rssi;
  uint16_t read_error_binary_string;
  uint8_t read_transmitting_allowed;

  byte *read_data;
  uint16_t read_data_length;
  
  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BaseStation1Telemetry::SystemStatus::get_values(read_data, read_uptime, read_battery_voltage, read_wifi_rssi, read_error_binary_string, read_transmitting_allowed);

  Serial.println("Read Base System Status Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_SYSTEM_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_SYSTEM_STATUS == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Uptime: " + String(uptime) + " Read Uptime: " + String(read_uptime) + " Match: " + String(uptime == read_uptime));
  Serial.println("Original Battery Voltage: " + String(battery_voltage) + " Read Battery Voltage: " + String(read_battery_voltage) + " Match: " + String(battery_voltage == read_battery_voltage));
  Serial.println("Original Wi-Fi RSSI: " + String(wifi_rssi) + " Read Wi-Fi RSSI: " + String(read_wifi_rssi) + " Match: " + String(wifi_rssi == read_wifi_rssi));
  Serial.println("Original Error Binary String: " + String(error_binary_string, BIN) + " Read Error Binary String: " + String(read_error_binary_string, BIN) + " Match: " + String(error_binary_string == read_error_binary_string));
  Serial.println("Original Transmitting Allowed: " + String(transmitting_allowed) + " Read Transmitting Allowed: " + String(read_transmitting_allowed) + " Match: " + String(transmitting_allowed == read_transmitting_allowed));
}

void test_base_configuration()
{
  // Create Base configuration
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float lora_frequency = 123.456;
  CCSDS_Enums::LoRa_TX_Power lora_tx_power = CCSDS_Enums::LoRa_TX_Power::TX_POWER_20;
  CCSDS_Enums::LoRa_Spreading_Factor lora_spreading_factor = CCSDS_Enums::LoRa_Spreading_Factor::SF_7;
  CCSDS_Enums::LoRa_Bandwidth lora_bandwidth = CCSDS_Enums::LoRa_Bandwidth::BW_125;
  CCSDS_Enums::LoRa_Coding_Rate lora_coding_rate = CCSDS_Enums::LoRa_Coding_Rate::CR_4_5;

  uint8_t packet_length;
  byte *packet = BaseStation1Telemetry::Configuration::create(packet_length, sequence_count, epoch_time, subseconds, lora_frequency, lora_tx_power, lora_spreading_factor, lora_bandwidth, lora_coding_rate);

  Serial.println("Created base Configuration Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_lora_frequency;
  CCSDS_Enums::LoRa_TX_Power read_lora_tx_power;
  CCSDS_Enums::LoRa_Spreading_Factor read_lora_spreading_factor;
  CCSDS_Enums::LoRa_Bandwidth read_lora_bandwidth;
  CCSDS_Enums::LoRa_Coding_Rate read_lora_coding_rate;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BaseStation1Telemetry::Configuration::get_values(read_data, read_lora_frequency, read_lora_tx_power, read_lora_spreading_factor, read_lora_bandwidth, read_lora_coding_rate);

  Serial.println("Read Base Configuration Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_CONFIGURATION) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_CONFIGURATION == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original LoRa Frequency: " + String(lora_frequency) + " Read LoRa Frequency: " + String(read_lora_frequency) + " Match: " + String(lora_frequency == read_lora_frequency));
  Serial.println("Original LoRa TX Power: " + String(static_cast<uint8_t>(lora_tx_power)) + " Read LoRa TX Power: " + String(static_cast<uint8_t>(read_lora_tx_power)) + " Match: " + String(static_cast<uint8_t>(lora_tx_power) == static_cast<uint8_t>(read_lora_tx_power)));
  Serial.println("Original LoRa Spreading Factor: " + String(static_cast<uint8_t>(lora_spreading_factor)) + " Read LoRa Spreading Factor: " + String(static_cast<uint8_t>(read_lora_spreading_factor)) + " Match: " + String(static_cast<uint8_t>(lora_spreading_factor) == static_cast<uint8_t>(read_lora_spreading_factor)));
  Serial.println("Original LoRa Bandwidth: " + String(static_cast<uint8_t>(lora_bandwidth)) + " Read LoRa Bandwidth: " + String(static_cast<uint8_t>(read_lora_bandwidth)) + " Match: " + String(static_cast<uint8_t>(lora_bandwidth) == static_cast<uint8_t>(read_lora_bandwidth)));
  Serial.println("Original LoRa Coding Rate: " + String(static_cast<uint8_t>(lora_coding_rate)) + " Read LoRa Coding Rate: " + String(static_cast<uint8_t>(read_lora_coding_rate)) + " Match: " + String(static_cast<uint8_t>(lora_coding_rate) == static_cast<uint8_t>(read_lora_coding_rate)));
  Serial.println("");
}

void test_base_location()
{
  // Create Base location
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float latitude = 12.345;
  float longitude = 123.45;
  float altitude = 1234.5;
  uint8_t satellites = 12;

  uint8_t packet_length;
  byte *packet = BaseStation1Telemetry::Location::create(packet_length, sequence_count, epoch_time, subseconds, latitude, longitude, altitude, satellites);

  Serial.println("Created base Location Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base location packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_latitude;
  float read_longitude;
  float read_altitude;
  uint8_t read_satellites;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BaseStation1Telemetry::Location::get_values(read_data, read_latitude, read_longitude, read_altitude, read_satellites);

  Serial.println("Read Base Location Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_LOCATION) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_LOCATION == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Latitude: " + String(latitude) + " Read Latitude: " + String(read_latitude) + " Match: " + String(latitude == read_latitude));
  Serial.println("Original Longitude: " + String(longitude) + " Read Longitude: " + String(read_longitude) + " Match: " + String(longitude == read_longitude));
  Serial.println("Original Altitude: " + String(altitude) + " Read Altitude: " + String(read_altitude) + " Match: " + String(altitude == read_altitude));
  Serial.println("Original Satellites: " + String(satellites) + " Read Satellites: " + String(read_satellites) + " Match: " + String(satellites == read_satellites));
  Serial.println("");
}

void test_base_rotator_status()
{
  // Create Base rotator status
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  float target_azimuth = 123.45;
  float target_elevation = 123.45;
  float current_azimuth = 123.45;
  float current_elevation = 123.45;
  uint8_t packet_length;

  byte *packet = BaseStation1Telemetry::RotatorStatus::create(packet_length, sequence_count, epoch_time, subseconds, target_azimuth, target_elevation, current_azimuth, current_elevation);

  Serial.println("Created base Rotator Status Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base rotator status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  float read_target_azimuth;
  float read_target_elevation;
  float read_current_azimuth;
  float read_current_elevation;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BaseStation1Telemetry::RotatorStatus::get_values(read_data, read_target_azimuth, read_target_elevation, read_current_azimuth, read_current_elevation);

  Serial.println("Read Base Rotator Status Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_SUBSYSTEM_1_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_SUBSYSTEM_1_STATUS == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Target Azimuth: " + String(target_azimuth) + " Read Target Azimuth: " + String(read_target_azimuth) + " Match: " + String(target_azimuth == read_target_azimuth));
  Serial.println("Original Target Elevation: " + String(target_elevation) + " Read Target Elevation: " + String(read_target_elevation) + " Match: " + String(target_elevation == read_target_elevation));
  Serial.println("Original Current Azimuth: " + String(current_azimuth) + " Read Current Azimuth: " + String(read_current_azimuth) + " Match: " + String(current_azimuth == read_current_azimuth));
  Serial.println("Original Current Elevation: " + String(current_elevation) + " Read Current Elevation: " + String(read_current_elevation) + " Match: " + String(current_elevation == read_current_elevation));
  Serial.println("");
}

void test_base_ranging_status()
{
  // Create Base ranging status
  uint16_t sequence_count = 123;
  uint32_t epoch_time = 1234567890;
  uint16_t subseconds = 1234;
  uint16_t time_since_last_ranging = 123;
  uint8_t packet_length;

  byte *packet = BaseStation1Telemetry::RangingStatus::create(packet_length, sequence_count, epoch_time, subseconds, time_since_last_ranging);

  Serial.println("Created base Ranging Status Packet: ");

  for (int i = 12; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base ranging status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint32_t read_epoch_time;
  uint16_t read_subseconds;
  uint16_t read_time_since_last_ranging;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telemetry(packet, read_apid, read_sequence_count, read_epoch_time, read_subseconds, read_data, read_data_length);

  BaseStation1Telemetry::RangingStatus::get_values(read_data, read_time_since_last_ranging);

  Serial.println("Read Base Ranging Status Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_SUBSYSTEM_2_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_SUBSYSTEM_2_STATUS == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Epoch Time: " + String(epoch_time) + " Read Epoch Time: " + String(read_epoch_time) + " Match: " + String(epoch_time == read_epoch_time));
  Serial.println("Original Subseconds: " + String(subseconds) + " Read Subseconds: " + String(read_subseconds) + " Match: " + String(subseconds == read_subseconds));
  Serial.println("Original Time Since Last Ranging: " + String(time_since_last_ranging) + " Read Time Since Last Ranging: " + String(read_time_since_last_ranging) + " Match: " + String(time_since_last_ranging == read_time_since_last_ranging));
  Serial.println("");
}


void test_balloon_set_configuration()
{
  // Create Balloon configuration
  uint16_t sequence_count = 123;
  float lora_frequency = 123.456;
  CCSDS_Enums::LoRa_TX_Power lora_tx_power = CCSDS_Enums::LoRa_TX_Power::TX_POWER_20;
  CCSDS_Enums::LoRa_Spreading_Factor lora_spreading_factor = CCSDS_Enums::LoRa_Spreading_Factor::SF_7;
  CCSDS_Enums::LoRa_Bandwidth lora_bandwidth = CCSDS_Enums::LoRa_Bandwidth::BW_125;
  CCSDS_Enums::LoRa_Coding_Rate lora_coding_rate = CCSDS_Enums::LoRa_Coding_Rate::CR_4_5;
  float barometer_reference_pressure = 1234.5;
  uint8_t packet_length;

  byte *packet = BalloonTelecommands::SetConfiguration::create(packet_length, sequence_count, lora_frequency, lora_tx_power, lora_spreading_factor, lora_bandwidth, lora_coding_rate, barometer_reference_pressure);

  Serial.println("Created Balloon Set Configuration Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  float read_lora_frequency;
  CCSDS_Enums::LoRa_TX_Power read_lora_tx_power;
  CCSDS_Enums::LoRa_Spreading_Factor read_lora_spreading_factor;
  CCSDS_Enums::LoRa_Bandwidth read_lora_bandwidth;
  CCSDS_Enums::LoRa_Coding_Rate read_lora_coding_rate;
  float read_barometer_reference_pressure;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  BalloonTelecommands::SetConfiguration::get_values(read_data, read_lora_frequency, read_lora_tx_power, read_lora_spreading_factor, read_lora_bandwidth, read_lora_coding_rate, read_barometer_reference_pressure);

  Serial.println("Read Balloon Set Configuration Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BALLOON_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BALLOON_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::SET_CONFIGURATION) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::SET_CONFIGURATION == read_packet_id));
  Serial.println("Original LoRa Frequency: " + String(lora_frequency) + " Read LoRa Frequency: " + String(read_lora_frequency) + " Match: " + String(lora_frequency == read_lora_frequency));
  Serial.println("Original LoRa TX Power: " + String(static_cast<uint8_t>(lora_tx_power)) + " Read LoRa TX Power: " + String(static_cast<uint8_t>(read_lora_tx_power)) + " Match: " + String(static_cast<uint8_t>(lora_tx_power) == static_cast<uint8_t>(read_lora_tx_power)));
  Serial.println("Original LoRa Spreading Factor: " + String(static_cast<uint8_t>(lora_spreading_factor)) + " Read LoRa Spreading Factor: " + String(static_cast<uint8_t>(read_lora_spreading_factor)) + " Match: " + String(static_cast<uint8_t>(lora_spreading_factor) == static_cast<uint8_t>(read_lora_spreading_factor)));
  Serial.println("Original LoRa Bandwidth: " + String(static_cast<uint8_t>(lora_bandwidth)) + " Read LoRa Bandwidth: " + String(static_cast<uint8_t>(read_lora_bandwidth)) + " Match: " + String(static_cast<uint8_t>(lora_bandwidth) == static_cast<uint8_t>(read_lora_bandwidth)));
  Serial.println("Original LoRa Coding Rate: " + String(static_cast<uint8_t>(lora_coding_rate)) + " Read LoRa Coding Rate: " + String(static_cast<uint8_t>(read_lora_coding_rate)) + " Match: " + String(static_cast<uint8_t>(lora_coding_rate) == static_cast<uint8_t>(read_lora_coding_rate)));
  Serial.println("Original Barometer Reference Pressure: " + String(barometer_reference_pressure) + " Read Barometer Reference Pressure: " + String(read_barometer_reference_pressure) + " Match: " + String(barometer_reference_pressure == read_barometer_reference_pressure));
  Serial.println("");
}

void test_balloon_request_configuration()
{
  // Create Balloon request configuration
  uint16_t sequence_count = 123;
  uint8_t packet_length;

  byte *packet = BalloonTelecommands::RequestConfiguration::create(packet_length, sequence_count);

  Serial.println("Created Balloon Request Configuration Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon request configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  Serial.println("Read Balloon Request Configuration Data length == 0? " + String(read_data_length == 0));

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BALLOON_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BALLOON_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::REQUEST_CONFIGURATION) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::REQUEST_CONFIGURATION == read_packet_id));
  Serial.println("");
}

void test_balloon_request_rwc_status()
{
  // Create Balloon request RWC status
  uint16_t sequence_count = 123;
  uint8_t packet_length;

  byte *packet = BalloonTelecommands::RequestSubsystem1Status::create(packet_length, sequence_count);

  Serial.println("Created Balloon Request RWC Status Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Balloon request RWC status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  Serial.println("Read Balloon Request RWC Status Data length == 0? " + String(read_data_length == 0));

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BALLOON_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BALLOON_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::REQUEST_SUBSYSTEM_1_STATUS) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::REQUEST_SUBSYSTEM_1_STATUS == read_packet_id));
  Serial.println("");
}

void test_payload_set_configuration()
{
  // Create Payload configuration
  uint16_t sequence_count = 123;
  float lora_frequency = 123.456;
  CCSDS_Enums::LoRa_TX_Power lora_tx_power = CCSDS_Enums::LoRa_TX_Power::TX_POWER_20;
  CCSDS_Enums::LoRa_Spreading_Factor lora_spreading_factor = CCSDS_Enums::LoRa_Spreading_Factor::SF_7;
  CCSDS_Enums::LoRa_Bandwidth lora_bandwidth = CCSDS_Enums::LoRa_Bandwidth::BW_125;
  CCSDS_Enums::LoRa_Coding_Rate lora_coding_rate = CCSDS_Enums::LoRa_Coding_Rate::CR_4_5;
  float barometer_reference_pressure = 1234.5;
  uint8_t packet_length;

  byte *packet = PayloadTelecommands::SetConfiguration::create(packet_length, sequence_count, lora_frequency, lora_tx_power, lora_spreading_factor, lora_bandwidth, lora_coding_rate, barometer_reference_pressure);

  Serial.println("Created Payload Set Configuration Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Payload configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  float read_lora_frequency;
  CCSDS_Enums::LoRa_TX_Power read_lora_tx_power;
  CCSDS_Enums::LoRa_Spreading_Factor read_lora_spreading_factor;
  CCSDS_Enums::LoRa_Bandwidth read_lora_bandwidth;
  CCSDS_Enums::LoRa_Coding_Rate read_lora_coding_rate;
  float read_barometer_reference_pressure;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  PayloadTelecommands::SetConfiguration::get_values(read_data, read_lora_frequency, read_lora_tx_power, read_lora_spreading_factor, read_lora_bandwidth, read_lora_coding_rate, read_barometer_reference_pressure);

  Serial.println("Read Payload Set Configuration Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::PAYLOAD_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::PAYLOAD_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::SET_CONFIGURATION) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::SET_CONFIGURATION == read_packet_id));
  Serial.println("Original LoRa Frequency: " + String(lora_frequency) + " Read LoRa Frequency: " + String(read_lora_frequency) + " Match: " + String(lora_frequency == read_lora_frequency));
  Serial.println("Original LoRa TX Power: " + String(static_cast<uint8_t>(lora_tx_power)) + " Read LoRa TX Power: " + String(static_cast<uint8_t>(read_lora_tx_power)) + " Match: " + String(static_cast<uint8_t>(lora_tx_power) == static_cast<uint8_t>(read_lora_tx_power)));
  Serial.println("Original LoRa Spreading Factor: " + String(static_cast<uint8_t>(lora_spreading_factor)) + " Read LoRa Spreading Factor: " + String(static_cast<uint8_t>(read_lora_spreading_factor)) + " Match: " + String(static_cast<uint8_t>(lora_spreading_factor) == static_cast<uint8_t>(read_lora_spreading_factor)));
  Serial.println("Original LoRa Bandwidth: " + String(static_cast<uint8_t>(lora_bandwidth)) + " Read LoRa Bandwidth: " + String(static_cast<uint8_t>(read_lora_bandwidth)) + " Match: " + String(static_cast<uint8_t>(lora_bandwidth) == static_cast<uint8_t>(read_lora_bandwidth)));
  Serial.println("Original LoRa Coding Rate: " + String(static_cast<uint8_t>(lora_coding_rate)) + " Read LoRa Coding Rate: " + String(static_cast<uint8_t>(read_lora_coding_rate)) + " Match: " + String(static_cast<uint8_t>(lora_coding_rate) == static_cast<uint8_t>(read_lora_coding_rate)));
  Serial.println("Original Barometer Reference Pressure: " + String(barometer_reference_pressure) + " Read Barometer Reference Pressure: " + String(read_barometer_reference_pressure) + " Match: " + String(barometer_reference_pressure == read_barometer_reference_pressure));
  Serial.println("");
}

void test_payload_request_configuration()
{
  // Create Payload request configuration
  uint16_t sequence_count = 123;
  uint8_t packet_length;

  byte *packet = PayloadTelecommands::RequestConfiguration::create(packet_length, sequence_count);

  Serial.println("Created Payload Request Configuration Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Payload request configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  Serial.println("Read Payload Request Configuration Data length == 0? " + String(read_data_length == 0));

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::PAYLOAD_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::PAYLOAD_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::REQUEST_CONFIGURATION) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::REQUEST_CONFIGURATION == read_packet_id));
  Serial.println("");
}

void test_payload_request_heated_container_status()
{
  // Create Payload request heated container status
  uint16_t sequence_count = 123;
  uint8_t packet_length;

  byte *packet = PayloadTelecommands::RequestSubsystem1Status::create(packet_length, sequence_count);

  Serial.println("Created Payload Request Heated Container Status Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Payload request heated container status packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  Serial.println("Read Payload Request Heated Container Status Data length == 0? " + String(read_data_length == 0));

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::PAYLOAD_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::PAYLOAD_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::REQUEST_SUBSYSTEM_1_STATUS) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::REQUEST_SUBSYSTEM_1_STATUS == read_packet_id));
  Serial.println("");
}

void test_base_set_configuration()
{
  // Create Base configuration
  uint16_t sequence_count = 123;
  float lora_frequency = 123.456;
  CCSDS_Enums::LoRa_TX_Power lora_tx_power = CCSDS_Enums::LoRa_TX_Power::TX_POWER_20;
  CCSDS_Enums::LoRa_Spreading_Factor lora_spreading_factor = CCSDS_Enums::LoRa_Spreading_Factor::SF_7;
  CCSDS_Enums::LoRa_Bandwidth lora_bandwidth = CCSDS_Enums::LoRa_Bandwidth::BW_125;
  CCSDS_Enums::LoRa_Coding_Rate lora_coding_rate = CCSDS_Enums::LoRa_Coding_Rate::CR_4_5;
  uint8_t packet_length;

  byte *packet = BaseStation1Telecommands::SetConfiguration::create(packet_length, sequence_count, lora_frequency, lora_tx_power, lora_spreading_factor, lora_bandwidth, lora_coding_rate);

  Serial.println("Created Base Set Configuration Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  float read_lora_frequency;
  CCSDS_Enums::LoRa_TX_Power read_lora_tx_power;
  CCSDS_Enums::LoRa_Spreading_Factor read_lora_spreading_factor;
  CCSDS_Enums::LoRa_Bandwidth read_lora_bandwidth;
  CCSDS_Enums::LoRa_Coding_Rate read_lora_coding_rate;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  BaseStation1Telecommands::SetConfiguration::get_values(read_data, read_lora_frequency, read_lora_tx_power, read_lora_spreading_factor, read_lora_bandwidth, read_lora_coding_rate);

  Serial.println("Read Base Set Configuration Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }

  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::SET_CONFIGURATION) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::SET_CONFIGURATION == read_packet_id));
  Serial.println("Original LoRa Frequency: " + String(lora_frequency) + " Read LoRa Frequency: " + String(read_lora_frequency) + " Match: " + String(lora_frequency == read_lora_frequency));
  Serial.println("Original LoRa TX Power: " + String(static_cast<uint8_t>(lora_tx_power)) + " Read LoRa TX Power: " + String(static_cast<uint8_t>(read_lora_tx_power)) + " Match: " + String(static_cast<uint8_t>(lora_tx_power) == static_cast<uint8_t>(read_lora_tx_power)));
  Serial.println("Original LoRa Spreading Factor: " + String(static_cast<uint8_t>(lora_spreading_factor)) + " Read LoRa Spreading Factor: " + String(static_cast<uint8_t>(read_lora_spreading_factor)) + " Match: " + String(static_cast<uint8_t>(lora_spreading_factor) == static_cast<uint8_t>(read_lora_spreading_factor)));
  Serial.println("Original LoRa Bandwidth: " + String(static_cast<uint8_t>(lora_bandwidth)) + " Read LoRa Bandwidth: " + String(static_cast<uint8_t>(read_lora_bandwidth)) + " Match: " + String(static_cast<uint8_t>(lora_bandwidth) == static_cast<uint8_t>(read_lora_bandwidth)));
  Serial.println("Original LoRa Coding Rate: " + String(static_cast<uint8_t>(lora_coding_rate)) + " Read LoRa Coding Rate: " + String(static_cast<uint8_t>(read_lora_coding_rate)) + " Match: " + String(static_cast<uint8_t>(lora_coding_rate) == static_cast<uint8_t>(read_lora_coding_rate)));
  Serial.println("");
}

void test_base_request_configuration()
{
  // Create Base request configuration
  uint16_t sequence_count = 123;
  uint8_t packet_length;

  byte *packet = BaseStation1Telecommands::RequestConfiguration::create(packet_length, sequence_count);

  Serial.println("Created Base Request Configuration Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base request configuration packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  Serial.println("Read Base Request Configuration Data length == 0? " + String(read_data_length == 0));

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::REQUEST_CONFIGURATION) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::REQUEST_CONFIGURATION == read_packet_id));
  Serial.println("");
}

void test_base_set_angles()
{
  // Create Base set angles
  uint16_t sequence_count = 123;
  float azimuth = 123.45;
  float elevation = 123.45;
  uint8_t packet_length;

  byte *packet = BaseStation1Telecommands::SetAngles::create(packet_length, sequence_count, azimuth, elevation);

  Serial.println("Created Base Set Angles Packet: ");

  for (int i = 8; i < packet_length; i++)
  {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Get values from Base set angles packet and compare to expected values
  uint16_t read_apid;
  uint16_t read_sequence_count;
  uint16_t read_packet_id;
  float read_azimuth;
  float read_elevation;
  byte *read_data;
  uint16_t read_data_length;

  parse_ccsds_telecommand(packet, read_apid, read_sequence_count, read_packet_id, read_data, read_data_length);

  BaseStation1Telecommands::SetAngles::get_values(read_data, read_azimuth, read_elevation);

  Serial.println("Read Base Set Angles Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::APID::BASE_STATION_1_TELECOMMAND) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::APID::BASE_STATION_1_TELECOMMAND == read_apid));
  Serial.println("Original Sequence Count: " + String(sequence_count) + " Read Sequence Count: " + String(read_sequence_count) + " Match: " + String(sequence_count == read_sequence_count));
  Serial.println("Original Packet ID: " + String(CCSDS_Enums::PacketID::BASE_STATION_SET_ANGLES) + " Read Packet ID: " + String(read_packet_id) + " Match: " + String(CCSDS_Enums::PacketID::BASE_STATION_SET_ANGLES == read_packet_id));
  Serial.println("Original Azimuth: " + String(azimuth) + " Read Azimuth: " + String(read_azimuth) + " Match: " + String(azimuth == read_azimuth));
  Serial.println("Original Elevation: " + String(elevation) + " Read Elevation: " + String(read_elevation) + " Match: " + String(elevation == read_elevation));
  Serial.println("");
}



void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1000);
  }

  Serial.println("Starting tests...");
  Serial.println("");

  Serial.println("Ballon Telemetry Tests:");
  Serial.println("");
  Serial.println("--------------------------------------------------");
  test_balloon_telecommand_acknowledgement();
  Serial.println("--------------------------------------------------");
  test_balloon_system_status();
  Serial.println("--------------------------------------------------");
  test_balloon_configuration();
  Serial.println("--------------------------------------------------");
  test_balloon_location();
  Serial.println("--------------------------------------------------");
  test_balloon_rwc_status();
  Serial.println("--------------------------------------------------");
  Serial.println("");

  Serial.println("Payload Telemetry Tests:");
  Serial.println("");
  Serial.println("--------------------------------------------------");
  test_payload_telecommand_acknowledgement();
  Serial.println("--------------------------------------------------");
  test_payload_system_status();
  Serial.println("--------------------------------------------------");
  test_payload_configuration();
  Serial.println("--------------------------------------------------");
  test_payload_location();
  Serial.println("--------------------------------------------------");
  test_payload_heater_container_status();
  Serial.println("--------------------------------------------------");
  Serial.println("");

  Serial.println("Base Station Telemetry Tests:");
  Serial.println("");
  Serial.println("--------------------------------------------------");
  test_base_telecommand_acknowledgement();
  Serial.println("--------------------------------------------------");
  test_base_system_status();
  Serial.println("--------------------------------------------------");
  test_base_configuration();
  Serial.println("--------------------------------------------------");
  test_base_location();
  Serial.println("--------------------------------------------------");
  test_base_rotator_status();
  Serial.println("--------------------------------------------------");
  test_base_ranging_status();
  Serial.println("--------------------------------------------------");
  Serial.println("");

  Serial.println("Balloon Telecommand Tests:");
  Serial.println("");
  Serial.println("--------------------------------------------------");
  test_balloon_set_configuration();
  Serial.println("--------------------------------------------------");
  test_balloon_request_configuration();
  Serial.println("--------------------------------------------------");
  test_balloon_request_rwc_status();
  Serial.println("--------------------------------------------------");
  Serial.println("");

  Serial.println("Payload Telecommand Tests:");
  Serial.println("");
  Serial.println("--------------------------------------------------");
  test_payload_set_configuration();
  Serial.println("--------------------------------------------------");
  test_payload_request_configuration();
  Serial.println("--------------------------------------------------");
  test_payload_request_heated_container_status();
  Serial.println("--------------------------------------------------");
  Serial.println("");

  Serial.println("Base Station Telecommand Tests:");
  Serial.println("");
  Serial.println("--------------------------------------------------");
  test_base_set_configuration();
  Serial.println("--------------------------------------------------");
  test_base_request_configuration();
  Serial.println("--------------------------------------------------");
  test_base_set_angles();
  Serial.println("--------------------------------------------------");

  Serial.println("All tests done!");
}

void loop()
{
}