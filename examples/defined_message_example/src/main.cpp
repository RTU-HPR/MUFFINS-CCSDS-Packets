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

  for (int i = 0; i < packet_length; i++)
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

  parse_ccsds_telemetry(packet, read_apid, sequence_count, epoch_time, subseconds, read_data, read_data_length);

  Serial.println("Read Balloon Telecommand Acknowledgement Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::BalloonAPID::TELECOMMAND_ACKNOWLEDGEMENT) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::BalloonAPID::TELECOMMAND_ACKNOWLEDGEMENT == read_apid));
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

  for (int i = 0; i < packet_length; i++)
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

  parse_ccsds_telemetry(packet, read_apid, sequence_count, epoch_time, subseconds, read_data, read_data_length);

  BalloonTelemetry::SystemStatus::get_values(read_data, read_uptime, read_battery_voltage, read_uplink_rssi, read_error_binary_string, read_flight_mode, read_gps_lock, read_uplink_status);

  Serial.println("Read Balloon System Status Packet: ");
  
  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::BalloonAPID::SYSTEM_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::BalloonAPID::SYSTEM_STATUS == read_apid));
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

  for (int i = 0; i < packet_length; i++)
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

  parse_ccsds_telemetry(packet, read_apid, sequence_count, epoch_time, subseconds, read_data, read_data_length);

  BalloonTelemetry::Configuration::get_values(read_data, read_lora_frequency, read_lora_tx_power, read_lora_spreading_factor, read_lora_bandwidth, read_lora_coding_rate, read_barometer_reference_pressure);

  Serial.println("Read Balloon Configuration Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::BalloonAPID::CONFIGURATION) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::BalloonAPID::CONFIGURATION == read_apid));
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

  for (int i = 0; i < packet_length; i++)
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

  parse_ccsds_telemetry(packet, read_apid, sequence_count, epoch_time, subseconds, read_data, read_data_length);

  BalloonTelemetry::Location::get_values(read_data, read_latitude, read_longitude, read_gps_altitude, read_barometer_altitude, read_satellites);

  Serial.println("Read Balloon Location Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::BalloonAPID::LOCATION) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::BalloonAPID::LOCATION == read_apid));
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

  for (int i = 0; i < packet_length; i++)
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

  parse_ccsds_telemetry(packet, read_apid, sequence_count, epoch_time, subseconds, read_data, read_data_length);

  BalloonTelemetry::RwcStatus::get_values(read_data, read_heading, read_angular_velocity_x, read_angular_velocity_y, read_angular_velocity_z, read_motor_angular_velocity, read_rwc_battery_voltage, read_rwc_temperature);

  Serial.println("Read Balloon RWC Status Packet: ");

  for (int i = 0; i < read_data_length; i++)
  {
    Serial.print(read_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Compare values
  Serial.println("Original APID: " + String(CCSDS_Enums::BalloonAPID::SUBSYSTEM_1_STATUS) + " Read APID: " + String(read_apid) + " Match: " + String(CCSDS_Enums::BalloonAPID::SUBSYSTEM_1_STATUS == read_apid));
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

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1000);
  }

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
  Serial.println("Done!");
}

void loop()
{
}