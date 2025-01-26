#include "../enums/enums.h"
#include "../../MUFFINS_CCSDS_Packets.h"

using namespace CCSDS_Enums;

namespace BalloonTelemetry
{
  namespace TelecommandAckowledgement
  {
    /**
     * @brief Get values from telecommand acknowledgement data field
     *
     * @param data Data field of telecommand acknowledgement packet
     * @param received_packet_id Received packet ID
     */
    void get_values(const byte *data, uint16_t &received_packet_id)
    {
      received_packet_id = uint16_from_bytes(data[0], data[1]);
    }

    /**
     * @brief Create a telecommand acknowledgement packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param received_packet_id Received packet ID
     *
     * @return Pointer to telecommand acknowledgement packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const uint16_t &received_packet_id)
    {
      uint16_t apid = APID::BALLOON_TELECOMMAND_ACKNOWLEDGEMENT;
      uint16_t data_values_count = 1;
      Converter data_values[data_values_count] = {{.ui16 = received_packet_id}};
      String data_format[data_values_count] = {"uint16"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }

  }

  namespace SystemStatus
  {
    /**
     * @brief Get values from system status data field
     *
     * @param data Data field of system status packet
     * @param uptime Uptime in seconds
     * @param battery_voltage Battery voltage in volts
     * @param uplink_rssi Uplink RSSI in dBm
     * @param error_binary_string Error binary string
     * @param flight_mode Flight mode
     * @param gps_lock GPS lock
     * @param uplink_status Uplink status
     */
    void get_values(
        const byte *data,
        uint16_t &uptime,
        float &battery_voltage,
        uint8_t &uplink_rssi,
        uint16_t &error_binary_string,
        uint8_t &flight_mode,
        uint8_t &gps_lock,
        uint8_t &uplink_status)
    {
      uptime = uint16_from_bytes(data[0], data[1]);
      battery_voltage = float_from_bytes(data[2], data[3], data[4], data[5]);
      uplink_rssi = uint8_from_bytes(data[6]);
      error_binary_string = uint16_from_bytes(data[7], data[8]);

      // Flight mode is first 2 bits
      flight_mode = (data[9] >> 6) & 0x03;

      // GPS lock is next 1 bit
      gps_lock = (data[9] >> 5) & 0x01;

      // Uplink status is next 1 bit
      uplink_status = (data[9] >> 4) & 0x01;

      // Remaining 4 bits are reserved
    }

    /**
     * @brief Create a system status packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param uptime Uptime in seconds
     * @param battery_voltage Battery voltage in volts
     * @param uplink_rssi Uplink RSSI in dBm
     * @param error_binary_string Error binary string
     * @param flight_mode Flight mode
     * @param gps_lock GPS lock
     * @param uplink_status Uplink status
     *
     * @return Pointer to system status packet
     */

    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const uint16_t &uptime,
        const float &battery_voltage,
        const uint8_t &uplink_rssi,
        const uint16_t &error_binary_string,
        const uint8_t &flight_mode,
        const uint8_t &gps_lock,
        const uint8_t &uplink_status)
    {
      const uint8_t combined_bitmask = (flight_mode << 6) | (gps_lock << 5) | (uplink_status << 4) | 0x0F;

      uint16_t apid = APID::BALLOON_SYSTEM_STATUS;
      uint16_t data_values_count = 5;
      Converter data_values[data_values_count] = {
          {.ui16 = uptime},
          {.f = battery_voltage},
          {.ui8 = uplink_rssi},
          {.ui16 = error_binary_string},
          {.ui8 = combined_bitmask}};
      String data_format[data_values_count] = {"uint16", "float", "uint8", "uint16", "uint8"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }

  namespace Configuration
  {
    /**
     * @brief Get values from configuration data field
     *
     * @param data Data field of configuration packet
     * @param lora_frequency LoRa frequency
     * @param lora_tx_power LoRa TX power
     * @param lora_spreading_factor LoRa spreading factor
     * @param lora_bandwidth LoRa bandwidth
     * @param lora_coding_rate LoRa coding rate
     * @param barometer_reference_pressure Barometer reference pressure
     */
    void get_values(
        const byte *data,
        float &lora_frequency,
        CCSDS_Enums::LoRa_TX_Power &lora_tx_power,
        CCSDS_Enums::LoRa_Spreading_Factor &lora_spreading_factor,
        CCSDS_Enums::LoRa_Bandwidth &lora_bandwidth,
        CCSDS_Enums::LoRa_Coding_Rate &lora_coding_rate,
        float &barometer_reference_pressure)
    {
      lora_frequency = float_from_bytes(data[0], data[1], data[2], data[3]);
      lora_tx_power = static_cast<CCSDS_Enums::LoRa_TX_Power>(uint8_from_bytes(data[4]));
      lora_spreading_factor = static_cast<CCSDS_Enums::LoRa_Spreading_Factor>(uint8_from_bytes(data[5]));
      lora_bandwidth = static_cast<CCSDS_Enums::LoRa_Bandwidth>(uint8_from_bytes(data[6]));
      lora_coding_rate = static_cast<CCSDS_Enums::LoRa_Coding_Rate>(uint8_from_bytes(data[7]));
      barometer_reference_pressure = float_from_bytes(data[8], data[9], data[10], data[11]);
    }

    /**
     * @brief Create a configuration packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param lora_frequency LoRa frequency
     * @param lora_tx_power LoRa TX power
     * @param lora_spreading_factor LoRa spreading factor
     * @param lora_bandwidth LoRa bandwidth
     * @param lora_coding_rate LoRa coding rate
     * @param barometer_reference_pressure Barometer reference pressure
     *
     * @return Pointer to configuration packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const float &lora_frequency,
        const CCSDS_Enums::LoRa_TX_Power &lora_tx_power,
        const CCSDS_Enums::LoRa_Spreading_Factor &lora_spreading_factor,
        const CCSDS_Enums::LoRa_Bandwidth &lora_bandwidth,
        const CCSDS_Enums::LoRa_Coding_Rate &lora_coding_rate,
        const float &barometer_reference_pressure)
    {
      uint16_t apid = APID::BALLOON_CONFIGURATION;
      uint16_t data_values_count = 6;
      Converter data_values[data_values_count] = {
          {.f = lora_frequency},
          {.ui8 = lora_tx_power},
          {.ui8 = lora_spreading_factor},
          {.ui8 = lora_bandwidth},
          {.ui8 = lora_coding_rate},
          {.f = barometer_reference_pressure}};
      String data_format[data_values_count] = {"float", "uint8", "uint8", "uint8", "uint8", "float"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }

  namespace Location
  {
    /**
     * @brief Get values from location data field
     *
     * @param data Data field of location packet
     * @param latitude Latitude
     * @param longitude Longitude
     * @param gps_altitude GPS altitude
     * @param barometer_altitude Barometer altitude
     * @param satellites Satellites
     */
    void get_values(
        const byte *data,
        float &latitude,
        float &longitude,
        float &gps_altitude,
        float &barometer_altitude,
        uint8_t &satellites)
    {
      latitude = float_from_bytes(data[0], data[1], data[2], data[3]);
      longitude = float_from_bytes(data[4], data[5], data[6], data[7]);
      gps_altitude = float_from_bytes(data[8], data[9], data[10], data[11]);
      barometer_altitude = float_from_bytes(data[12], data[13], data[14], data[15]);
      satellites = uint8_from_bytes(data[16]);
    }

    /**
     * @brief Create a location packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param latitude Latitude
     * @param longitude Longitude
     * @param gps_altitude GPS altitude
     * @param barometer_altitude Barometer altitude
     * @param satellites Satellites
     *
     * @return Pointer to location packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const float &latitude,
        const float &longitude,
        const float &gps_altitude,
        const float &barometer_altitude,
        const uint8_t &satellites)
    {
      uint16_t apid = APID::BALLOON_LOCATION;
      uint16_t data_values_count = 5;
      Converter data_values[data_values_count] = {
          {.f = latitude},
          {.f = longitude},
          {.f = gps_altitude},
          {.f = barometer_altitude},
          {.ui8 = satellites}};
      String data_format[data_values_count] = {"float", "float", "float", "float", "uint8"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }

  namespace RwcStatus
  {
    /**
     * @brief Get values from RWC status data field
     *
     * @param data Data field of RWC status packet
     * @param heading Heading
     * @param angular_velocity_x Angular velocity x (miliradians/s)
     * @param angular_velocity_y Angular velocity y (miliradians/s)
     * @param angular_velocity_z Angular velocity z (miliradians/s)
     * @param motor_angular_velocity Motor angular velocity
     * @param rwc_battery_voltage RWC battery voltage
     * @param rwc_temperature RWC temperature
     */
    void get_values(
        const byte *data,
        float &heading,
        int16_t &angular_velocity_x,
        int16_t &angular_velocity_y,
        int16_t &angular_velocity_z,
        float &motor_angular_velocity,
        float &rwc_battery_voltage,
        int8_t &rwc_temperature)
    {
      heading = float_from_bytes(data[0], data[1], data[2], data[3]);
      angular_velocity_x = int16_from_bytes(data[4], data[5]);
      angular_velocity_y = int16_from_bytes(data[6], data[7]);
      angular_velocity_z = int16_from_bytes(data[8], data[9]);
      motor_angular_velocity = float_from_bytes(data[10], data[11], data[12], data[13]);
      rwc_battery_voltage = float_from_bytes(data[14], data[15], data[16], data[17]);
      rwc_temperature = int8_from_bytes(data[18]);
    }

    /**
     * @brief Create a RWC status packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param heading Heading
     * @param angular_velocity_x Angular velocity x (miliradians/s)
     * @param angular_velocity_y Angular velocity y (miliradians/s)
     * @param angular_velocity_z Angular velocity z (miliradians/s)
     * @param motor_angular_velocity Motor angular velocity
     * @param rwc_battery_voltage RWC battery voltage
     * @param rwc_temperature RWC temperature
     *
     * @return Pointer to RWC status packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const float &heading,
        const int16_t &angular_velocity_x,
        const int16_t &angular_velocity_y,
        const int16_t &angular_velocity_z,
        const float &motor_angular_velocity,
        const float &rwc_battery_voltage,
        const int8_t &rwc_temperature)
    {
      uint16_t apid = APID::BALLOON_SUBSYSTEM_1_STATUS;
      uint16_t data_values_count = 7;
      Converter data_values[data_values_count] = {
          {.f = heading},
          {.i16 = angular_velocity_x},
          {.i16 = angular_velocity_y},
          {.i16 = angular_velocity_z},
          {.f = motor_angular_velocity},
          {.f = rwc_battery_voltage},
          {.i8 = rwc_temperature}};
      String data_format[data_values_count] = {"float", "int16", "int16", "int16", "float", "float", "int8"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }
}
