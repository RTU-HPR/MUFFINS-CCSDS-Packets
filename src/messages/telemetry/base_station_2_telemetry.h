#include "../enums/enums.h"
#include "../../MUFFINS_CCSDS_Packets.h"

using namespace CCSDS_Enums;

namespace BaseStation2Telemetry
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
      uint16_t apid = APID::BASE_STATION_1_TELECOMMAND_ACKNOWLEDGEMENT;
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
     * @param wifi_rssi Wi-Fi RSSI in dBm
     * @param error_binary_string Error binary string
     * @param transmitting_allowed Transmitting allowed
     */
    void get_values(
        const byte *data,
        uint16_t &uptime,
        float &battery_voltage,
        uint8_t &wifi_rssi,
        uint16_t &error_binary_string,
        uint8_t &transmitting_allowed)
    {
      uptime = uint16_from_bytes(data[0], data[1]);
      battery_voltage = float_from_bytes(data[2], data[3], data[4], data[5]);
      wifi_rssi = uint8_from_bytes(data[6]);
      error_binary_string = uint16_from_bytes(data[7], data[8]);

      // Highest bit is transmitting allowed
      transmitting_allowed = (data[9] & 0b10000000) >> 7;

      // Remaining 7 bits are reserved
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
     * @param wifi_rssi Wi-Fi RSSI in dBm
     * @param error_binary_string Error binary string
     * @param transmitting_allowed Transmitting allowed
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
        const uint8_t &wifi_rssi,
        const uint16_t &error_binary_string,
        const uint8_t &transmitting_allowed)
    {
      const uint8_t combined_bitmask = (transmitting_allowed << 7);

      uint16_t apid = APID::BASE_STATION_1_SYSTEM_STATUS;
      uint16_t data_values_count = 5;
      Converter data_values[data_values_count] = {
          {.ui16 = uptime},
          {.f = battery_voltage},
          {.ui8 = wifi_rssi},
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
     *
     * @param ranging_lora_frequency Ranging LoRa frequency
     * @param ranging_lora_tx_power Ranging LoRa TX power
     * @param ranging_lora_spreading_factor Ranging LoRa spreading factor
     * @param ranging_lora_bandwidth Ranging LoRa bandwidth
     * @param ranging_lora_coding_rate Ranging LoRa coding rate
     *
     * @param
     */
    void get_values(
        const byte *data,
        float &lora_frequency,
        CCSDS_Enums::LoRa_TX_Power &lora_tx_power,
        CCSDS_Enums::LoRa_Spreading_Factor &lora_spreading_factor,
        CCSDS_Enums::LoRa_Bandwidth &lora_bandwidth,
        CCSDS_Enums::LoRa_Coding_Rate &lora_coding_rate,
        float &ranging_lora_frequency,
        CCSDS_Enums::LoRa_TX_Power &ranging_lora_tx_power,
        CCSDS_Enums::LoRa_Spreading_Factor &ranging_lora_spreading_factor,
        CCSDS_Enums::LoRa_Bandwidth &ranging_lora_bandwidth,
        CCSDS_Enums::LoRa_Coding_Rate &ranging_lora_coding_rate)
    {
      lora_frequency = float_from_bytes(data[0], data[1], data[2], data[3]);
      lora_tx_power = static_cast<CCSDS_Enums::LoRa_TX_Power>(uint8_from_bytes(data[4]));
      lora_spreading_factor = static_cast<CCSDS_Enums::LoRa_Spreading_Factor>(uint8_from_bytes(data[5]));
      lora_bandwidth = static_cast<CCSDS_Enums::LoRa_Bandwidth>(uint8_from_bytes(data[6]));
      lora_coding_rate = static_cast<CCSDS_Enums::LoRa_Coding_Rate>(uint8_from_bytes(data[7]));

      ranging_lora_frequency = float_from_bytes(data[8], data[9], data[10], data[11]);
      ranging_lora_tx_power = static_cast<CCSDS_Enums::LoRa_TX_Power>(uint8_from_bytes(data[12]));
      ranging_lora_spreading_factor = static_cast<CCSDS_Enums::LoRa_Spreading_Factor>(uint8_from_bytes(data[13]));
      ranging_lora_bandwidth = static_cast<CCSDS_Enums::LoRa_Bandwidth>(uint8_from_bytes(data[14]));
      ranging_lora_coding_rate = static_cast<CCSDS_Enums::LoRa_Coding_Rate>(uint8_from_bytes(data[15]));
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
     *
     * @param ranging_lora_frequency Ranging LoRa frequency
     * @param ranging_lora_tx_power Ranging LoRa TX power
     * @param ranging_lora_spreading_factor Ranging LoRa spreading factor
     * @param ranging_lora_bandwidth Ranging LoRa bandwidth
     * @param ranging_lora_coding_rate Ranging LoRa coding rate
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
        const float &ranging_lora_frequency,
        const CCSDS_Enums::LoRa_TX_Power &ranging_lora_tx_power,
        const CCSDS_Enums::LoRa_Spreading_Factor &ranging_lora_spreading_factor,
        const CCSDS_Enums::LoRa_Bandwidth &ranging_lora_bandwidth,
        const CCSDS_Enums::LoRa_Coding_Rate &ranging_lora_coding_rate)
    {
      uint16_t apid = APID::BASE_STATION_2_CONFIGURATION;
      uint16_t data_values_count = 10;
      Converter data_values[data_values_count] = {
          {.f = lora_frequency},
          {.ui8 = lora_tx_power},
          {.ui8 = lora_spreading_factor},
          {.ui8 = lora_bandwidth},
          {.ui8 = lora_coding_rate},
          {.f = ranging_lora_frequency},
          {.ui8 = ranging_lora_tx_power},
          {.ui8 = ranging_lora_spreading_factor},
          {.ui8 = ranging_lora_bandwidth},
          {.ui8 = ranging_lora_coding_rate}};
      String data_format[data_values_count] = {"float", "uint8", "uint8", "uint8", "uint8", "float", "uint8", "uint8", "uint8", "uint8"};
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
     * @param satellites Satellites
     */
    void get_values(
        const byte *data,
        float &latitude,
        float &longitude,
        float &gps_altitude,
        uint8_t &satellites)
    {
      latitude = float_from_bytes(data[0], data[1], data[2], data[3]);
      longitude = float_from_bytes(data[4], data[5], data[6], data[7]);
      gps_altitude = float_from_bytes(data[8], data[9], data[10], data[11]);
      satellites = uint8_from_bytes(data[12]);
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
        const uint8_t &satellites)
    {
      uint16_t apid = APID::BASE_STATION_1_LOCATION;
      uint16_t data_values_count = 4;
      Converter data_values[data_values_count] = {
          {.f = latitude},
          {.f = longitude},
          {.f = gps_altitude},
          {.ui8 = satellites}};
      String data_format[data_values_count] = {"float", "float", "float", "uint8"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }

  namespace RotatorStatus
  {
    /**
     * @brief Get values from rotator status data field
     *
     * @param data Data field of rotator status packet
     * @param target_azimuth Target azimuth
     * @param target_elevation Target elevation
     * @param current_azimuth Current azimuth
     * @param current_elevation Current elevation
     */
    void get_values(
        const byte *data,
        float &target_azimuth,
        float &target_elevation,
        float &current_azimuth,
        float &current_elevation)
    {
      target_azimuth = float_from_bytes(data[0], data[1], data[2], data[3]);
      target_elevation = float_from_bytes(data[4], data[5], data[6], data[7]);
      current_azimuth = float_from_bytes(data[8], data[9], data[10], data[11]);
      current_elevation = float_from_bytes(data[12], data[13], data[14], data[15]);
    }

    /**
     * @brief Create a rotator status packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param target_azimuth Target azimuth
     * @param target_elevation Target elevation
     * @param current_azimuth Current azimuth
     * @param current_elevation Current elevation
     *
     * @return Pointer to rotator status packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const float &target_azimuth,
        const float &target_elevation,
        const float &current_azimuth,
        const float &current_elevation)
    {
      uint16_t apid = APID::BASE_STATION_1_SUBSYSTEM_1_STATUS;
      uint16_t data_values_count = 4;
      Converter data_values[data_values_count] = {
          {.f = target_azimuth},
          {.f = target_elevation},
          {.f = current_azimuth},
          {.f = current_elevation}};
      String data_format[data_values_count] = {"float", "float", "float", "float"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }

  namespace RangingStatus
  {
    /**
     * @brief Get values from ranging status data field
     *
     * @param data Data field of ranging status packet
     * @param time_since_last_ranging Time since last ranging in seconds
     */
    void get_values(
        const byte *data,
        uint16_t &time_since_last_ranging)
    {
      time_since_last_ranging = uint16_from_bytes(data[0], data[1]);
    }

    /**
     * @brief Create a ranging status packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param time_since_last_ranging Time since last ranging in seconds
     *
     * @return Pointer to ranging status packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const uint16_t &time_since_last_ranging)
    {
      uint16_t apid = APID::BASE_STATION_1_SUBSYSTEM_2_STATUS;
      uint16_t data_values_count = 1;
      Converter data_values[data_values_count] = {{.ui16 = time_since_last_ranging}};
      String data_format[data_values_count] = {"uint16"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }
}
