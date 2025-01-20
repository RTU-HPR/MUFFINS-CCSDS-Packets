#include "../enums/enums.h"
#include "../../MUFFINS_CCSDS_Packets.h"

using namespace CCSDS_Enums;

namespace PayloadTelemetry
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
      received_packet_id = data[0] << 8 | data[1];
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
      uint16_t apid = PayloadAPID::TELECOMMAND_ACKNOWLEDGEMENT;
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
      uptime = data[0] << 8 | data[1];
      battery_voltage = data[2] << 8 | data[3];
      uplink_rssi = data[4];
      error_binary_string = data[5] << 8 | data[6];

      // Flight mode is first 2 bits
      flight_mode = (data[7] >> 6) & 0x03;

      // GPS lock is next 1 bit
      gps_lock = (data[7] >> 5) & 0x01;

      // Uplink status is next 1 bit
      uplink_status = (data[7] >> 4) & 0x01;

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

      uint16_t apid = PayloadAPID::SYSTEM_STATUS;
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
      lora_frequency = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
      lora_tx_power = static_cast<CCSDS_Enums::LoRa_TX_Power>(data[4]);
      lora_spreading_factor = static_cast<CCSDS_Enums::LoRa_Spreading_Factor>(data[5]);
      lora_bandwidth = static_cast<CCSDS_Enums::LoRa_Bandwidth>(data[6]);
      lora_coding_rate = static_cast<CCSDS_Enums::LoRa_Coding_Rate>(data[7]);
      barometer_reference_pressure = data[8] << 24 | data[9] << 16 | data[10] << 8 | data[11];
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
      uint16_t apid = PayloadAPID::CONFIGURATION;
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
      latitude = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
      longitude = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
      gps_altitude = data[8] << 24 | data[9] << 16 | data[10] << 8 | data[11];
      barometer_altitude = data[12] << 24 | data[13] << 16 | data[14] << 8 | data[15];
      satellites = data[16];
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
      uint16_t apid = PayloadAPID::LOCATION;
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

  namespace HeatedContainerStatus
  {
    /**
    * @brief Get values from heated container status data field
    * 
    * @param data Data field of heated container status packet
    * @param air_temperature Air temperature in degrees Celsius
    * @param baro_temperature Barometer temperature in degrees Celsius
    * @param heatsink_temperature Heatsink temperature in degrees Celsius
    * @param pressure Pressure in pa
    * @param propotional_value Proportional value
    * @param integral_value Integral value
    * @param duty_cycle Duty cycle
    */
    void get_values(
        const byte *data,
        float &air_temperature,
        int8_t &baro_temperature,
        int8_t &heatsink_temperature,
        uint32_t &pressure,
        uint16_t &propotional_value,
        uint16_t &integral_value,
        uint8_t &duty_cycle)
    {
      air_temperature = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
      baro_temperature = data[4];
      heatsink_temperature = data[5];
      pressure = data[6] << 24 | data[7] << 16 | data[8] << 8 | data[9];
      propotional_value = data[10] << 8 | data[11];
      integral_value = data[12] << 8 | data[13];
      duty_cycle = data[14];
    }

    /**
     * @brief Create a heated container status packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param air_temperature Air temperature in degrees Celsius
     * @param baro_temperature Barometer temperature in degrees Celsius
     * @param heatsink_temperature Heatsink temperature in degrees Celsius
     * @param pressure Pressure in pa
     * @param propotional_value Proportional value
     * @param integral_value Integral value
     * @param duty_cycle Duty cycle
     * 
     * @return Pointer to heated container status packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const float &air_temperature,
        const int8_t &baro_temperature,
        const int8_t &heatsink_temperature,
        const uint32_t &pressure,
        const uint16_t &propotional_value,
        const uint16_t &integral_value,
        const uint8_t &duty_cycle)
    {
      uint16_t apid = PayloadAPID::SUBSYSTEM_1_STATUS;
      uint16_t data_values_count = 7;
      Converter data_values[data_values_count] = {
          {.f = air_temperature},
          {.i8 = baro_temperature},
          {.i8 = heatsink_temperature},
          {.ui32 = pressure},
          {.ui16 = propotional_value},
          {.ui16 = integral_value},
          {.ui8 = duty_cycle}};
      String data_format[data_values_count] = {"float", "int8", "int8", "uint32", "uint16", "uint16", "uint8"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }
}
