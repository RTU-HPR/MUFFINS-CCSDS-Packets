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
      uint16_t apid = APID::PAYLOAD_TELECOMMAND_ACKNOWLEDGEMENT;
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

      // Flight mode are the highest 2 bits
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

      uint16_t apid = APID::PAYLOAD_SYSTEM_STATUS;
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
     *
     * @param ranging_lora_frequency Ranging LoRa frequency
     * @param ranging_lora_tx_power Ranging LoRa TX power
     * @param ranging_lora_spreading_factor Ranging LoRa spreading factor
     * @param ranging_lora_bandwidth Ranging LoRa bandwidth
     * @param ranging_lora_coding_rate Ranging LoRa coding rate
     *
     * @param base_1_address Base station 1 address
     * @param base_1_latitude Base station 1 latitude
     * @param base_1_longitude Base station 1 longitude
     * @param base_1_altitude Base station 1 altitude
     *
     * @param base_2_address Base station 2 address
     * @param base_2_latitude Base station 2 latitude
     * @param base_2_longitude Base station 2 longitude
     * @param base_2_altitude Base station 2 altitude
     *
     * @param base_3_address Base station 3 address
     * @param base_3_latitude Base station 3 latitude
     * @param base_3_longitude Base station 3 longitude
     * @param base_3_altitude Base station 3 altitude
     *
     * @param barometer_reference_pressure Barometer reference pressure
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
        CCSDS_Enums::LoRa_Coding_Rate &ranging_lora_coding_rate,
        int8_t &base_1_address,
        float &base_1_latitude,
        float &base_1_longitude,
        float &base_1_altitude,
        int8_t &base_2_address,
        float &base_2_latitude,
        float &base_2_longitude,
        float &base_2_altitude,
        int8_t &base_3_address,
        float &base_3_latitude,
        float &base_3_longitude,
        float &base_3_altitude,
        float &barometer_reference_pressure)
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

      base_1_address = int8_from_bytes(data[16]);
      base_1_latitude = float_from_bytes(data[17], data[18], data[19], data[20]);
      base_1_longitude = float_from_bytes(data[21], data[22], data[23], data[24]);
      base_1_altitude = float_from_bytes(data[25], data[26], data[27], data[28]);

      base_2_address = int8_from_bytes(data[29]);
      base_2_latitude = float_from_bytes(data[30], data[31], data[32], data[33]);
      base_2_longitude = float_from_bytes(data[34], data[35], data[36], data[37]);
      base_2_altitude = float_from_bytes(data[38], data[39], data[40], data[41]);

      base_3_address = int8_from_bytes(data[42]);
      base_3_latitude = float_from_bytes(data[43], data[44], data[45], data[46]);
      base_3_longitude = float_from_bytes(data[47], data[48], data[49], data[50]);
      base_3_altitude = float_from_bytes(data[51], data[52], data[53], data[54]);

      barometer_reference_pressure = float_from_bytes(data[55], data[56], data[57], data[58]);
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
     * @param base_1_address Base station 1 address
     * @param base_1_latitude Base station 1 latitude
     * @param base_1_longitude Base station 1 longitude
     * @param base_1_altitude Base station 1 altitude
     *
     * @param base_2_address Base station 2 address
     * @param base_2_latitude Base station 2 latitude
     * @param base_2_longitude Base station 2 longitude
     * @param base_2_altitude Base station 2 altitude
     *
     * @param base_3_address Base station 3 address
     * @param base_3_latitude Base station 3 latitude
     * @param base_3_longitude Base station 3 longitude
     * @param base_3_altitude Base station 3 altitude
     *
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
        const float &ranging_lora_frequency,
        const CCSDS_Enums::LoRa_TX_Power &ranging_lora_tx_power,
        const CCSDS_Enums::LoRa_Spreading_Factor &ranging_lora_spreading_factor,
        const CCSDS_Enums::LoRa_Bandwidth &ranging_lora_bandwidth,
        const CCSDS_Enums::LoRa_Coding_Rate &ranging_lora_coding_rate,
        const int8_t &base_1_address,
        const float &base_1_latitude,
        const float &base_1_longitude,
        const float &base_1_altitude,
        const int8_t &base_2_address,
        const float &base_2_latitude,
        const float &base_2_longitude,
        const float &base_2_altitude,
        const int8_t &base_3_address,
        const float &base_3_latitude,
        const float &base_3_longitude,
        const float &base_3_altitude,
        const float &barometer_reference_pressure)
    {
      uint16_t apid = APID::PAYLOAD_CONFIGURATION;
      uint16_t data_values_count = 23;
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
          {.ui8 = ranging_lora_coding_rate},
          {.i8 = base_1_address},
          {.f = base_1_latitude},
          {.f = base_1_longitude},
          {.f = base_1_altitude},
          {.i8 = base_2_address},
          {.f = base_2_latitude},
          {.f = base_2_longitude},
          {.f = base_2_altitude},
          {.i8 = base_3_address},
          {.f = base_3_latitude},
          {.f = base_3_longitude},
          {.f = base_3_altitude},
          {.f = barometer_reference_pressure}};
      String data_format[data_values_count] = {"float", "uint8", "uint8", "uint8", "uint8", "float", "uint8", "uint8", "uint8", "uint8", "int8", "float", "float", "float", "int8", "float", "float", "float", "int8", "float", "float", "float", "float"};
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
      uint16_t apid = APID::PAYLOAD_LOCATION;
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
        int16_t &propotional_value,
        int16_t &integral_value,
        uint8_t &duty_cycle)
    {
      air_temperature = float_from_bytes(data[0], data[1], data[2], data[3]);
      baro_temperature = int8_from_bytes(data[4]);
      heatsink_temperature = int8_from_bytes(data[5]);
      pressure = uint32_from_bytes(data[6], data[7], data[8], data[9]);
      propotional_value = int16_from_bytes(data[10], data[11]);
      integral_value = int16_from_bytes(data[12], data[13]);
      duty_cycle = uint8_from_bytes(data[14]);
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
        const int16_t &propotional_value,
        const int16_t &integral_value,
        const uint8_t &duty_cycle)
    {
      uint16_t apid = APID::PAYLOAD_SUBSYSTEM_1_STATUS;
      uint16_t data_values_count = 7;
      Converter data_values[data_values_count] = {
          {.f = air_temperature},
          {.i8 = baro_temperature},
          {.i8 = heatsink_temperature},
          {.ui32 = pressure},
          {.i16 = propotional_value},
          {.i16 = integral_value},
          {.ui8 = duty_cycle}};
      String data_format[data_values_count] = {"float", "int8", "int8", "uint32", "int16", "int16", "uint8"};
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
     * @param estimated_latitude Estimated latitude
     * @param estimated_longitude Estimated longitude
     * @param estimated_altitude Estimated altitude
     * @param distance_From_base_1 Distance from base 1
     * @param distance_From_base_2 Distance from base 2
     * @param distance_From_base_3 Distance from base 3
     * @param time_since_last_ranging_base_1 Time since last ranging base 1
     * @param time_since_last_ranging_base_2 Time since last ranging base 2
     * @param time_since_last_ranging_base_3 Time since last ranging base 3
     */
    void get_values(
        const byte *data,
        float &estimated_latitude,
        float &estimated_longitude,
        float &estimated_altitude,
        float &distance_From_base_1,
        float &distance_From_base_2,
        float &distance_From_base_3,
        uint32_t &time_since_last_ranging_base_1,
        uint32_t &time_since_last_ranging_base_2,
        uint32_t &time_since_last_ranging_base_3)
    {
      estimated_latitude = float_from_bytes(data[0], data[1], data[2], data[3]);
      estimated_longitude = float_from_bytes(data[4], data[5], data[6], data[7]);
      estimated_altitude = float_from_bytes(data[8], data[9], data[10], data[11]);
      distance_From_base_1 = float_from_bytes(data[12], data[13], data[14], data[15]);
      distance_From_base_2 = float_from_bytes(data[16], data[17], data[18], data[19]);
      distance_From_base_3 = float_from_bytes(data[20], data[21], data[22], data[23]);
      time_since_last_ranging_base_1 = uint32_from_bytes(data[24], data[25], data[26], data[27]);
      time_since_last_ranging_base_2 = uint32_from_bytes(data[28], data[29], data[30], data[31]);
      time_since_last_ranging_base_3 = uint32_from_bytes(data[32], data[33], data[34], data[35]);
    }

    /**
     * @brief Create a ranging status packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param epoch_time Epoch time
     * @param subseconds Subseconds
     * @param estimated_latitude Estimated latitude
     * @param estimated_longitude Estimated longitude
     * @param estimated_altitude Estimated altitude
     * @param distance_From_base_1 Distance from base 1
     * @param distance_From_base_2 Distance from base 2
     * @param distance_From_base_3 Distance from base 3
     * @param time_since_last_ranging_base_1 Time since last ranging base 1
     * @param time_since_last_ranging_base_2 Time since last ranging base 2
     * @param time_since_last_ranging_base_3 Time since last ranging base 3
     *
     * @return Pointer to ranging status packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
        const uint32_t &epoch_time,
        const uint16_t &subseconds,
        const float &estimated_latitude,
        const float &estimated_longitude,
        const float &estimated_altitude,
        const float &distance_From_base_1,
        const float &distance_From_base_2,
        const float &distance_From_base_3,
        const uint32_t &time_since_last_ranging_base_1,
        const uint32_t &time_since_last_ranging_base_2,
        const uint32_t &time_since_last_ranging_base_3)
    {
      uint16_t apid = APID::PAYLOAD_SUBSYSTEM_2_STATUS;
      uint16_t data_values_count = 9;
      Converter data_values[data_values_count] = {
          {.f = estimated_latitude},
          {.f = estimated_longitude},
          {.f = estimated_altitude},
          {.f = distance_From_base_1},
          {.f = distance_From_base_2},
          {.f = distance_From_base_3},
          {.ui32 = time_since_last_ranging_base_1},
          {.ui32 = time_since_last_ranging_base_2},
          {.ui32 = time_since_last_ranging_base_3}};
      String data_format[data_values_count] = {"float", "float", "float", "float", "float", "float", "uint32", "uint32", "uint32"};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telemetry_packet(apid, sequence_count, epoch_time, subseconds, data_values_count, data_format, data_values, data_length);

      packet_length = 12 + data_length;

      return packet;
    }
  }
}
