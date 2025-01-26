#include "../enums/enums.h"
#include "../../MUFFINS_CCSDS_Packets.h"

using namespace CCSDS_Enums;

namespace PayloadTelecommands
{
  namespace SetConfiguration
  {
    /**
     * @brief Get values from set configuration data field
     *
     * @param data Data field of set configuration packet
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
     * @brief Create a set configuration packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
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
     * @return Pointer to set configuration packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count,
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
      uint16_t apid = APID::PAYLOAD_TELECOMMAND;
      uint16_t packet_id = PacketID::SET_CONFIGURATION;
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

      byte *packet = create_ccsds_telecommand_packet(apid, sequence_count, packet_id, data_values_count, data_format, data_values, data_length);

      packet_length = 8 + data_length;

      return packet;
    }
  }

  namespace RequestConfiguration
  {
    /**
     * @brief Create a request configuration packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     *
     * @return Pointer to request configuration packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count)
    {
      uint16_t apid = APID::PAYLOAD_TELECOMMAND;
      uint16_t packet_id = PacketID::REQUEST_CONFIGURATION;
      uint16_t data_values_count = 0;
      Converter data_values[data_values_count] = {};
      String data_format[data_values_count] = {};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telecommand_packet(apid, sequence_count, packet_id, data_values_count, data_format, data_values, data_length);

      packet_length = 8 + data_length;

      return packet;
    }
  }

  namespace RequestSubsystem1Status
  {
    /**
     * @brief Create a request subsystem 1 status packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     *
     * @return Pointer to request subsystem 1 status packet
     */
    byte *create(
        uint8_t &packet_length,
        const uint16_t &sequence_count)
    {
      uint16_t apid = APID::PAYLOAD_TELECOMMAND;
      uint16_t packet_id = PacketID::REQUEST_SUBSYSTEM_1_STATUS;
      uint16_t data_values_count = 0;
      Converter data_values[data_values_count] = {};
      String data_format[data_values_count] = {};
      uint16_t data_length = 0;

      byte *packet = create_ccsds_telecommand_packet(apid, sequence_count, packet_id, data_values_count, data_format, data_values, data_length);

      packet_length = 8 + data_length;

      return packet;
    }
  }
}