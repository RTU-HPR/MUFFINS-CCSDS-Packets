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
     * @brief Create a set configuration packet
     *
     * @param packet_length Created packet length
     * @param sequence_count Sequence count
     * @param lora_frequency LoRa frequency
     * @param lora_tx_power LoRa TX power
     * @param lora_spreading_factor LoRa spreading factor
     * @param lora_bandwidth LoRa bandwidth
     * @param lora_coding_rate LoRa coding rate
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
        const float &barometer_reference_pressure)
    {
      uint16_t apid = APID::PAYLOAD_TELECOMMAND;
      uint16_t packet_id = PacketID::SET_CONFIGURATION;
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