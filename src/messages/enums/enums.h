#pragma once

namespace CCSDS_Enums
{
  enum APID
  {
    BALLOON_TELECOMMAND = 0,
    BALLOON_TELECOMMAND_ACKNOWLEDGEMENT = 1,
    BALLOON_SYSTEM_STATUS = 2,
    BALLOON_CONFIGURATION = 3,
    BALLOON_LOCATION = 4,
    BALLOON_SUBSYSTEM_1_STATUS = 5,

    PAYLOAD_TELECOMMAND = 100,
    PAYLOAD_TELECOMMAND_ACKNOWLEDGEMENT = 101,
    PAYLOAD_SYSTEM_STATUS = 102,
    PAYLOAD_CONFIGURATION = 103,
    PAYLOAD_LOCATION = 104,
    PAYLOAD_SUBSYSTEM_1_STATUS = 105,
    PAYLOAD_SUBSYSTEM_2_STATUS = 106,

    BASE_STATION_1_TELECOMMAND = 1000,
    BASE_STATION_1_TELECOMMAND_ACKNOWLEDGEMENT = 1001,
    BASE_STATION_1_SYSTEM_STATUS = 1002,
    BASE_STATION_1_CONFIGURATION = 1003,
    BASE_STATION_1_LOCATION = 1004,
    BASE_STATION_1_SUBSYSTEM_1_STATUS = 1005,
    BASE_STATION_1_SUBSYSTEM_2_STATUS = 1006,

    BASE_STATION_2_TELECOMMAND = 1100,
    BASE_STATION_2_TELECOMMAND_ACKNOWLEDGEMENT = 1101,
    BASE_STATION_2_SYSTEM_STATUS = 1102,
    BASE_STATION_2_CONFIGURATION = 1103,
    BASE_STATION_2_LOCATION = 1104,
    BASE_STATION_2_SUBSYSTEM_1_STATUS = 1105,
    BASE_STATION_2_SUBSYSTEM_2_STATUS = 1106,

    BASE_STATION_3_TELECOMMAND = 1200,
    BASE_STATION_3_TELECOMMAND_ACKNOWLEDGEMENT = 1201,
    BASE_STATION_3_SYSTEM_STATUS = 1202,
    BASE_STATION_3_CONFIGURATION = 1203,
    BASE_STATION_3_LOCATION = 1204,
    BASE_STATION_3_SUBSYSTEM_1_STATUS = 1205,
    BASE_STATION_3_SUBSYSTEM_2_STATUS = 1206
  };

  enum PacketID
  {
    SET_CONFIGURATION = 0,
    REQUEST_CONFIGURATION = 1,
    REQUEST_SUBSYSTEM_1_STATUS = 2,
    BASE_STATION_SET_ANGLES = 5,
  };

  enum LoRa_TX_Power
  {
    DO_NOT_CHANGE,
    TX_POWER_1,
    TX_POWER_2,
    TX_POWER_3,
    TX_POWER_4,
    TX_POWER_5,
    TX_POWER_6,
    TX_POWER_7,
    TX_POWER_8,
    TX_POWER_9,
    TX_POWER_10,
    TX_POWER_11,
    TX_POWER_12,
    TX_POWER_13,
    TX_POWER_14,
    TX_POWER_15,
    TX_POWER_16,
    TX_POWER_17,
    TX_POWER_18,
    TX_POWER_19,
    TX_POWER_20,
    TX_POWER_21,
    TX_POWER_22,
    TX_POWER_23,
    TX_POWER_24,
    TX_POWER_25,
    TX_POWER_MAX = 255
  };

  enum LoRa_Spreading_Factor
  {
    SF_DO_NOT_CHANGE,
    SF_6,
    SF_7,
    SF_8,
    SF_9,
    SF_10,
    SF_11,
    SF_12,
  };

  enum LoRa_Bandwidth
  {
    BW_DO_NOT_CHANGE,
    BW_7_8,
    BW_10_4,
    BW_15_6,
    BW_20_8,
    BW_31_25,
    BW_41_7,
    BW_62_5,
    BW_125,
    BW_250,
    BW_500,
  };

  enum LoRa_Coding_Rate
  {
    CR_DO_NOT_CHANGE,
    CR_4_5,
    CR_4_6,
    CR_4_7,
    CR_4_8,
  };
}

