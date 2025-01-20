#pragma once

namespace CCSDS_Enums
{
  enum BalloonAPID
  {
    TELECOMMAND = 0,
    TELECOMMAND_ACKNOWLEDGEMENT = 1,
    SYSTEM_STATUS = 2,
    CONFIGURATION = 3,
    LOCATION = 4,
    SUBSYSTEM_1_STATUS = 5
  };

  enum BalloonPacketID
  {
    SET_CONFIGURATION = 0,
    REQUEST_CONFIGURATION = 1,
    REQUEST_SUBSYSTEM_1_STATUS = 2
  };

  enum PayloadAPID
  {
    TELECOMMAND = 100,
    TELECOMMAND_ACKNOWLEDGEMENT = 101,
    SYSTEM_STATUS = 102,
    CONFIGURATION = 103,
    LOCATION = 104,
    SUBSYSTEM_1_STATUS = 105
  };

  enum PayloadPacketID
  {
    SET_CONFIGURATION = 0,
    REQUEST_CONFIGURATION = 1,
    REQUEST_SUBSYSTEM_1_STATUS = 2
  };

  enum BaseStation1APID
  {
    TELECOMMAND = 1000,
    TELECOMMAND_ACKNOWLEDGEMENT = 1001,
    SYSTEM_STATUS = 1002,
    CONFIGURATION = 1003,
    LOCATION = 1004,
    SUBSYSTEM_1_STATUS = 1005,
    SUBSYSTEM_2_STATUS = 1006
  };

  enum BaseStation2APID
  {
    TELECOMMAND = 1100,
    TELECOMMAND_ACKNOWLEDGEMENT = 1101,
    SYSTEM_STATUS = 1102,
    CONFIGURATION = 1103,
    LOCATION = 1104,
    SUBSYSTEM_1_STATUS = 1105,
    SUBSYSTEM_2_STATUS = 1106
  };

  enum BaseStation3APID
  {
    TELECOMMAND = 1200,
    TELECOMMAND_ACKNOWLEDGEMENT = 1201,
    SYSTEM_STATUS = 1202,
    CONFIGURATION = 1203,
    LOCATION = 1204,
    SUBSYSTEM_1_STATUS = 1205,
    SUBSYSTEM_2_STATUS = 1206
  };

  enum BaseStationPacketID
  {
    SET_CONFIGURATION = 0,
    REQUEST_CONFIGURATION = 1,
    SET_ANGLES = 5
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
    DO_NOT_CHANGE,
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
    DO_NOT_CHANGE,
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
    DO_NOT_CHANGE,
    CR_4_5,
    CR_4_6,
    CR_4_7,
    CR_4_8,
  };
}

