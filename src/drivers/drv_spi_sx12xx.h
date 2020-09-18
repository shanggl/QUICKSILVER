#pragma once

#include <stdint.h>

enum sx12xx_modifiers {
  SX12XX_READ = 0b00000000,
  SX12XX_WRITE = 0b10000000,
};

enum sx12xx_registers {
  SX12XX_FIFO = 0x00,
  SX12XX_OP_MODE = 0x01,
  SX12XX_FRF_MSB = 0x06,
  SX12XX_FRF_MID = 0x07,
  SX12XX_FRF_LSB = 0x08,
  SX12XX_PA_CONFIG = 0x09,
  SX12XX_PA_RAMP = 0x0A,
  SX12XX_OCP = 0x0B,
  SX12XX_LNA = 0x0C,
  SX12XX_FIFO_ADDR_PTR = 0x0D,
  SX12XX_FIFO_TX_BASE_ADDR = 0x0E,
  SX12XX_FIFO_RX_BASE_ADDR = 0x0F,
  SX12XX_FIFO_RX_CURRENT_ADDR = 0x10,
  SX12XX_IRQ_FLAGS_MASK = 0x11,
  SX12XX_IRQ_FLAGS = 0x12,
  SX12XX_RX_NB_BYTES = 0x13,
  SX12XX_RX_HEADER_CNT_VALUE_MSB = 0x14,
  SX12XX_RX_HEADER_CNT_VALUE_LSB = 0x15,
  SX12XX_RX_PACKET_CNT_VALUE_MSB = 0x16,
  SX12XX_RX_PACKET_CNT_VALUE_LSB = 0x17,
  SX12XX_MODEM_STAT = 0x18,
  SX12XX_PKT_SNR_VALUE = 0x19,
  SX12XX_PKT_RSSI_VALUE = 0x1A,
  SX12XX_RSSI_VALUE = 0x1B,
  SX12XX_HOP_CHANNEL = 0x1C,
  SX12XX_MODEM_CONFIG_1 = 0x1D,
  SX12XX_MODEM_CONFIG_2 = 0x1E,
  SX12XX_MODEM_CONFIG_3 = 0x26,
  SX12XX_SYMB_TIMEOUT_LSB = 0x1F,
  SX12XX_PREAMBLE_MSB = 0x20,
  SX12XX_PREAMBLE_LSB = 0x21,
  SX12XX_PAYLOAD_LENGTH = 0x22,
  SX12XX_MAX_PAYLOAD_LENGTH = 0x23,
  SX12XX_HOP_PERIOD = 0x24,
  SX12XX_FIFO_RX_BYTE_ADDR = 0x25,
  SX12XX_FEI_MSB = 0x28,
  SX12XX_FEI_MID = 0x29,
  SX12XX_FEI_LSB = 0x2A,
  SX12XX_RSSI_WIDEBAND = 0x2C,
  SX12XX_DETECT_OPTIMIZE = 0x31,
  SX12XX_INVERT_IQ = 0x33,
  SX12XX_DETECTION_THRESHOLD = 0x37,
  SX12XX_SYNC_WORD = 0x39,
  SX12XX_DIO_MAPPING_1 = 0x40,
  SX12XX_DIO_MAPPING_2 = 0x41,
  SX12XX_VERSION = 0x42,
};

enum sx12xx_modulation_modes {
  SX12XX_OPMODE_FSK_OOK = 0b00000000,
  SX12XX_OPMODE_LORA = 0b10000000,
  SX12XX_ACCESS_SHARED_REG_OFF = 0b00000000,
  SX12XX_ACCESS_SHARED_REG_ON = 0b01000000
};

typedef enum {
  SX12XX_OPMODE_SLEEP = 0b00000000,
  SX12XX_OPMODE_STANDBY = 0b00000001,
  SX12XX_OPMODE_FSTX = 0b00000010,
  SX12XX_OPMODE_TX = 0b00000011,
  SX12XX_OPMODE_FSRX = 0b00000100,
  SX12XX_OPMODE_RXCONTINUOUS = 0b00000101,
  SX12XX_OPMODE_RXSINGLE = 0b00000110,
  SX12XX_OPMODE_CAD = 0b00000111,
} sx12xx_radio_op_modes_t;

typedef enum {
  SX12XX_BW_7_80_KHZ = 0b00000000,
  SX12XX_BW_10_40_KHZ = 0b00010000,
  SX12XX_BW_15_60_KHZ = 0b00100000,
  SX12XX_BW_20_80_KHZ = 0b00110000,
  SX12XX_BW_31_25_KHZ = 0b01000000,
  SX12XX_BW_41_70_KHZ = 0b01010000,
  SX12XX_BW_62_50_KHZ = 0b01100000,
  SX12XX_BW_125_00_KHZ = 0b01110000,
  SX12XX_BW_250_00_KHZ = 0b10000000,
  SX12XX_BW_500_00_KHZ = 0b10010000
} sx12xx_bandwidth_t;

typedef enum {
  SX12XX_SF_6 = 0b01100000,
  SX12XX_SF_7 = 0b01110000,
  SX12XX_SF_8 = 0b10000000,
  SX12XX_SF_9 = 0b10010000,
  SX12XX_SF_10 = 0b10100000,
  SX12XX_SF_11 = 0b10110000,
  SX12XX_SF_12 = 0b11000000
} sx12xx_spreading_factor_t;

typedef enum {
  SX12XX_CR_4_5 = 0b00000010,
  SX12XX_CR_4_6 = 0b00000100,
  SX12XX_CR_4_7 = 0b00000110,
  SX12XX_CR_4_8 = 0b00001000,
} sx12xx_coding_rate_t;

enum sx12xx_lna_mode {
  SX12XX_LNA_GAIN_0 = 0b00000000,    //  7     5     LNA gain setting:   not used
  SX12XX_LNA_GAIN_1 = 0b00100000,    //  7     5                         max gain
  SX12XX_LNA_GAIN_2 = 0b01000000,    //  7     5                         .
  SX12XX_LNA_GAIN_3 = 0b01100000,    //  7     5                         .
  SX12XX_LNA_GAIN_4 = 0b10000000,    //  7     5                         .
  SX12XX_LNA_GAIN_5 = 0b10100000,    //  7     5                         .
  SX12XX_LNA_GAIN_6 = 0b11000000,    //  7     5                         min gain
  SX12XX_LNA_GAIN_7 = 0b11100000,    //  7     5                         not used
  SX12XX_LNA_BOOST_OFF = 0b00000000, //  1     0     default LNA current
  SX12XX_LNA_BOOST_ON = 0b00000011,  //  1     0     150% LNA current
};

enum sx12xx_modem_config_3 {
  SX1278_LOW_DATA_RATE_OPT_OFF = 0b00000000, //  3     3     low data rate optimization disabled
  SX1278_LOW_DATA_RATE_OPT_ON = 0b00001000,  //  3     3     low data rate optimization enabled
  SX1278_AGC_AUTO_OFF = 0b00000000,          //  2     2     LNA gain set by REG_LNA
  SX1278_AGC_AUTO_ON = 0b00000100,           //  2     2     LNA gain set by internal AGC loop
};

enum sx12xx_ocp_mode {
  SX127X_OCP_OFF = 0b00000000,   //  5     5     PA overload current protection disabled
  SX127X_OCP_ON = 0b00100000,    //  5     5     PA overload current protection enabled
  SX127X_OCP_TRIM = 0b00001011,  //  4     0     OCP current: I_max(OCP_TRIM = 0b1011) = 100 mA
  SX127X_OCP_150MA = 0b00010010, //  4     0     OCP current: I_max(OCP_TRIM = 10010) = 150 mA
};

enum sx12xx_detect_optimize {
  SX12XX_DETECT_OPTIMIZE_SF_6 = 0b00000101,    //  2     0     SF6 detection optimization
  SX12XX_DETECT_OPTIMIZE_SF_7_12 = 0b00000011, //  2     0     SF7 to SF12 detection optimization
};

enum sx12xx_detection_threshold {
  SX12XX_DETECTION_THRESHOLD_SF_6 = 0b00001100,    //  7     0     SF6 detection threshold
  SX12XX_DETECTION_THRESHOLD_SF_7_12 = 0b00001010, //  7     0     SF7 to SF12 detection threshold
};

enum sx12xx_header_mode {
  SX1278_HEADER_EXPL_MODE = 0b00000000, //  0     0     explicit header mode
  SX1278_HEADER_IMPL_MODE = 0b00000001, //  0     0     implicit header mode
};

enum sx12xx_pa_config {
  SX12XX_PA_SELECT_RFO = 0b00000000,    //  7     7     RFO pin output, power limited to +14 dBm
  SX12XX_PA_SELECT_BOOST = 0b10000000,  //  7     7     PA_BOOST pin output, power limited to +20 dBm
  SX12XX_OUTPUT_POWER = 0b00001111,     //  3     0     output power: P_out = 17 - (15 - OUTPUT_POWER) [dBm] for PA_SELECT_BOOST
  SX12XX_MAX_OUTPUT_POWER = 0b01110000, //              Enable max output power
};

void sx12xx_init();
uint8_t sx12xx_detect();

void sx12xx_set_mode(sx12xx_radio_op_modes_t mode);

uint8_t sx12xx_read_reg(uint8_t reg);
uint8_t sx12xx_write_reg(uint8_t reg, uint8_t data);

void sx12xx_write_reg_burst(uint8_t reg, uint8_t *data, uint8_t size);

void sx12xx_read_fifo(uint8_t *data, uint8_t size);
void sx12xx_write_fifo(uint8_t *data, uint8_t size);

uint8_t sx12xx_read_dio0();
uint32_t sx12xx_packet_time();