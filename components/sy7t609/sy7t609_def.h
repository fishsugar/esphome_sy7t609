// The SY7T609+S1 is an energy measurement processor (EMP) designed for monitoring any 2-wire circuit.
// Datasheet: https://datasheet4u.com/pdf-down/S/Y/7/SY7T609+S1-Silergy.pdf

#pragma once

#include <cstddef>
#include <cstdint>
#include "esphome/core/helpers.h"

namespace esphome::sy7t609 {

inline constexpr uint8_t SSI_REQ_HEADER_CODE = 0xAA;
inline constexpr size_t SSI_REQ_HEADER_SIZE = 2;
inline constexpr size_t SSI_REQ_PAYLOAD_SIZE_SELECT = 3;
inline constexpr size_t SSI_REQ_PAYLOAD_SIZE_WRITE_3BYTE = 4;
inline constexpr size_t SSI_REQ_PAYLOAD_SIZE_READ_3BYTE = 1;
inline constexpr size_t SSI_REQ_TRAILING_SIZE = 1;
inline constexpr size_t SSI_REQ_MAX_PACKET_SIZE = 255;
inline constexpr size_t SSI_RESP_WITHDATA_HEADER_SIZE = 2;
inline constexpr size_t SSI_RESP_DATA_SIZE_3BYTE = 3;
inline constexpr size_t SSI_RESP_WITHDATA_TAILING_SIZE = 1;

inline constexpr uint8_t SSI_CMD_CLEAR_ADDRESS = 0xA0;
inline constexpr uint8_t SSI_CMD_SELECT_REGISTER_ADDRESS = 0xA3;
inline constexpr uint8_t SSI_CMD_READ_REGITSTER_3BYTES = 0xE3;
inline constexpr uint8_t SSI_CMD_WRITE_RETISTER_3BYTES = 0xD3;

enum SsiReplyCode : uint8_t {
  REPLY_CODE_ACK_WITH_DATA = 0xAA,            ///< Acknowledge with data.
  REPLY_CODE_AUTO_REPORTING_HEADER = 0xAE,    ///< Auto Reporting Header (with data).
  REPLY_CODE_ACK_WITHOUT_DATA = 0xAD,         ///< Acknowledge without data.
  REPLY_CODE_NEGATIVE_ACK = 0xB0,             ///< Negative Acknowledge (NACK).
  REPLY_CODE_COMMAND_NOT_IMPLEMENTED = 0xBC,  ///< Command not implemented.
  REPLY_CODE_CHECKSUM_FAILED = 0xBD,          ///< Checksum failed.
  REPLY_CODE_BUFFER_OVERFLOW = 0xBF,          ///< Buffer overflow (or packet too long).
};

/// @brief Commands used in command register
enum CommandRegistryCode : uint32_t {
  CMD_REG_SAVE_TO_FLASH = 0xACC200,           ///< Save to flash the calibration coefficients and system defaults
  CMD_REG_AUTO_REPORT_OFF = 0x0AE000,         ///< Clear Control.ar
  CMD_REG_AUTO_REPORT_ON = 0x0AE001,          ///< Set Control.ar
  CMD_REG_CLEAR_ENGERGY_COUNTERS = 0xEC0000,  ///< Clear All Energy Counters
  CMD_REG_SOFT_RESET = 0xBD0000,              ///< Invoke Soft-Reset
  CMD_REG_CALIBRATE_POWER = 0xCA0100,         ///< Calibrate Current Gain using powertarget
  CMD_REG_CALIBRATE_VRMS = 0xCA0020,          ///< Calibrate Voltage Gain using vrmstarget
  CMD_REG_CALIBRATE_IRMS = 0xCA0010,          ///< Calibrate Current Gain using irmstarget
  CMD_REG_CALIBRATE_VOFFS = 0xCA0008,         ///< Calibrate Voltage Offset using vavgtarget
  CMD_REG_CALIBRATE_IOFFS = 0xCA0004,         ///< Calibrate Current Offset using iavgtarget
  CMD_REG_CALIBRATE_CTEMP = 0xCA0001,         ///< Calibrate Chip Temperature
  CMD_REG_CLEAR_FLASH_STORAGE_0 = 0xACC000,
  CMD_REG_CLEAR_FLASH_STORAGE_1 = 0XACC100,
};

/// @brief SY7T609 register value (24bits)
struct RegValue24 {
  uint32_t raw;

  inline static RegValue24 from_u24(uint32_t value) { return {value}; }

  /// @brief Read as data type U24
  inline uint32_t u24_value() const { return this->raw; }

  /// @brief Read as data type S24
  inline int32_t s24_value() const {
    int32_t v = static_cast<int32_t>(this->raw);
    return (v & 0x800000) ? (v | 0xFF000000) : v;
  }

  /// @brief Read as data type like S.21/S.23
  template<int FP> inline float s24_value() const {
    return static_cast<float>(this->s24_value()) / static_cast<float>(1 << FP);
  }

  /// @brief Read as data type like U.21/U.23/U.24
  template<int FP> inline float u24_value() const {
    return static_cast<float>(this->u24_value()) / static_cast<float>(1 << FP);
  }
};

/// @brief SY7T609 register UART address (12bits)
enum RegAddr : uint16_t {
  REG_COMMAND = 0x000,       ///< Command Register
  REG_FWVERSION = 0x003,     ///< Firmware release version in hex format
  REG_CONTROL = 0x006,       ///< Control Register
  REG_DIVISOR = 0x00C,       ///< Number of samples used for last accumulation interval
  REG_FRAMEL = 0x00F,        ///< Low Rate Accumulation interval counter
  REG_FRAMEH = 0x012,        ///< Low Rate Accumulation interval counter
  REG_ALARMS = 0x015,        ///< Alarm Status Registers
  REG_DIO_STATE = 0x018,     ///< State of DIO Outputs
  REG_SPI_WR_DATA = 0x01B,   ///< Indirect Register Access
  REG_SPI_WR_ADDR = 0x01E,   ///< Indirect Register Access
  REG_SPI_RD_ADDR = 0x021,   ///< Indirect Register Access
  REG_SPI_RD_DATA = 0x024,   ///< Indirect Register Access
  REG_CTEMP = 0x027,         ///< Scaled Chip/Die Temperature
  REG_VAVG = 0x02D,          ///< Scaled Average Voltage
  REG_IAVG = 0x030,          ///< Scaled Average Current
  REG_VRMS = 0x033,          ///< Scaled RMS Voltage
  REG_IRMS = 0x036,          ///< Scaled RMS Current
  REG_POWER = 0x039,         ///< Scaled Active Power
  REG_VAR = 0x03C,           ///< Scaled Reactive Power
  REG_VA = 0x03F,            ///< Scaled Apparent Power
  REG_FREQUENCY = 0x042,     ///< Scaled Line Frequency
  REG_AVGPOWER = 0x045,      ///< Scaled Average Active Power
  REG_PF = 0x048,            ///< Scaled Power Factor
  REG_VFUND = 0x04B,         ///< Scaled Fundamental RMS Voltage
  REG_IFUND = 0x04E,         ///< Scaled Fundamental RMS Current
  REG_PFUND = 0x051,         ///< Scaled Fundamental Active Power
  REG_QFUND = 0x054,         ///< Scaled Fundamental Reactive Power
  REG_VAFUND = 0x057,        ///< Scaled Fundamental Apparent Power
  REG_VHARM = 0x05A,         ///< Scaled Harmonic RMS Voltage
  REG_IHARM = 0x05D,         ///< Scaled Harmonic RMS Current
  REG_PHARM = 0x060,         ///< Scaled Harmonic Active Power
  REG_QHARM = 0x063,         ///< Scaled Harmonic Reactive Power
  REG_VAHARM = 0x066,        ///< Scaled Harmonic Apparent Power
  REG_EPPCNT = 0x069,        ///< Positive Active Energy Count
  REG_EPMCNT = 0x06C,        ///< Negative Active Energy Count
  REG_EPNCNT = 0x06F,        ///< Net Active Energy Count
  REG_EQNCNT = 0x078,        ///< Net Reactive Energy Count
  REG_ESNCNT = 0x081,        ///< Net Apparent Energy Count
  REG_ILO = 0x084,           ///< Lowest scaled RMS Current result since last clear or reset
  REG_IHI = 0x087,           ///< Highest scaled RMS Current result since last clear or reset
  REG_IPEAK = 0x08A,         ///< Highest scaled instantaneous Current sample in last accumulation Interval
  REG_VLO = 0x08D,           ///< Lowest scaled RMS Voltage result since last clear or reset
  REG_VHI = 0x090,           ///< Highest scaled RMS Voltage result since last clear or reset
  REG_VPEAK = 0x093,         ///< Highest scaled instantaneous Voltage sample in last accumulation Interval
  REG_DIO_DIR = 0x099,       ///< DIO Direction Setting
  REG_DIO_POL = 0x09C,       ///< DIO Polarity Setting
  REG_DIO_SET = 0x09F,       ///< DIO Set High State Register
  REG_DIO_RST = 0x0A2,       ///< DIO Reset to Low State Register
  REG_MASK1 = 0x0A5,         ///< Alarm mask bits for DIO1 pin
  REG_MASK5 = 0x0A8,         ///< Alarm mask bits for DIO5 pin
  REG_MASK7 = 0x0AB,         ///< Alarm mask bits for DIO7 pin
  REG_MASK8 = 0x0AE,         ///< Alarm mask bits for DIO8 pin
  REG_ALARM_STICKY = 0x0B7,  ///< Bits to control auto-reset of alarm status
  REG_ALARM_SET = 0x0BA,     ///< Sets corresponding alarm bits
  REG_ALARM_RESET = 0x0BD,   ///< Clears corresponding alarm bits
  REG_BUCKETL = 0x0C0,       ///< Bucket Register(s) for Energy Accumulation
  REG_BUCKETH = 0x0C3,       ///< Combined Bucket is a U48.24
  REG_PHASECOMP = 0x0C6,     ///< Phase compensation (high-rate samples)
  REG_IROFF = 0x0C9,         ///< RMS Current Offset Adjust
  REG_VROFF = 0x0CC,         ///< RMS Voltage Offset Adjust
  REG_POFF = 0x0CF,          ///< Power Offset Adjust
  REG_IGAIN = 0x0D5,         ///< Current Gain Setting
  REG_VGAIN = 0x0D8,         ///< Voltage Gain Setting
  REG_IOFFS = 0x0DB,         ///< Current DC offset.
  REG_VOFFS = 0x0DE,         ///< Voltage DC offset.
  REG_TGAIN = 0x0E1,         ///< Die temperature gain setting.
  REG_TOFFS = 0x0E4,         ///< Die Temperature offset.
  REG_ISCALE = 0x0ED,        ///< Current scaling register.
  REG_VSCALE = 0x0F0,        ///< Voltage scaling register.
  REG_PSCALE = 0x0F3,        ///< Power scaling register.
  REG_PFSCALE = 0x0F6,       ///< Power Factor scaling register.
  REG_FSCALE = 0x0F9,        ///< Frequency scaling register.
  REG_TSCALE = 0x0FC,        ///< Temperature Scaling register.
  REG_ACCUMCYC = 0x102,      ///< Line Cycles to set Accumulation Interval at
  REG_ACCUM = 0x105,         ///< Accumulation Interval for low rate calculations (RMS, etc.).
  REG_CALCYC = 0x108,        ///< \# of accumulation intervals to average in each Calibration Iteration.
  REG_CALITR = 0x10B,        ///< \# of Iterations in each Calibration command.
  REG_HARM = 0x10E,          ///< Harmonic selector.
  REG_IAVGTARGET = 0x111,    ///< Average Current target for Calibration.
  REG_VAVGTARGET = 0x114,    ///< Average Voltage target for Calibration.
  REG_IRMSTARGET = 0x117,    ///< RMS Current target for Calibration.
  REG_VRMSTARGET = 0x11A,    ///< RMS Voltage target for Calibration.
  REG_POWERTARGET = 0x11D,   ///< Active Power target for Calibration.
  REG_BAUD = 0x120,          ///< Baud Rate Register for UART
  REG_VHYST = 0x129,         ///< Hysteresis Voltage for ZC detection
  REG_ACDCV = 0x12C,         ///< Average Voltage Threshold to Disable HPF
  REG_ACDCI = 0x12F,         ///< Average Current Threshold to Disable HPF
  REG_VSURGETH = 0x135,      ///< Voltage threshold above which VSURGE alarm is activated.
  REG_VSAGTH = 0x138,        ///< Voltage threshold below which VSAG alarm will be activated.
  REG_VMINTH = 0x13B,        ///< Voltage threshold below which UNDERVOLT alarm will be activated.
  REG_VMAXTH = 0x13E,        ///< Voltage threshold above which OVERVOLT alarm will be activated.
  REG_VDROPTH = 0x141,       ///< Voltage threshold below which VDROPOUT alarm will be activated.
  REG_IMAXTH = 0x144,        ///< Current High Alarm Limit.
  REG_PMAXTH = 0x147,        ///< Power High Alarm Limit.
  REG_TMINTH = 0x14A,        ///< Die temperature threshold below which the UNDERTEMP alarm will be activated.
  REG_TMAXTH = 0x14D,        ///< Die temperature threshold above which the OVERTEMP alarm will be activated.
  REG_FMINTH = 0x153,        ///< Line Frequency threshold below which the UNDERFREQ alarm will be activated.
  REG_FMAXTH = 0x156,        ///< Line frequency threshold above which the OVERFREQ alarm will be activated.
  REG_VDROPHOLD = 0x159,     ///< Number of ADC samples below VdropTh (Threshold) to generate alarm
  REG_VMINHOLD = 0x15C,      ///< \# of consecutive accumulation intervals in which the RMS voltage must exceed the
                             ///< specified limit before the UNDERVOLT or OVERVOLT alarms will be activated.
  REG_IMAXHOLD = 0x15F,      ///< \# of consecutive accumulation intervals in which the RMS current must exceed ImaxTh
                             ///< before the OVERCURRENT alarm will be activated.
  REG_PMAXHOLD = 0x162,      ///< \# of consecutive accumulation intervals in which power must exceed the PMAX threshold
                             ///< before the OVERPOWER alarm will be activated.
  REG_TMINHOLD = 0x165,   ///< \# of consecutive accumulation intervals in which the die temperature must exceed either
                          ///< TMIN or TMAX before the OVERTEMP or UNDERTEMP alarm will be activated.
  REG_FMINHOLD = 0x168,   ///< \# of consecutive accumulation intervals in which the line frequency must exceed either
                          ///< FMIN or FMAX before the UNDERFREQ or OVERFREQ alarm will be activated.
  REG_TMINCNT = 0x16B,    ///< The number of times an UNDERTEMP event has been detected.
  REG_TMAXCNT = 0x16E,    ///< The number of times an OVERTEMP event has been detected.
  REG_VMINCNT = 0x171,    ///< The number of times an UNDERVOLT event has been detected.
  REG_VMAXCNT = 0x174,    ///< The number of times an OVERVOLT event has been detected.
  REG_IMAXCNT = 0x177,    ///< The number of times an OVERCURRENT event has been detected.
  REG_PMAXCNT = 0x17A,    ///< The number of times an OVERPOWER event has been detected.
  REG_FMINCNT = 0x17D,    ///< The number of times an UNDERFREQ event has been detected.
  REG_FMAXCNT = 0x180,    ///< The number of times an OVERFREQ event has been detected.
  REG_VSAGCNT = 0x183,    ///< The number of times a VSAG event has been detected.
  REG_VSURGECNT = 0x186,  ///< The number of times a VSURGE event has been detected.
};

union ControlRegister {
  uint32_t raw : 24;
  struct {
    bool hpfv : 1;      ///< Voltage HPF
    bool hpfi : 1;      ///< Current HPF
    uint8_t : 1;        ///< Reserved
    bool ar : 1;        ///< Enable Auto Reporting
    bool tc : 1;        ///< Enable Gain/Temperature compensation
    uint8_t : 3;        ///< Reserved
    uint8_t : 1;        ///< Reserved
    bool arrst : 1;     ///< Set Auto-Report (ar) on reset
    bool ct : 1;        ///< Stop chip/die temperature update: 1=stop update; 0=update
    bool pshift : 1;    ///< PSCALE shift: 1=PSCALE<<8; 0=PSCALE
    bool acdcv : 1;     ///< Auto Voltage HPF Control: 1=Automatic control; 0=use hpfv
    bool acdci : 1;     ///< Auto Current HPF Control: 1=Automatic control; 0=use hpfi
    bool swapv : 1;     ///< Swap Voltage Input: 1= V←(AVN-AVP); 0= V←(AVP-AVN)
    bool swapi : 1;     ///< Swap Current Input: 1= V←(AIN-AIP); 0= V←(AIP-AIN)
    uint8_t : 1;        ///< Reserved
    bool negpower : 1;  ///< Negative Active Power not allowed. 1= P←MAX(P,0); 0= P←P;
    bool ishift : 1;    ///< ISCALE shift: 1=ISCALE<<8; 0=ISCALE
    uint8_t : 5;        ///< Reserved
  } PACKED;
};
}  // namespace esphome::sy7t609
