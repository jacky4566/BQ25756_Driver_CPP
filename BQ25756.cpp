#include "BQ25756.h"

#define BQ25756_I2C_ADDR 0x6B

#define BQ25756_WHOAMI 0x02

#define REG0X00_CHARGE_VOLTAGE_LIMIT 0x00                   // Charge Voltage Limit
#define REG0X02_CHARGE_CURRENT_LIMIT 0x02                   // Charge Current Limit
#define REG0X06_INPUT_CURRENT_DPM_LIMIT 0x06                // Input Current DPM Limit
#define REG0X08_INPUT_VOLTAGE_DPM_LIMIT 0x08                // Input Voltage DPM Limit
#define REG0X0A_REVERSE_MODE_INPUT_CURRENT 0x0A             // Reverse Mode Input Current Limit
#define REG0X0C_REVERSE_MODE_INPUT_VOLTAGE 0x0C             // Reverse Mode Input Voltage Limit
#define REG0X10_PRECHARGE_CURRENT_LIMIT 0x10                // Precharge Current Limit
#define REG0X12_TERMINATION_CURRENT_LIMIT 0x12              // Termination Current Limit
#define REG0X14_PRECHARGE_TERMINATION_CTRL 0x14             // Precharge and Termination Control
#define REG0X15_TIMER_CONTROL 0x15                          // Timer Control
#define REG0X16_THREE_STAGE_CHARGE_CTRL 0x16                // Three-Stage Charge Control
#define REG0X17_CHARGER_CONTROL 0x17                        // Charger Control
#define REG0X18_PIN_CONTROL 0x18                            // Pin Control
#define REG0X19_POWER_PATH_REVERSE_MODE_CTRL 0x19           // Power Path and Reverse Mode Control
#define REG0X1A_MPPT_CONTROL 0x1A                           // MPPT Control
#define REG0X1B_TS_CHARGING_THRESHOLD_CTRL 0x1B             // TS Charging Threshold Control
#define REG0X1C_TS_CHARGING_REGION_BEHAVIOR 0x1C            // TS Charging Region Behavior Control
#define REG0X1D_TS_REVERSE_MODE_THRESHOLD 0x1D              // TS Reverse Mode Threshold Control
#define REG0X1E_REVERSE_UNDERVOLTAGE_CTRL 0x1E              // Reverse Undervoltage Control
#define REG0X1F_VAC_MAX_POWER_POINT_DETECT 0x1F             // VAC Max Power Point Detected
#define REG0X21_CHARGER_STATUS_1 0x21                       // Charger Status 1
#define REG0X22_CHARGER_STATUS_2 0x22                       // Charger Status 2
#define REG0X23_CHARGER_STATUS_3 0x23                       // Charger Status 3
#define REG0X24_FAULT_STATUS 0x24                           // Fault Status
#define REG0X25_CHARGER_FLAG_1 0x25                         // Charger Flag 1
#define REG0X26_CHARGER_FLAG_2 0x26                         // Charger Flag 2
#define REG0X27_FAULT_FLAG 0x27                             // Fault Flag
#define REG0X28_CHARGER_MASK_1 0x28                         // Charger Mask 1
#define REG0X29_CHARGER_MASK_2 0x29                         // Charger Mask 2
#define REG0X2A_FAULT_MASK 0x2A                             // Fault Mask
#define REG0X2B_ADC_CONTROL 0x2B                            // ADC Control
#define REG0X2C_ADC_CHANNEL_CONTROL 0x2C                    // ADC Channel Control
#define REG0X2D_IAC_ADC 0x2D                                // IAC ADC
#define REG0X2F_IBAT_ADC 0x2F                               // IBAT ADC
#define REG0X31_VAC_ADC 0x31                                // VAC ADC
#define REG0X33_VBAT_ADC 0x33                               // VBAT ADC
#define REG0X37_TS_ADC 0x37                                 // TS ADC
#define REG0X39_VFB_ADC 0x39                                // VFB ADC
#define REG0X3B_GATE_DRIVER_STRENGTH_CTRL 0x3B              // Gate Driver Strength Control
#define REG0X3C_GATE_DRIVER_DEAD_TIME_CTRL 0x3C             // Gate Driver Dead Time Control
#define REG0X3D_PART_INFORMATION 0x3D                       // Part Information
#define REG0X62_REVERSE_MODE_BATTERY_DISCHARGE_CURRENT 0x62 // Reverse Mode Battery Discharge Current

// Interrupt mask flags 1 for 0x28
#define ADC_DONE_MASK (1 << 7)
#define IAC_DPM_MASK (1 << 6)
#define VAC_DPM_MASK (1 << 5)
#define WD_MASK (1 << 3)
#define CV_TMR_MASK (1 << 1)
#define CHARGE_MASK (1 << 0)

// Interrupt mask flags 2 for 0x29
#define PG_MASK (1 << 7)
#define TS_MASK (1 << 4)
#define REVERSE_MASK (1 << 3)
#define FSW_SYNC_MASK (1 << 1)
#define MPPT_MASK (1 << 0)

// Interrupt mask flags 2 for 0x2A
#define VAC_UV_MASK (1 << 7)
#define VAC_OV_MASK (1 << 6)
#define IBAT_OCP_MASK (1 << 5)
#define VBAT_OV_MASK (1 << 4)
#define TSHUT_MASK (1 << 3)
#define CHG_TMR_MASK (1 << 2)
#define DRV_OKZ_MASK (1 << 1)

enum VBATLowV
{
    VBAT_LOWV_30_PERCENT = 0,   // 30% x VFB_REG
    VBAT_LOWV_55_PERCENT = 1,   // 55% x VFB_REG
    VBAT_LOWV_66_7_PERCENT = 2, // 66.7% x VFB_REG
    VBAT_LOWV_71_4_PERCENT = 3  // 71.4% x VFB_REG
};

enum VRECHG
{
    VRECHG_93_PERCENT = 0,   // 93.0% x VFB_REG
    VRECHG_94_3_PERCENT = 1, // 94.3% x VFB_REG
    VRECHG_95_2_PERCENT = 2, // 95.2% x VFB_REG
    VRECHG_97_6_PERCENT = 3  // 97.6% x VFB_REG
};

enum TopOffTimer
{
    DISABLE = 0,    // 00b = Disable
    MINUTES_15 = 1, // 01b = 15 minutes
    MINUTES_30 = 2, // 10b = 30 minutes
    MINUTES_45 = 3  // 11b = 45 minutes
};

enum WDTTimer
{
    DISABLE = 0,
    SECONDS_40 = 1,
    SECONDS_80 = 2,
    SECONDS_160 = 3
};

enum CHGTMR
{
    HOURS_5 = 0,
    HOURS_8 = 1,
    HOURS_12 = 2,
    HOURS_24 = 3
};

enum P_AND_O_TMR
{
    SECONDS_0 = 0,
    SECONDS_0_5 = 1,
    SECONDS_1 = 2,
    SECONDS_10 = 3
};

enum FULL_SWEEP_TMR
{
    MIN_3 = 0,
    MIN_10 = 1,
    MIN_15 = 2,
    MIN_20 = 3
};

enum REVERSEIBATLIMIT
{
    20A = 0,
    15A = 1,
    10A = 2,
    5A = 3
};

enum ADC_BIT_DEPTH
{
    15BIT = 0,
    14BIT = 1,
    13BIT = 2
};

enum CHARGESTATE
{
    CHARGE_STATE_NOT_CHARGING = 0, // Not charging
    CHARGE_STATE_TRICKLE = 1,      // Trickle Charge (VBAT < VBAT_SHORT)
    CHARGE_STATE_PRE_CHARGE = 2,   // Pre-Charge (VBAT < VBAT_LOWV)
    CHARGE_STATE_FAST_CHARGE = 3,  // Fast Charge (CC mode)
    CHARGE_STATE_TAPER_CHARGE = 4, // Taper Charge (CV mode)
    CHARGE_STATE_RESERVED = 5,     // Reserved
    CHARGE_STATE_TOP_OFF = 6,      // Top-off Timer Charge
    CHARGE_STATE_DONE = 7          // Charge Termination Done
};

enum TSStatus
{
    TS_STATUS_NORMAL = 1, // Normal
    TS_STATUS_WARM = 2,   // TS Warm
    TS_STATUS_COOL = 3,   // TS Cool
    TS_STATUS_COLD = 4,   // TS Cold
    TS_STATUS_HOT = 5     // TS Hot
};

enum MPPTSTATUS
{
    DISABLED = 1,
    STANDBY = 2,    // MPPT Enabled, But Not Running
    PANELSWEEP = 3, // Full Panel Sweep In Progress
    MPPTEN = 4,     // Max Power Voltage Detected
};

// Initializes the I2C interface
int BQ25756::begin()
{
    Wire.begin();
    byte whoami = whoami();
    if (whoami != BQ25756_WHOAMI)
        return whoami;
    return 0; // Success
}

// Set Volts and Amps
void BQ25756::setFBmVLimit(uint16_t milliVolts)
{
    milliVolts = min(1566, max(milliVolts, 1504)); // Range Check
    milliVolts = (milliVolts - 1540) / 2;          // Remove offset and apply bitstep
    busWrite2B(REG0X00_CHARGE_VOLTAGE_LIMIT, milliVolts);
}

void BQ25756::setChargeALimit(uint16_t milliAmps)
{
    milliAmps = min(20000, max(milliAmps, 400)); // Range Check
    milliAmps = (milliAmps / 50) << 2;           // Apply bitstep and offset
    busWrite2B(REG0X02_CHARGE_CURRENT_LIMIT, milliAmps);
}

void BQ25756::setInputALimit(uint16_t milliAmps)
{
    milliAmps = min(20000, max(milliAmps, 400)); // Range Check
    milliAmps = (milliAmps / 50) << 2;           // Apply bitstep and offset
    busWrite2B(REG0X06_INPUT_CURRENT_DPM_LIMIT, milliAmps);
}

void BQ25756::setInputVLimit(uint16_t milliVolts)
{
    milliVolts = min(65000, max(milliVolts, 4200)); // Range Check
    milliVolts = (milliVolts / 20) << 2;            // Apply bitstep and offset
    busWrite2B(REG0X08_INPUT_VOLTAGE_DPM_LIMIT, milliVolts);
}

void BQ25756::setReverseModeALimit(uint16_t milliAmps)
{
    milliAmps = min(20000, max(milliAmps, 400)); // Range Check
    milliAmps = (milliAmps / 50) << 2;           // Apply bitstep and offset
    busWrite2B(REG0X0A_REVERSE_MODE_INPUT_CURRENT, milliAmps);
}

void BQ25756::setReverseModeVLimit(uint16_t milliVolts)
{
    milliVolts = min(65000, max(milliVolts, 3300)); // Range Check
    milliVolts = (milliVolts / 20) << 2;            // Apply bitstep and offset
    busWrite2B(REG0X0C_REVERSE_MODE_INPUT_VOLTAGE, milliVolts);
}

void BQ25756::setPrechargeALimit(uint16_t milliAmps)
{
    milliAmps = min(10000, max(milliAmps, 250)); // Range Check
    milliAmps = (milliAmps / 50) << 2;           // Apply bitstep and offset
    busWrite2B(REG0X10_PRECHARGE_CURRENT_LIMIT, milliAmps);
}

void BQ25756::setTerminationALimit(uint16_t milliAmps)
{
    milliAmps = min(20000, max(milliAmps, 250)); // Range Check
    milliAmps = (milliAmps / 50) << 2;           // Apply bitstep and offset
    busWrite2B(REG0X12_TERMINATION_CURRENT_LIMIT, milliAmps);
}

// Charge Control
void BQ25756::enableTerminationControl(bool en)
{
    byte addr = REG0X14_PRECHARGE_TERMINATION_CTRL;
    byte mask = (1 << 3);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::enablePrechargeControl(bool en)
{
    byte addr = REG0X14_PRECHARGE_TERMINATION_CTRL;
    byte mask = (1 << 0);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::setPrechargeLimit(VBATLowV value)
{
    byte addr = REG0X14_PRECHARGE_TERMINATION_CTRL;
    byte mask = 0b00000110;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 1)); // Write new value
}

void BQ25756::setRechargeThresh(VRECHG value)
{
    byte addr = REG0X17_CHARGER_CONTROL;
    byte mask = 0b11000000;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 6)); // Write new value
}

// Timers
void BQ25756::setTopOffTimer(TopOffTimer value)
{
    byte addr = REG0X15_TIMER_CONTROL;
    byte mask = 0b11000000;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 6)); // Write new value
}

void BQ25756::setWatchDogTimer(WDTTimer value)
{
    byte addr = REG0X15_TIMER_CONTROL;
    byte mask = 0b00110000;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 4)); // Write new value
}

void BQ25756::enableChargeTimer(bool en)
{
    byte addr = REG0X15_TIMER_CONTROL;
    byte mask = (1 << 3);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::setChargeTimer(CHGTMR value)
{
    byte addr = REG0X15_TIMER_CONTROL;
    byte mask = 0b00000110;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 1)); // Write new value
}

void BQ25756::enableTimer2X(bool en)
{
    byte addr = REG0X15_TIMER_CONTROL;
    byte mask = (1 << 0);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::setCVTimer(byte hours)
{
    byte addr = REG0X16_THREE_STAGE_CHARGE_CTRL;
    byte mask = 0b00001111;
    busWrite1B(addr, (hours & mask)); // Write new value
}

// Special Features
byte BQ25756::whoami()
{
    byte addr = REG0X3D_PART_INFORMATION;
    byte value = (busRead1B(addr) & 0b01111000) >> 3;
    return value;
}

void BQ25756::kickWatchdog()
{
    busWrite1B(REG0X17_CHARGER_CONTROL, (1 << 5));
}

void BQ25756::chargeBehaviorOnWDT(bool en)
{
    byte addr = REG0X17_CHARGER_CONTROL;
    byte mask = (1 << 3);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::highZ(bool en)
{
    byte addr = REG0X17_CHARGER_CONTROL;
    byte mask = (1 << 2);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::chargeEN(bool en)
{
    byte addr = REG0X17_CHARGER_CONTROL;
    byte mask = (1 << 0);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::driveStrength(byte value)
{
    byte addr = REG0X3B_GATE_DRIVER_STRENGTH_CTRL;
    byte mask = 0b11111111;
    busWrite1B(addr, (value & mask)); // Write new value
}

void BQ25756::deadTime(byte value)
{
    byte addr = REG0X3C_GATE_DRIVER_DEAD_TIME_CTRL;
    byte mask = 00001111;
    busWrite1B(addr, (value & mask)); // Write new value
}

// Pin Control
void BQ25756::CEPinFunction(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 0);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::enICHGPin(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 7);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::enILIMHIZpin(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 6);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::disPGPin(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 5);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::disStatPins(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 4);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::forceStat4(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 3);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::forceStat3(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 2);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::forceStat2(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 1);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::forceStat1(bool en)
{
    byte addr = REG0X18_PIN_CONTROL;
    byte mask = (1 << 0);
    safeWriteBool1B(addr, mask, en);
}

// Power Path and Reverse Control
void BQ25756::enIBatLoad(bool en)
// DANGER! Sinks current from SRN to GND.
{
    byte addr = REG0X17_CHARGER_CONTROL;
    byte mask = (1 << 1);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::enIACLoad(bool en)
{
    byte addr = REG0X19_POWER_PATH_REVERSE_MODE_CTRL;
    byte mask = (1 << 6);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::enPFM(bool en)
{
    byte addr = REG0X19_POWER_PATH_REVERSE_MODE_CTRL;
    byte mask = (1 << 5);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::enREV(bool en)
{
    byte addr = REG0X19_POWER_PATH_REVERSE_MODE_CTRL;
    byte mask = (1 << 0);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::setReverseUVP(bool fixed)
{
    byte addr = REG0X1E_REVERSE_UNDERVOLTAGE_CTRL;
    byte mask = (1 << 5);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::setReverseCurrentLimit(REVERSEIBATLIMIT value)
{
    byte addr = REG0X62_REVERSE_MODE_BATTERY_DISCHARGE_CURRENT;
    byte mask = 0b11000000;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 6)); // Write new value
}

// MPPT
void BQ25756::forceSweep(bool en)
{
    byte addr = REG0X1A_MPPT_CONTROL;
    byte mask = (1 << 7);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::setPandOTimer(P_AND_O_TMR value)
{
    byte addr = REG0X62_REVERSE_MODE_BATTERY_DISCHARGE_CURRENT;
    byte mask = 0b01100000;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 5)); // Write new value
}

void BQ25756::setFullSweepTimer(FULL_SWEEP_TMR value)
{
    byte addr = REG0X62_REVERSE_MODE_BATTERY_DISCHARGE_CURRENT;
    byte mask = 0b00000110;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 1)); // Write new value
}

void BQ25756::enMPPT(bool en)
{
    byte addr = REG0X1A_MPPT_CONTROL;
    byte mask = (1 << 0);
    safeWriteBool1B(addr, mask, en);
}

long BQ25756::readMPP()
{
    byte addr = REG0X1F_VAC_MAX_POWER_POINT_DETECT;
    uint16_t value = busRead2B(addr);
    long returnValue = (long)(value >> 2) * (long)20;
    return returnValue;
}

BQ25756::MPPTSTATUS BQ25756::getMPPTStatus()
{
    byte addr = REG0X22_CHARGER_STATUS_2;
    byte value = busRead1B(addr) & 0x02;
    return (MPPTSTATUS)value;
}

// Status
bool BQ25756::ADCBusy()
{
    (busRead1B(REG0X21_CHARGER_STATUS_1) & 0x80) ? return true : return false;
}

bool BQ25756::isInputCurrentRegulated()
{
    (busRead1B(REG0X21_CHARGER_STATUS_1) & 0x40) ? return true : return false;
}

bool BQ25756::isInputVoltageRegulated()
{
    (busRead1B(REG0X21_CHARGER_STATUS_1) & 0x20) ? return true : return false;
}

bool BQ25756::watchDogExpired()
{
    (busRead1B(REG0X21_CHARGER_STATUS_1) & 0x08) ? return true : return false;
}

BQ25756::CHARGESTATE BQ25756::chargeStatus()
{
    byte addr = REG0X21_CHARGER_STATUS_1;
    byte value = busRead1B(addr) & 0x07;
    return (CHARGESTATE)value;
}

BQ25756::TSStatus BQ25756::tempStatus()
{
    byte addr = REG0X22_CHARGER_STATUS_2;
    byte value = (busRead1B(addr) & 70) >> 4;
    return (TSStatus)value;
}

bool BQ25756::inputPG()
{
    (busRead1B(REG0X21_CHARGER_STATUS_1) & 0x08) ? return true : return false;
}

byte BQ25756::FSWSyncStatus()
{
    byte addr = REG0X23_CHARGER_STATUS_3;
    byte value = (busRead1B(addr) & 30) >> 4;
    return value;
}

bool BQ25756::CVtimerExpired()
{
    (busRead1B(REG0X23_CHARGER_STATUS_3) & 0x08) ? return true : return false;
}

bool BQ25756::isReverseMode()
{
    (busRead1B(REG0X23_CHARGER_STATUS_3) & 0x04) ? return true : return false;
}

byte BQ25756::readFault()
{
    busRead1B(REG0X24_FAULT_STATUS);
}

// ADC Control
void BQ25756::enableADC(bool en)
{
    byte addr = REG0X2B_ADC_CONTROL;
    byte mask = (1 << 7);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::enableOneShot(bool en)
{
    byte addr = REG0X2B_ADC_CONTROL;
    byte mask = (1 << 6);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::adcBitDepth(ADC_BIT_DEPTH value)
{
    byte addr = REG0X2B_ADC_CONTROL;
    byte mask = 0b00110000;
    byte value = busRead1B(addr) & (~mask);       // Read register ignore mask
    busWrite1B(addr, value & ((byte)value << 4)); // Write new value
}

void BQ25756::enADCAverageControl(bool en)
{
    byte addr = REG0X2B_ADC_CONTROL;
    byte mask = (1 << 3);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::resetADCRunningAverage(bool newADC)
{
    byte addr = REG0X2B_ADC_CONTROL;
    byte mask = (1 << 2);
    safeWriteBool1B(addr, mask, en);
}

void BQ25756::setADCChannels(byte channelMask)
{
    busWrite1B(REG0X2C_ADC_CHANNEL_CONTROL, channelMask); // Write new value
}

long BQ25756::readADCValueIAC()
// 0.8mA scaling WTF TI
{
    long value = (long)busRead2B(REG0X2D_IAC_ADC);
    value = (value * 4) / 5;
    return value;
}

long BQ25756::readADCValueIBAT()
{
    long value = (long)busRead2B(REG0X2F_IBAT_ADC);
    value = value * 2;
    return value;
}

long BQ25756::readADCValueVAC()
{
    long value = (long)busRead2B(REG0X31_VAC_ADC);
    value = value * 2;
    return value;
}

long BQ25756::readADCValueVBAT()
{
    long value = (long)busRead2B(REG0X33_VBAT_ADC);
    value = value * 2;
    return value;
}

double BQ25756::readADCValueTS()
// TS ADC reading as percentage of REGN
// Reported as unsigned integer
{
    double value = (double)busRead2B(REG0X37_TS_ADC) * (double)0.09765625;
    return value;
}

long BQ25756::readADCValueVFB()
{
    long value = (long)busRead2B(REG0X33_VBAT_ADC);
    return value;
}

// Temperature Control
void BQ25756::enTempSense(bool en)
{
    byte addr = REG0X1C_TS_CHARGING_REGION_BEHAVIOR;
    byte mask = (1 << 0);
    safeWriteBool1B(addr, mask, en);
}

// Interrupt Control
void BQ25756::setInterrupts1(byte value)
{
    busWrite1B(REG0X28_CHARGER_MASK_1, value); // Write new value
}

void BQ25756::setInterrupts2(byte value)
{
    busWrite1B(REG0X29_CHARGER_MASK_2, value); // Write new value
}

void BQ25756::setInterrupts3(byte value)
{
    busWrite1B(REG0X2A_FAULT_MASK, value); // Write new value
}

void BQ25756::safeWriteBool1B(byte addr, byte mask, bool en)
// Helper function that writes a boolean value to a register with mask
{
    byte value = busRead1B(addr);    // Read Reg
    if (((value & mask) != 0) == en) // Check if already set
        return;
    busWrite1B(addr, (value ^ mask)); // Invert bit and write
}

void BQ25756::busWrite1B(byte addr, byte value)
{
    Wire.beginTransmission(BQ25756_I2C_ADDR); // Start I2C transmission
    Wire.write(addr);                         // Write the register address
    Wire.write(value);
    if (Wire.endTransmission() != 0) // End the transmission and check for success
    {
        Serial.println("I2C Write Error!");
    }
}
void BQ25756::busWrite2B(byte addr, uint16_t value)
{
    Wire.beginTransmission(BQ25756_I2C_ADDR); // Start I2C transmission
    Wire.write(addr);                         // Write the register address
    Wire.write((value >> 8) & 0xFF);          // Write the MSB (high byte)
    Wire.write(value & 0xFF);                 // Write the LSB (low byte)
    if (Wire.endTransmission() != 0)          // End the transmission and check for success
    {
        Serial.println("I2C Write Error!");
    }
}
byte BQ25756::busRead1B(byte addr)
{
    uint16_t result = 0;

    Wire.beginTransmission(BQ25756_I2C_ADDR); // Start I2C transmission
    Wire.write(addr);                         // Write the register address
    if (Wire.endTransmission(false) != 0)     // Send the address, but don't release the bus
    {
        Serial.println("I2C Transmission Error!");
        return 0; // Return 0 on failure
    }

    Wire.requestFrom(BQ25756_I2C_ADDR, (uint8_t)1); // Request 2 bytes from the device
    return Wire.read();
}
uint16_t BQ25756::busRead2B(byte addr)
{
    uint16_t result = 0;

    Wire.beginTransmission(BQ25756_I2C_ADDR); // Start I2C transmission
    Wire.write(addr);                         // Write the register address
    if (Wire.endTransmission(false) != 0)     // Send the address, but don't release the bus
    {
        Serial.println("I2C Transmission Error!");
        return 0; // Return 0 on failure
    }

    Wire.requestFrom(BQ25756_I2C_ADDR, (uint8_t)2); // Request 2 bytes from the device
    if (Wire.available() < 2)                       // Check if 2 bytes are available
    {
        Serial.println("I2C Read Error!");
        return 0; // Return 0 on failure
    }

    // Read two bytes (MSB first)
    result = Wire.read() << 8; // Read MSB and shift it into the high byte
    result |= Wire.read();     // Read LSB and combine with MSB

    return result;
}