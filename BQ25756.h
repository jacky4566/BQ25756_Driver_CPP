#ifndef BQ25756_DRIVER
#define BQ25756_DRIVER

#include <Arduino.h>
#include <Wire.h>

class BQ25756
{
public:
    // Constructor to initialize the device with the I2C address
    BQ25756(uint8_t deviceAddress) : deviceAddress(deviceAddress) {}

    // Initializes the I2C interface (must be called in setup())
    int begin()
    {
        Wire.begin();
        return 0;
    }

    // Set Volts and Amps
    void setFBmVLimit(uint16_t milliVolts);
    void setChargeALimit(uint16_t milliAmps);
    void setInputALimit(uint16_t milliAmps);
    void setInputVLimit(uint16_t milliVolts); // Note if EN_MPPT = 1, the Full Sweep method will use this limit as the lower search window for Full Panel Sweep
    void setReverseModeALimit(uint16_t milliAmps);
    void setReverseModeVLimit(uint16_t milliVolts);
    void setPrechargeALimit(uint16_t milliAmps);
    void setTerminationALimit(uint16_t milliAmps);

    // Charge Control
    void enableTerminationControl(bool en); // Default on
    void enablePrechargeControl(bool en);   // Default on
    void setPrechargeLimit(VBATLowV value); // Battery threshold for PRECHG to FASTCHG transition, as percentage of VFB_REG
    void setRechargeThresh(VRECHG value);

    // Timers
    void setTopOffTimer(TopOffTimer value);
    void setWatchDogTimer(WDTTimer value);
    void enableChargeTimer(bool en); // Default on
    void setChargeTimer(CHGTMR value);
    void enableTimer2X(bool en); // Default on
    void setCVTimer(byte hours);

    // Special Features
    byte whoami();                     // Should return 2
    void kickWatchdog();               // Must be called faster than BQ Watchdog (40S) REG0x17
    void chargeBehaviorOnWDT(bool en); // Controls the EN_CHG bit behavior when WATCHDOG expires
    void highZ(bool en);               // Set converter to HIZ mode
    void chargeEN(bool en);            // Charge Enable Control, Default ENs
    void driveStrength(byte value);    // Write whole register 0x3B
    void deadTime(byte value);         // Write whole register 0x3C

    // Pin Control
    void CEPinFunction(bool en); // Default Enabled, Inverted logic
    void enICHGPin(bool en);     // Default Enabled
    void enILIMHIZpin(bool en);  // Default Enabled
    void disPGPin(bool en);      // Default Disabled
    void disStatPins(bool en);   // Default Disabled
    void forceStat4(bool en);    // Default Disabled
    void forceStat3(bool en);    // Default Disabled
    void forceStat2(bool en);    // Default Disabled
    void forceStat1(bool en);    // Default Disabled

    // Power Path and Reverse Control
    void enIBatLoad(bool en);       // Sinks current from SRN to GND! DANGER!!!
    void enIACLoad(bool en);        // Default Disabled
    void enPFM(bool en);            // Default Disabled
    void enREV(bool en);            // Default Disabled
    void setReverseUVP(bool fixed); // Default to 80% of VSYS_REV target, set 1 to fix at 3.3V
    void setReverseCurrentLimit(REVERSEIBATLIMIT value);

    // MPPT
    void forceSweep(bool en); // Force mppt sweep
    void setPandOTimer(P_AND_O_TMR value);
    void setFullSweepTimer(FULL_SWEEP_TMR value); // Full panel scan appears to take 12S? Need confirmation
    void enMPPT(bool en);                         // Default Disabled, When MPPT is enabled, the ADC is controlled by the device, writes to REG2B (datasheet error here) are ignored
    long readMPP();                               // Read target Max Power Point detected
    enum MPPTSTATUS getMPPTStatus();

    // Status
    bool ADCBusy(); // For one-shot mode only
    bool isInputCurrentRegulated();
    bool isInputVoltageRegulated();
    bool watchDogExpired();
    enum CHARGESTATE chargeStatus();
    enum TSStatus tempStatus();
    bool inputPG();
    byte FSWSyncStatus();
    bool CVtimerExpired();
    bool isReverseMode();
    byte readFault(); // Returns entire fault register

    // ADC Control
    // When MPPT is enabled, the ADC is controlled by the device, writes to REG2B (datasheet error here) are ignored
    void enableADC(bool en);                  // Default = (Disabled & EN_VREG_TEMP_COMP)
    void enableOneShot(bool en);              // Default Enabled
    void adcBitDepth(ADC_BIT_DEPTH value);    // Default 2 (13 Bit)
    void enADCAverageControl(bool en);        // Enable running average, Default Disabled
    void resetADCRunningAverage(bool newADC); // Reset running average, set 1 to use a new ADC or false to use existiing value
    void setADCChannels(byte channelMask);    // Set channels for ADC to read
    long readADCValueIAC();
    long readADCValueIBAT(); // Recommend to disable IBAT ADC channel when EN_IBAT_LOAD bit is 1
    long readADCValueVAC();
    long readADCValueVBAT();
    double readADCValueTS();
    long readADCValueVFB(); // Recommend to disable this channel when charging is enabled

    // Temperature Control
    // TODO: Reg x1C and x1D
    void enTempSense(bool en); // Default Enabled

    // Interrupt Control
    void setInterrupts1(byte value); // Use flags1
    void setInterrupts2(byte value); // Use flags2
    void setInterrupts3(byte value); // Use flags3
    //  TODO: Read Charger Flags 0x25 0x26 0x27

private:
    void safeWriteBool1B(byte addr, byte mask, bool en);
    void busWrite1B(byte addr, byte value);
    void busWrite2B(byte addr, uint16_t value);
    byte busRead1B(byte addr);
    uint16_t busRead2B(byte addr);
};

#endif // BQ25756_DRIVER