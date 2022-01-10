#include <cstdint>
#ifndef LETO_IBLDC_H
#define LETO_IBLDC_H

#define CMD_Reset                       0x01
#define CMD_calibrationComplete         0x02
#define CMD_motorIsMoving               0x03
#define CMD_returnCurrentLocation       0x04
#define CMD_goToAbsoluteLocation        0x05
#define CMD_goToRelativeLocation        0x06
#define CMD_travelAtVelocity            0x07
#define CMD_setAccelecration            0x08
#define CMD_goToAbsInSetTime            0x09
#define CMD_goToRelatInSetTime          0x0A
#define CMD_getAcceleration             0x0B
#define CMD_setPID_P_Gain               0x0C
#define CMD_getPID_P_Gain               0x0D
#define CMD_setPID_I_Gain               0x0E
#define CMD_getPID_I_Gain               0x0F
#define CMD_setPID_D_Gain               0x10
#define CMD_getPID_D_Gain               0x11
#define CMD_setFirstEnd2MechEndstop     0x12
#define CMD_setMechMotorRange       0x13
#define CMD_setContinuous           0x19
#define CMD_getContinuous           0x1A
#define CMD_getFirmwareVersion      0x1B
#define CMD_wakeUp                  0x1C
#define CMD_setSleepPowUpMode       0x1D
#define CMD_getEncoderReading       0x1E
#define CMD_saveSetting2Ash         0x23
#define CMD_reloadFactoryDefault    0x24
#define CMD_getSleepPowUpMode       0x2F
#define CMD_getIsSleeping       0x30
#define CMD_gotoRelatPos360       0x40
#define CMD_gotoRelatPosAtSpeed       0x41
#define CMD_gotoAbsolPosAtSpeed       0x42
#define CMD_getSerialFull       0x45
#define CMD_getFirstEndstopDistance       0x4A
#define CMD_getMechMotorRange       0x4B
#define CMD_setUseHallSensor       0x4E
#define CMD_getUseHallSensor       0x4F
#define CMD_setUseTurboMode       0x53
#define CMD_getUseTurboMode       0x54
#define CMD_gotoAbsPosInMs        0x5E
#define CMD_gotoRelatPosInMs        0x5F
#define CMD_setOverTempProtectioni        0x99
#define CMD_getOverTempProtectioni        0x9A
#define CMD_getTemperature        0x9B
#define CMD_getProgramState        0xFE



class LETO_BLDC_Motor {
public:
char name[20] = " _LETO";
uint8_t I2C_addr = 50>>1;
bool isMoving = false;
bool isSleeping = false;
uint16_t currentLocation;
uint16_t travelVelocity;
uint16_t travelAccelection;
uint16_t targetLocation;

void begin();
void resetMotor();
uint16_t getCurrentLocation();
uint16_t LETO_BLDC_Motor::getEncoderReading();
uint16_t setTravelVelocity(uint16_t _travelVelocity);
void goToTargetLocation();
uint8_t getIsMoving();
bool getIsSleeping();
void wakeupMotor();
void getMotorFirmwareVersion();
void getMotorSeiral();
uint8_t isCalibrationComplete();
void gotoAbsoluteLocation(uint16_t _targetLocation);
void gotoRelativeLocation(uint16_t _targetLocation);
void gotoAbsoluteLocationAtSpeed(uint16_t _targetLocation, uint16_t _speed);
void gotoRelativeLocationAtSpeed(uint16_t _targetLocation, uint16_t _speed);
bool finishedHoming();
uint8_t isMotorMoving();

uint16_t getTemperature();
uint8_t getUseHallSensor();
void setUseHallSensor(bool _hallSensor);
void saveSettingsToFlash();

void set_P_Gain(uint16_t _gainData);
uint16_t get_P_Gain();
void set_I_Gain(uint16_t _gainData);
uint16_t get_I_Gain();
void set_D_Gain(uint16_t _gainData);
uint16_t get_D_Gain();

void setFirstEndstop(uint16_t _firstEndStop);
uint16_t getFirstEndstop();

void writeRecommendPID_Data();
void loadFactoryData();

uint8_t getContinuous();
void setContinuous(bool _contiouous);

private:
uint8_t tx_data[4];
uint8_t rx_data[6];

void TitanWrite(uint8_t command,  int n_uint8_ts);
void TitanRead(uint8_t command,  int n_uint8_ts);

};
#endif
