#include <Arduino.h>
#include <LetoBldc.h>
#include <Wire.h>
//
//
void LETO_BLDC_Motor::begin()
{
  Wire.setClock(100000);
  Wire.begin();
}

//
//
void LETO_BLDC_Motor::gotoAbsoluteLocation(uint16_t _targetLocation) {}

//
//
//
//
uint8_t LETO_BLDC_Motor::isMotorMoving()
{
  TitanRead(CMD_motorIsMoving, 1);
  return rx_data[0];
}

//
//
bool LETO_BLDC_Motor::finishedHoming()
{
  rx_data[0] = 0;
  TitanRead(CMD_calibrationComplete, 1);
  if (rx_data[0] == 1)
  {
    Serial.printf("%c Homing Completed\r\n", name[0]);
    return true;
  }
  else if (rx_data[0] == 2)
  {
    Serial.printf("%c Homing failed\r\n", name[0]);
  }
  else
  {
    // Serial.printf("%c Homing status %d\r\n", name[0], rx_data[0]);
  }
  return false;
  delay(100);
}

//
//
void LETO_BLDC_Motor::gotoRelativeLocation(uint16_t _targetLocation)
{
  // Serial.println(_targetLocation);
  tx_data[1] = 0xff & _targetLocation;
  // Serial.println(this->tx_data[0]);
  tx_data[0] = (_targetLocation >> 8);
  // Serial.println(tx_data[1]);
  // Serial.printf("%c, D0:%d, D1:%d, Relative Target: %d\r\n", this->name[0],
  //               this->tx_data[0], this->tx_data[1],
  //               ((this->tx_data[1] << 8) + this->tx_data[0]));
  TitanWrite(CMD_goToRelativeLocation, 2);
}

//
//
uint16_t LETO_BLDC_Motor::getTemperature()
{
  uint16_t tempData = 0;
  TitanRead(CMD_getTemperature, 2);
  tempData = this->rx_data[0] + this->rx_data[1];
  Serial.printf("%c Motor Temp: %d\r\n", this->name[0], tempData);
  return tempData;
}

//
//
uint8_t LETO_BLDC_Motor::isCalibrationComplete()
{
  TitanRead(CMD_calibrationComplete, 1);
  return rx_data[0];
  delay(100);
}

//
//
bool LETO_BLDC_Motor::getIsSleeping()
{
  TitanRead(CMD_getIsSleeping, 1);
  if (this->rx_data[0] == 0)
  {
    isSleeping = false;
    return false;
  }
  else
  {
    isSleeping = true;
    return true;
  }
}
//
//
void LETO_BLDC_Motor::getMotorFirmwareVersion()
{
  TitanRead(CMD_getFirmwareVersion, 4);
  // print received data in human readable form
  Serial.print("// ");
  Serial.print(this->name[0]);
  Serial.print(" FW version: ");
  Serial.print(this->rx_data[0]);
  Serial.print(".");
  Serial.print(this->rx_data[1]);
  Serial.print(".");
  Serial.print((this->rx_data[2] << 8) | this->rx_data[3]);
  Serial.print("\r\n");
}
//
//
void LETO_BLDC_Motor::getMotorSeiral()
{
  TitanRead(CMD_getSerialFull, 4);
  // print received data in human readable form
  Serial.print(this->name[0]);
  Serial.print(" Motor Serial: ");
  Serial.print(this->rx_data[0]);
  Serial.print(".");
  Serial.print(this->rx_data[1]);
  Serial.print(".");
  Serial.print((this->rx_data[2] << 8) | this->rx_data[3]);
  Serial.print("\r\n");
}

//
//
uint8_t LETO_BLDC_Motor::getUseHallSensor()
{
  TitanRead(CMD_getUseHallSensor, 1);
  return rx_data[0];
}

//
//
void LETO_BLDC_Motor::setUseHallSensor(bool _hallSensor)
{
  if (_hallSensor)
    tx_data[0] = 1;
  else
    tx_data[0] = 0;
  TitanWrite(CMD_setUseHallSensor, 1);
}

uint16_t LETO_BLDC_Motor::getCurrentLocation()
{
  uint16_t _returnData = 0;
  TitanRead(CMD_returnCurrentLocation, 2);
  _returnData =
      (((uint16_t)this->rx_data[0]) << 8) + (uint16_t)(this->rx_data[1]);
  return _returnData;
}

uint16_t LETO_BLDC_Motor::setTravelVelocity(uint16_t _travelVelocity)
{
  tx_data[1] = 0xff & _travelVelocity;
  // Serial.println(this->tx_data[0]);
  tx_data[0] = (_travelVelocity >> 8);
  TitanWrite(CMD_travelAtVelocity, 2);
}

//
//
void LETO_BLDC_Motor::gotoAbsoluteLocationAtSpeed(uint16_t _targetLocation,
                                                  uint16_t _speed)
{
  tx_data[0] = (uint8_t)(_targetLocation >> 8);
  tx_data[1] = (uint8_t)(_targetLocation & 0xff);
  tx_data[2] = (uint8_t)(_speed >> 8);
  tx_data[3] = (uint8_t)(_speed & 0xff);
  TitanWrite(CMD_gotoAbsolPosAtSpeed, 4);
}

// CCW direction count: 0xFFFF - target speed
// 1 count/second ~ 0.022 degrees/second
// Max speet at 540 degree/s mapped to 0x5FFF --> 0.022 degrees/s
void LETO_BLDC_Motor::gotoRelativeLocationAtSpeed(uint16_t _targetLocation,
                                                  uint16_t _speed)
{
  tx_data[0] = (uint8_t)(_targetLocation >> 8 & 0xff);
  tx_data[1] = (uint8_t)(_targetLocation & 0xff);
  tx_data[2] = (uint8_t)(_speed >> 8 & 0xff);
  tx_data[3] = (uint8_t)(_speed & 0xff);
  TitanWrite(CMD_gotoRelatPosAtSpeed, 4);
}

//
//
void LETO_BLDC_Motor::goToTargetLocation() {}

//
// Max update rate is 1k/second.
// The 1ms delay can be removed but don't read too fast
uint16_t LETO_BLDC_Motor::getEncoderReading()
{
  uint16_t tempReadings = 0;
  TitanRead(CMD_getEncoderReading, 2);
  delay(1);
  tempReadings = (((uint16_t)rx_data[0]) << 8) + rx_data[1];
  return tempReadings;
}

void LETO_BLDC_Motor::resetMotor()
{
  TitanWrite(CMD_Reset, 0);
  delay(30);
}

void LETO_BLDC_Motor::saveSettingsToFlash()
{
  TitanWrite(CMD_saveSetting2Ash, 0);
  Serial.printf("%c motor save flash memory\r\n", name[0]);
  delay(2100);
}

uint8_t LETO_BLDC_Motor::getIsMoving()
{
  TitanRead(CMD_getIsSleeping, 1);
  return this->rx_data[0];
}

void LETO_BLDC_Motor::wakeupMotor() { TitanWrite(CMD_wakeUp, 0); }

void LETO_BLDC_Motor::TitanWrite(uint8_t command, int n_uint8_ts)
{
  Wire.beginTransmission(I2C_addr);
  Wire.write(command);
  if (n_uint8_ts > 0)
    Wire.write(this->tx_data, n_uint8_ts);
  Wire.endTransmission();
  delayMicroseconds(300);
  // Serial.printf("W CMD: 0x%02X, Add: %d, \tData 0: %d,\tData 1: %d,\tData 2: "
  //               "%d,\tData 3: %d\n",
  //               command, I2C_addr, tx_data[0], tx_data[1], tx_data[2],
  //               tx_data[3]);
}

void LETO_BLDC_Motor::TitanRead(uint8_t command, int n_uint8_ts)
{
  Wire.beginTransmission(this->I2C_addr);
  Wire.write(command);
  Wire.endTransmission();
  uint8_t *_temp = this->rx_data;
  uint32_t __ReadTimeOut = millis() + 100;
  uint8_t _datadata = 0;
  // Serial.printf("R CMD: 0x%02X, Addr: %d ----->\n", command, this->I2C_addr);
  delayMicroseconds(100);
  Wire.requestFrom(this->I2C_addr, n_uint8_ts);
  // Serial.print("Rx Data:");
  while (Wire.available())
  {
    _datadata = Wire.read();
    // Serial.printf("%d,", _datadata);
    *_temp++ = _datadata;
    if (__ReadTimeOut < millis())
    {
      Serial.println("I2C read timeout");
      break;
    }
  }
  // Serial.println(" end");
}

//
//
//
void LETO_BLDC_Motor::set_P_Gain(uint16_t _gainData)
{
  tx_data[0] = (uint8_t)(_gainData >> 8 & 0xff);
  tx_data[1] = (uint8_t)(_gainData & 0xff);
  Serial.printf("Write P Gain: 0x%04X\r\n", _gainData);
  TitanWrite(CMD_setPID_P_Gain, 2);
}

//
//
//
uint16_t LETO_BLDC_Motor::get_P_Gain()
{
  uint16_t tempRecData = 0;
  TitanRead(CMD_getPID_P_Gain, 2);
  tempRecData = ((uint16_t)(rx_data[0])) << 8 + rx_data[1];
  Serial.printf("// %c P Gain:0x%04X\r\n", name[0], tempRecData);
  return tempRecData;
}

//
//
//
void LETO_BLDC_Motor::set_I_Gain(uint16_t _gainData)
{
  tx_data[0] = (uint8_t)(_gainData >> 8 & 0xff);
  tx_data[1] = (uint8_t)(_gainData & 0xff);
  Serial.printf("Write I Gain: 0x%04X\r\n", _gainData);
  TitanWrite(CMD_setPID_I_Gain, 2);
}

//
//
//
uint16_t LETO_BLDC_Motor::get_I_Gain()
{
  uint16_t tempRecData = 0;
  TitanRead(CMD_getPID_I_Gain, 2);
  tempRecData = (uint16_t)(rx_data[0]) << 8 + rx_data[1];
  Serial.printf("// %c I Gain:0x%04X\r\n", name[0], tempRecData);
  return tempRecData;
}
//
//
//
void LETO_BLDC_Motor::set_I_IdleGain(uint16_t _gainData)
{
  tx_data[0] = (uint8_t)(_gainData >> 8 & 0xff);
  tx_data[1] = (uint8_t)(_gainData & 0xff);
  Serial.printf("Write I Idel Gain: 0x%04X\r\n", _gainData);
  TitanWrite(CMD_setPID_I_IdleGain, 2);
}

//
//
//
uint16_t LETO_BLDC_Motor::get_I_IdleGain()
{
  uint16_t tempRecData = 0;
  TitanRead(CMD_getPID_I_IdleGain, 2);
  tempRecData = (uint16_t)(rx_data[0]) << 8 + rx_data[1];
  Serial.printf("// %c I Idel Gain:0x%04X\r\n", name[0], tempRecData);
  return tempRecData;
}

//
//
//
void LETO_BLDC_Motor::set_D_Gain(uint16_t _gainData)
{
  tx_data[0] = (uint8_t)(_gainData >> 8 & 0xff);
  tx_data[1] = (uint8_t)(_gainData & 0xff);
  Serial.printf("Write D Gain: 0x%04X\r\n", _gainData);

  TitanWrite(CMD_setPID_D_Gain, 2);
}

//
//
//
uint16_t LETO_BLDC_Motor::get_D_Gain()
{
  uint16_t tempRecData = 0;
  TitanRead(CMD_getPID_D_Gain, 2);
  tempRecData = (uint16_t)(rx_data[0]) << 8 + rx_data[1];
  Serial.printf("// %c D Gain:0x%04X\r\n", name[0], tempRecData);
  return tempRecData;
}

//
//
void LETO_BLDC_Motor::writeRecommendPID_Data(int _idx)
{

  switch (_idx)
  {
  case 0:
    set_P_Gain(0x03FB);
    // delay(5);
    set_I_Gain(0x5);
    // delay(5);
    set_D_Gain(0x0080);
    // delay(5);
    break;
    /// H axial
  case 1:
    set_P_Gain(0x600);
    // delay(5);
    set_I_Gain(0x10);
    // delay(5);
    set_D_Gain(0x900);
    // delay(5);
    break;
    /// V axial
  case 2:
    set_P_Gain(0x800);
    // delay(5);
    set_I_Gain(0x10);
    // delay(5);
    set_D_Gain(0x1000);
    // delay(5);
    break;

  default:
    set_P_Gain(0x600);
    // delay(5);
    set_I_Gain(0x25);
    // delay(5);
    set_D_Gain(0x1000);
    // delay(5);
    break;
  }

  get_P_Gain();
  get_I_Gain();
  get_D_Gain();

  // saveSettingsToFlash();
  // delay(2100);
}

//
//

void LETO_BLDC_Motor::loadFactoryData()
{
  TitanWrite(CMD_reloadFactoryDefault, 0);
  Serial.println("Factor data loaded");
  delay(2000);
  get_P_Gain();
  get_I_Gain();
  get_D_Gain();
}

//
//
//
uint8_t LETO_BLDC_Motor::getContinuous()
{
  TitanRead(CMD_getContinuous, 2);
  // Serial.printf("%c Motor Continuous: %d,
  // %d\r\n",name[0],rx_data[0],rx_data[1]);
  return rx_data[1];
}

//
//
// True to set to continuous mode
void LETO_BLDC_Motor::setContinuous(bool _isContiouous)
{
  if (_isContiouous)
  {
    tx_data[0] = 0;
    tx_data[1] = 1;
    TitanWrite(CMD_setContinuous, 2);
  }
  else
  {
    tx_data[0] = 0;
    tx_data[1] = 0;
    TitanWrite(CMD_setContinuous, 2);
  }
  Serial.printf("Set continuous mode to: %d\n", _isContiouous);
  delay(2000);
}

//
//
void LETO_BLDC_Motor::setFirstEndstop(uint16_t _firstEndStop)
{
  tx_data[0] = (uint8_t)(_firstEndStop >> 8 & 0xff);
  tx_data[1] = (uint8_t)(_firstEndStop & 0xff);
  TitanWrite(CMD_setFirstEnd2MechEndstop, 2);
  // saveSettingsToFlash();
  // delay(2000);
}

//
//
uint16_t LETO_BLDC_Motor::getFirstEndstop()
{
  TitanRead(CMD_getFirstEndstopDistance, 2);
  uint16_t tempFirstStopData = rx_data[0] << 8 + rx_data[1];
  Serial.printf("%c First endstop: %d\r\n", name[0], tempFirstStopData);
  return (tempFirstStopData);
}


//
// True: power on sleep
void LETO_BLDC_Motor::setSleepOnPowerUpMode(bool _powerOnSleep)
{
  if (_powerOnSleep)
  {
    tx_data[0] = 1;
  }
  else
  {
    tx_data[0] = 0;
  }
  TitanWrite(CMD_setSleepPowUpMode,1);
}

//
// True: power on sleep
// False: power on calibrates immediately
bool LETO_BLDC_Motor::getSleepOnPowerUpMode()
{
  TitanRead(CMD_getSleepPowUpMode, 1);
  if(rx_data[0] == 0)
  {
    return false;
  }
  else
  {
    return true;
  }  
}
