#include <Arduino.h>
#include <LetoBldc.h>
#include <Wire.h>
//
//
void LETO_BLDC_Motor::begin() { Wire.begin(); }

//
//
void LETO_BLDC_Motor::gotoAbsoluteLocation(uint16_t _targetLocation) {}

//
//
bool LETO_BLDC_Motor::finishedHoming() {
  rx_data[0] = 0;
  TitanRead(CMD_calibrationComplete,1);
  if (rx_data[0] == 1)
  {
    Serial.printf("%c Homing Completed\r\n", name[0]);
    return true;
  }
  else if(rx_data[0] == 2)
  {
    Serial.printf("%c Homing failed\r\n", name[0]);
  }
  else
  {
    Serial.printf("%c Homing status %d\r\n", name[0],rx_data[0]);
  }
  return false;
  delay(100);
}

//
//
void LETO_BLDC_Motor::gotoRelativeLocation(uint16_t _targetLocation) {
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
uint16_t LETO_BLDC_Motor::getTemperature() {
  uint16_t tempData = 0;
  TitanRead(CMD_getTemperature, 2);
  tempData = this->rx_data[0] + this->rx_data[1];
  Serial.printf("%c Motor Temp: %d\r\n", this->name[0], tempData);
  return tempData;
}

//
//
uint8_t LETO_BLDC_Motor::isCalibrationComplete() {
  TitanRead(CMD_calibrationComplete, 1);
  return rx_data[0];
  delay(100);
}

//
//
bool LETO_BLDC_Motor::getIsSleeping() {
  TitanRead(CMD_getIsSleeping, 1);
  if (this->rx_data[0] == 0)
    return false;
  else
    return true;
}
//
//
void LETO_BLDC_Motor::getMotorFirmwareVersion() {
  TitanRead(CMD_getFirmwareVersion, 4);
  // print received data in human readable form
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
void LETO_BLDC_Motor::getMotorSeiral() {
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

uint16_t LETO_BLDC_Motor::getCurrentLocation() {
  uint16_t _returnData = 0;
  TitanRead(CMD_returnCurrentLocation, 2);
  _returnData =
      (((uint16_t)this->rx_data[0]) << 8) + (uint16_t)(this->rx_data[1]);
  return _returnData;
}

uint16_t LETO_BLDC_Motor::setTravelVelocity(uint16_t _travelVelocity) {}
void LETO_BLDC_Motor::goToTargetLocation() {}

void LETO_BLDC_Motor::resetMotor() {
  TitanWrite(CMD_Reset, 0);
  delay(3000);
}

uint8_t LETO_BLDC_Motor::getIsMoving() {
  TitanRead(CMD_getIsSleeping, 1);
  return this->rx_data[0];
}

void LETO_BLDC_Motor::wakeupMotor() { TitanWrite(CMD_wakeUp, 0); }

void LETO_BLDC_Motor::TitanWrite(uint8_t command, int n_uint8_ts) {
  Wire.beginTransmission(I2C_addr);
  Wire.write(command);
  if (n_uint8_ts > 0)
    Wire.write(this->tx_data, n_uint8_ts);
  Wire.endTransmission();
}

void LETO_BLDC_Motor::TitanRead(uint8_t command, int n_uint8_ts) {
  Wire.beginTransmission(this->I2C_addr);
  Wire.write(command);
  Wire.endTransmission();
  uint8_t *_temp = this->rx_data;
  uint32_t __ReadTimeOut = millis() + 100;
  uint8_t _datadata = 0;

  // Serial.printf("Command: %d, Addr: %d\r\n", command, this->I2C_addr);

  Wire.requestFrom(this->I2C_addr, n_uint8_ts);
  while (Wire.available()) {
    _datadata = Wire.read();
    Serial.printf("Rx Data: %d\r\n", _datadata);
    *_temp++ = _datadata;
    if (__ReadTimeOut < millis()) {
      Serial.println("I2C read timeout");
      break;
    }
  }
}
