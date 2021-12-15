#include <LetoBldc.h>
#include <Arduino.h>
#include <Wire.h>

void LETO_BLDC_Motor::begin()
{
    Wire.begin(this->I2C_addr);
}
uint16_t LETO_BLDC_Motor::getCurrentLocation()
{

}
uint16_t LETO_BLDC_Motor::setTravelVelocity(uint16_t _travelVelocity)
{

}
void LETO_BLDC_Motor::goToTargetLocation()
{

}

void LETO_BLDC_Motor::TitanWrite( uint8_t command, uint8_t *tx_data, int n_uint8_ts) {
  Wire.beginTransmission(this->I2C_addr);
  Wire.write(command);
  Wire.write(tx_data, n_uint8_ts);
  Wire.endTransmission();
}

void LETO_BLDC_Motor::TitanRead( uint8_t command, uint8_t *rx_data, int n_uint8_ts) {
  Wire.beginTransmission(this->I2C_addr);
  Wire.write(command);
  Wire.endTransmission();

  Wire.requestFrom(this->I2C_addr, n_uint8_ts);
  while (Wire.available()) {
    *rx_data++ = Wire.read();
  }
}