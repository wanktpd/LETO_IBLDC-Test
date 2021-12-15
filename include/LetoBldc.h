#include <cstdint>
#ifndef LETO_IBLDC_H
#define LETO_IBLDC_H
class LETO_BLDC_Motor {
public:
uint8_t I2C_addr = 50;
bool isMoving = false;
bool isSleeping = false;
uint16_t currentLocation;
uint16_t travelVelocity;
uint16_t travelAccelection;
uint16_t targetLocation;

void begin();
uint16_t getCurrentLocation();
uint16_t setTravelVelocity(uint16_t _travelVelocity);
void goToTargetLocation();


private:
uint8_t tx_data[4];
uint8_t rx_data[6];

void TitanWrite(uint8_t command, uint8_t *tx_data, int n_uint8_ts);
void TitanRead(uint8_t command, uint8_t *rx_data, int n_uint8_ts);
};
#endif
