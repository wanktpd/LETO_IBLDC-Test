#include <Arduino.h>
#include <LetoBldc.h>

#define LENGTH 240
// #define LENGTH 230

byte tx_data[4];
byte rx_data[6];
int dev_V_addr = 0x52 >> 1; // JP1 in zero position => 7 bit address of 0x28
int dev_H_addr = 0x50 >> 1; // JP1 in zero position => 7 bit address of 0x28
// long start_time = 0;
// unsigned int encoder = 0;
// unsigned long encoder_array[LENGTH];
// unsigned int encoder_array[LENGTH];
// unsigned int demand_array[LENGTH];
// unsigned long time_array[LENGTH];
// unsigned long start_millis = 0;
unsigned long start_micros = 0;
unsigned int sin_array[LENGTH];
unsigned int cos_array[LENGTH];
unsigned int sector_array[LENGTH];
unsigned int final_sector = 0;
unsigned int current_sector = 0;
unsigned int initial_sector = 0xfffe;
byte test_started = 0;
byte wrapped = 0;
byte wrap_check = 0;
unsigned int reverse_start_millis = 0;
unsigned int reverse_end_millis = 0;

unsigned int index_pos = 0;
// unsigned int index_loop_count = 0;
// unsigned int minute_count = 0;
// int toggle_pin = 13;
// byte pin_read = LOW;
LETO_BLDC_Motor hAxialMotor;
LETO_BLDC_Motor vAxialMotor;

uint32_t mainGlobalTimer1 = millis();

uint16_t startLocation_H;
uint16_t startLocation_V;

void setup() {
  hAxialMotor.I2C_addr = 0x50 >> 1;
  vAxialMotor.I2C_addr = 0x52 >> 1;

  hAxialMotor.name[0] = 'H';
  vAxialMotor.name[0] = 'V';

  hAxialMotor.begin();
  delay(600);
  Serial.println("Wakeup Motor");
  hAxialMotor.wakeupMotor();
  delay(100);
  vAxialMotor.wakeupMotor();

  hAxialMotor.getMotorFirmwareVersion();
  vAxialMotor.getMotorFirmwareVersion();

  hAxialMotor.resetMotor();
  // mainGlobalTimer1 = millis() + 10000;

  while (hAxialMotor.finishedHoming()==false)
    delay(300);

  mainGlobalTimer1 = millis() + 10000;
  vAxialMotor.resetMotor();
  while (vAxialMotor.finishedHoming()==false)
    delay(300);

  startLocation_H = hAxialMotor.getCurrentLocation();
  startLocation_V = vAxialMotor.getCurrentLocation();
}

void loop() {
  if (mainGlobalTimer1 < millis()) {
    mainGlobalTimer1 = millis() + 6000;
    Serial.printf("H location: %d\r\n", hAxialMotor.getCurrentLocation());
    Serial.printf("V location: %d\r\n", vAxialMotor.getCurrentLocation());

    hAxialMotor.gotoRelativeLocation(startLocation_V + 500);
    vAxialMotor.gotoRelativeLocation(startLocation_H + 300);
    hAxialMotor.getTemperature();
    hAxialMotor.finishedHoming();
    delay(3000);

    Serial.printf("H location 1: %d\r\n", hAxialMotor.getCurrentLocation());
    Serial.printf("V location 1: %d\r\n", vAxialMotor.getCurrentLocation());

    hAxialMotor.gotoRelativeLocation(startLocation_V - 500);
    vAxialMotor.gotoRelativeLocation(startLocation_H - 300);
    vAxialMotor.getTemperature();
    vAxialMotor.finishedHoming();
    delay(3000);
  }
}