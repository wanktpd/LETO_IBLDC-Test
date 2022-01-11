#include <Arduino.h>
#include <LetoBldc.h>
#include <Wire.h>

#define LENGTH 240
// #define LENGTH 230

#define targetSpeed 158
#define ScanRange 273 // 1.5 degree rotation
#define AccelectrationRange 300
#define DecelectrationRange 200

#define V_Encoder_Ref
#define H_Encoder_Ref

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

int scanCounter = 0;
uint32_t tempTimer1 = 0;
uint16_t tempCooridinate = 0;

void setup() {
  hAxialMotor.I2C_addr = 0x50 >> 1;
  vAxialMotor.I2C_addr = 0x52 >> 1;

  hAxialMotor.name[0] = 'H';
  vAxialMotor.name[0] = 'V';

  vAxialMotor.begin();
  delay(5000);

  Serial.printf("// Pre Encoder Reading: v: %d, h: %d\n",
                vAxialMotor.getEncoderReading(),
                hAxialMotor.getEncoderReading());

  // hAxialMotor.setUseHallSensor(false);
  if (hAxialMotor.getUseHallSensor() == 0) {
    Serial.println("// H motor is not using Hall sensor");
    // hAxialMotor.setUseHallSensor(true);
    // Serial.println("// H motor is set to using Hall sensor");
    // hAxialMotor.saveSettingsToFlash();

  } else {
    Serial.println("// H motor is using Hall sensor");
    hAxialMotor.setUseHallSensor(false);
    Serial.println("// H motor is set to not using Hall sensor");
    hAxialMotor.saveSettingsToFlash();
  }
  // vAxialMotor.loadFactoryData();
  // hAxialMotor.loadFactoryData();

  if (vAxialMotor.getUseHallSensor() == 0) {
    // Serial.println("// V motor is not using Hall sensor");
    // vAxialMotor.setUseHallSensor(true);
    // Serial.println("// V motor is set to using Hall sensor");
    // vAxialMotor.saveSettingsToFlash();

  } else {
    Serial.println("// V motor is using Hall sensor");
    vAxialMotor.setUseHallSensor(false);
    Serial.println("// V motor set to not using Hall sensor");
    vAxialMotor.saveSettingsToFlash();
  }

  if (hAxialMotor.getContinuous() == 0) {
    // hAxialMotor.setContinuous(true);
    // Serial.printf("// H motor is in continuous mode\n");
  } else {
    hAxialMotor.setContinuous(false);
    Serial.printf("// H motor is in limited mode\n");
  }
  if (vAxialMotor.getContinuous() == 0) {
    // vAxialMotor.setContinuous(true);
    // Serial.printf("// V motor is in coutinuous mode\n");
  } else {
    vAxialMotor.setContinuous(false);
    Serial.printf("// V motor is in limited mode\n");
  }

  hAxialMotor.getMotorFirmwareVersion();
  vAxialMotor.getMotorFirmwareVersion();

  Serial.println("// Wakeup Motor");
  hAxialMotor.wakeupMotor();
  delay(100);
  vAxialMotor.wakeupMotor();
  delay(100);

  hAxialMotor.resetMotor();
  vAxialMotor.resetMotor();

  bool waitHoming = false;
  while (!waitHoming) {
    waitHoming = vAxialMotor.finishedHoming() && hAxialMotor.finishedHoming();
    // Serial.printf("v: %d, h: %d\n", vAxialMotor.getEncoderReading(),
    //               hAxialMotor.getEncoderReading());
  }

  delay(2000);
  Serial.printf("// Encoder Reading: v: %d, h: %d\r\n",
                vAxialMotor.getEncoderReading(),
                hAxialMotor.getEncoderReading());

  // hAxialMotor.setFirstEndstop(1000);
  // vAxialMotor.setFirstEndstop(1000);

  // hAxialMotor.getFirstEndstop();
  // vAxialMotor.getFirstEndstop();

  // hAxialMotor.loadFactoryData();
  // vAxialMotor.writeRecommendPID_Data(2);
  vAxialMotor.get_P_Gain();
  vAxialMotor.get_I_Gain();
  vAxialMotor.get_I_IdleGain();
  vAxialMotor.get_D_Gain();

  // hAxialMotor.writeRecommendPID_Data(1);
  hAxialMotor.get_P_Gain();
  hAxialMotor.get_I_Gain();
  // hAxialMotor.set_I_IdleGain(1234);
  hAxialMotor.get_I_IdleGain();
  hAxialMotor.get_D_Gain();

  // hAxialMotor.gotoRelativeLocationAtSpeed(1000, targetSpeed);
  vAxialMotor.gotoRelativeLocationAtSpeed(1500, targetSpeed);

  // Serial.println("power cycle motor ");
  // delay(10000);

  // // hAxialMotor.saveSettingsToFlash();
  // hAxialMotor.resetMotor();
  // // // mainGlobalTimer1 = millis() + 10000;

  // while (hAxialMotor.finishedHoming() == false)
  //   delay(300);

  // mainGlobalTimer1 = millis() + 10000;
  // vAxialMotor.resetMotor();
  // while (vAxialMotor.finishedHoming() == false)
  //   delay(300);

  // startLocation_H = hAxialMotor.getEncoderReading();
  // startLocation_V = vAxialMotor.getCurrentLocation();
  // hAxialMotor.gotoRelativeLocation(startLocation_V + 500);
  // vAxialMotor.gotoRelativeLocation(startLocation_V + 500);

  // vAxialMotor.writeRecommendPID_Data();
  // hAxialMotor.writeRecommendPID_Data();


  vAxialMotor.gotoRelativeLocationAtSpeed(100, targetSpeed);
  while (vAxialMotor.isMotorMoving() != 0) {
    Serial.printf("V: %d\r\n", vAxialMotor.getEncoderReading());
  }
  hAxialMotor.gotoRelativeLocationAtSpeed(100, targetSpeed);
  while (hAxialMotor.isMotorMoving() != 0) {
    Serial.printf("H: %d\r\n", hAxialMotor.getEncoderReading());
  }
  delay(5000);
  Serial.printf(
      "// Motor Speed Setup(count/second. 65536 counts per revolution): %d\r\n",
      targetSpeed);
  Serial.printf(
      "// Direction\tMotor\tScanCounter\tEncoderReading\tTime(ms)\r\n");


  // while (1) {
  //   hAxialMotor.gotoRelativeLocationAtSpeed(100, targetSpeed);
  //   Serial.println(millis());
  //   delay(1000);
  // }
}

void loop() {

  if (scanCounter < 50) {
    Serial.printf("// Encoder Reading: v: %d, h: %d\n",
                  vAxialMotor.getEncoderReading(),
                  hAxialMotor.getEncoderReading());
    scanCounter++;
    vAxialMotor.gotoRelativeLocationAtSpeed(
        AccelectrationRange + ScanRange + DecelectrationRange, targetSpeed);
    mainGlobalTimer1 = millis() + 10000;
    // Serial.printf("Motor running status at start: %d\n",
    //               vAxialMotor.isMotorMoving());
    tempCooridinate = vAxialMotor.getEncoderReading();
    tempTimer1 = millis();
    delay(200);
    while (vAxialMotor.isMotorMoving() != 0) {
      Serial.printf("Forward, V, %d, %d, %d\r\n", scanCounter,
                    vAxialMotor.getEncoderReading() - tempCooridinate,
                    millis() - tempTimer1);
      if (millis() > mainGlobalTimer1) {
        Serial.println(
            "//F Motor Moving Status time out Error ---------------------");
        Serial.printf("//Motor is Running: %d\n", vAxialMotor.isMotorMoving());

        break;
      }
      // delay(2);
    }
    delay(1000);
    mainGlobalTimer1 = millis() + 5000;

    vAxialMotor.gotoRelativeLocationAtSpeed(10, 0xFFFF - targetSpeed);
    delay(100);
    // Serial.printf("Motor running status in between: %d\n",
    //               vAxialMotor.isMotorMoving());
    tempCooridinate = vAxialMotor.getEncoderReading();
    tempTimer1 = millis();
    delay(200);

    while (vAxialMotor.isMotorMoving() != 0) {
      // delay(2);
      Serial.printf("Return, V, %d, %d, %d\r\n", scanCounter,
                    tempCooridinate - vAxialMotor.getEncoderReading(),
                    millis() - tempTimer1);

      if (millis() > mainGlobalTimer1) {
        Serial.println(
            "//R Motor Moving Status time out Error ---------------------");
        Serial.printf("//Motor is Running: %d\n", vAxialMotor.isMotorMoving());
        break;
      }
    }
    // Serial.printf("Motor running status return: %d\n",
    //               vAxialMotor.isMotorMoving());

    delay(1000);
    // Serial.printf("Sleeping: %d\n", vAxialMotor.getIsSleeping());

    hAxialMotor.gotoRelativeLocationAtSpeed(
        AccelectrationRange + ScanRange + DecelectrationRange, targetSpeed);
    mainGlobalTimer1 = millis() + 5000;
    // Serial.printf("Motor running status at start: %d\n",
    //               hAxialMotor.isMotorMoving());

    tempCooridinate = hAxialMotor.getEncoderReading();
    tempTimer1 = millis();
    delay(200);
    while (hAxialMotor.isMotorMoving() != 0) {

      Serial.printf("Forward, H, %d, %d, %d\r\n", scanCounter,
                    hAxialMotor.getEncoderReading() - tempCooridinate,
                    millis() - tempTimer1);
      if (millis() > mainGlobalTimer1) {
        Serial.println(
            "//F Motor Moving Status time out Error ---------------------");
        Serial.printf("//Motor is Running: %d\n", hAxialMotor.isMotorMoving());

        break;
      }
      // delay(2);
    }
    delay(1000);
    mainGlobalTimer1 = millis() + 5000;

    hAxialMotor.gotoRelativeLocationAtSpeed(10, 0xFFFF - targetSpeed);
    delay(100);
    // Serial.printf("Motor running status in between: %d\n",
    //               hAxialMotor.isMotorMoving());
    tempCooridinate = hAxialMotor.getEncoderReading();
    tempTimer1 = millis();
    delay(200);
    while (hAxialMotor.isMotorMoving() != 0) {
      // delay(2);
      Serial.printf("Return, H, %d, %d, %d\r\n", scanCounter,
                    tempCooridinate - hAxialMotor.getEncoderReading(),
                    millis() - tempTimer1);

      if (millis() > mainGlobalTimer1) {
        Serial.println(
            "//R Motor Moving Status time out Error ---------------------");
        Serial.printf("Motor is Running: %d\n", hAxialMotor.isMotorMoving());
        break;
      }
    }
    // Serial.printf("Motor running status return: %d\n",
    //               hAxialMotor.isMotorMoving());
    Serial.printf("// << ----- Scan %d ended ----->>\r\n", scanCounter);

    delay(1000);
    // Serial.printf("Sleeping: %d\n", hAxialMotor.getIsSleeping());

    // if (mainGlobalTimer1 < millis()) {
    //   mainGlobalTimer1 = millis() + 6000;
    //   Serial.printf("H location: %d\r\n", hAxialMotor.getEncoderReading());
    //   Serial.printf("V location: %d\r\n", vAxialMotor.getEncoderReading());

    //   hAxialMotor.gotoRelativeLocation(startLocation_V + 500);
    //   vAxialMotor.gotoRelativeLocation(startLocation_H + 800);
    //   hAxialMotor.getTemperature();
    //   hAxialMotor.finishedHoming();
    //          hAxialMotor.isMotorMoving() == 0xff) {
    //     Serial.printf("H: %d, V: %d\r\n", hAxialMotor.getEncoderReading(),
    //                   vAxialMotor.getEncoderReading());
    //   }

    //   Serial.printf("H location 1: %d\r\n", hAxialMotor.getEncoderReading());
    //   Serial.printf("V location 1: %d\r\n", vAxialMotor.getEncoderReading());

    //   hAxialMotor.gotoRelativeLocation(startLocation_V - 500);
    //   vAxialMotor.gotoRelativeLocation(startLocation_H - 800);
    //   vAxialMotor.getTemperature();
    //   vAxialMotor.finishedHoming();
    //   while (vAxialMotor.isMotorMoving() == 1 ||
    //          vAxialMotor.isMotorMoving() == 0xff) {
    //     Serial.printf("H: %d, V: %d\r\n", hAxialMotor.getEncoderReading(),
    //                   vAxialMotor.getEncoderReading());
    //   }

    //   vAxialMotor.getTemperature();
    //   hAxialMotor.getTemperature();
    // }
  }
}