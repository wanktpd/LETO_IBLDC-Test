#include <Arduino.h>
#include <LetoBldc.h>
#include <Wire.h>

#define LENGTH 240
// #define LENGTH 230

#define targetSpeed 158
#define ScanRange 273 // 1.5 degree rotation
#define AccelectrationRange 200
#define DecelectrationRange 200
#define ScanStartPos_H 6000
#define ScanStartPos_V 300

#define V_Encoder_Ref
#define H_Encoder_Ref
#define PID_init 0

bool Write2FlashMemory_H = false;
bool Write2FlashMemory_V = false;

bool processSerialCommand = false;

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

void TitanWrite(byte address, byte command, byte *tx_data, int n_bytes);
void TitanRead(int address, byte command, byte *rx_data, int n_bytes);

void processSerialCMD();
bool checkI2CBus();
char receivedSerialCMD[10];
uint8_t serialBitCounter = 0;

bool startScanTest = false;

void setup()
{
  Serial.begin(9600);
  hAxialMotor.I2C_addr = 0x50 >> 1;
  vAxialMotor.I2C_addr = 0x52 >> 1;

  hAxialMotor.name[0] = 'H';
  vAxialMotor.name[0] = 'V';

  vAxialMotor.begin();
  delay(5000);

  if (PID_init)
  {
    Serial.println("Set PID Initial value");
    vAxialMotor.get_P_Gain();
    vAxialMotor.get_I_Gain();
    vAxialMotor.get_D_Gain();

    vAxialMotor.writeRecommendPID_Data(1);
    vAxialMotor.get_P_Gain();
    vAxialMotor.get_I_Gain();
    vAxialMotor.get_D_Gain();
    vAxialMotor.setFirstEndstop(100);
    vAxialMotor.setMechanicalRange(5100);
    vAxialMotor.getMechanicalRange();

    vAxialMotor.saveSettingsToFlash();

    delay(5000);

    hAxialMotor.get_P_Gain();
    hAxialMotor.get_I_Gain();
    hAxialMotor.get_D_Gain();

    hAxialMotor.writeRecommendPID_Data(0);
    hAxialMotor.get_P_Gain();
    hAxialMotor.get_I_Gain();
    hAxialMotor.get_D_Gain();
    hAxialMotor.setFirstEndstop(100);
    hAxialMotor.setMechanicalRange(7500);
    hAxialMotor.getMechanicalRange();

    hAxialMotor.saveSettingsToFlash();
    delay(5000);

    uint32_t eventTimer1 = millis() + 3000;

    while (1)
    {
      if (eventTimer1 < millis())
      {
        eventTimer1 = millis() + 10000;

        Serial.println("PID initial firmware");
        Serial.println("Change the firmware setting to run the application firmware");

        vAxialMotor.get_P_Gain();
        vAxialMotor.get_I_Gain();
        vAxialMotor.get_D_Gain();

        hAxialMotor.get_P_Gain();
        hAxialMotor.get_I_Gain();
        hAxialMotor.get_D_Gain();

        vAxialMotor.getFirstEndstop();
        hAxialMotor.getFirstEndstop();

        vAxialMotor.getMechanicalRange();
        hAxialMotor.getMechanicalRange();
      }
      processSerialCMD();
    }
  }

  // Serial.printf("// Pre Encoder Reading: v: %d, h: %d\n",
  //               vAxialMotor.getEncoderReading(),
  //               hAxialMotor.getEncoderReading());

  //**************************************************************
  // Check Hall Sensor Status
  //**************************************************************

  // hAxialMotor.setUseHallSensor(false);
  if (hAxialMotor.getUseHallSensor() == 0)
  {
    Serial.println("// H motor is not using Hall sensor");
    // hAxialMotor.setUseHallSensor(true);
    // Serial.println("// H motor is set to using Hall sensor");
    // hAxialMotor.saveSettingsToFlash();
  }
  else
  {
    Serial.println("// H motor is using Hall sensor");
    hAxialMotor.setUseHallSensor(false);
    Serial.println("// H motor is set to not using Hall sensor");
    Write2FlashMemory_H = true;
  }
  // vAxialMotor.loadFactoryData();
  // hAxialMotor.loadFactoryData();

  if (vAxialMotor.getUseHallSensor() == 0)
  {
    Serial.println("// V motor is not using Hall sensor");
    // vAxialMotor.setUseHallSensor(true);
    // Serial.println("// V motor is set to using Hall sensor");
    // vAxialMotor.saveSettingsToFlash();
  }
  else
  {
    Serial.println("// V motor is using Hall sensor");
    vAxialMotor.setUseHallSensor(false);
    Serial.println("// V motor set to not using Hall sensor");
    Write2FlashMemory_V = true;
  }

  //**************************************************************
  // Check first soft end stop
  //**************************************************************

  Serial.printf("V motor first end stop %d\r\n", vAxialMotor.getFirstEndstop());
  Serial.printf("H motor first end stop %d\r\n", hAxialMotor.getFirstEndstop());

  //**************************************************************
  // Check rotation mode
  // Continuous mode ---> 0x001
  // Limited mode   --->  0x000
  //**************************************************************
  if (hAxialMotor.getContinuous() == 0)
  {
    //hAxialMotor.setContinuous(true);
    Serial.printf("// H motor is in limited mode\r\n");
    // hAxialMotor.setContinuous(true);
    // Write2FlashMemory_H = true;
    // Serial.printf("// H motor is set to continuous mode\r\n");
  }
  else
  {
    Serial.printf("// H motor is in coutinuous mode\r\n");
    hAxialMotor.setContinuous(false);
    Write2FlashMemory_H = true;
    Serial.printf("// H motor is set to limited mode\r\n");
  }

  if (vAxialMotor.getContinuous() == 0)
  {
    Serial.printf("// V motor is in limited mode\r\n");
    // vAxialMotor.setContinuous(true);
    // Write2FlashMemory_V = true;
    // Serial.printf("// V motor is set to continuous mode\r\n");
  }
  else
  {
    Serial.printf("// V motor is in coutinuous mode\r\n");
    vAxialMotor.setContinuous(false);
    Write2FlashMemory_V = true;
    Serial.printf("// V motor is set to limited mode\r\n");
  }

  // Serial.println("// Wakeup Motor");
  // hAxialMotor.wakeupMotor();
  // delay(100);
  // vAxialMotor.wakeupMotor();
  // delay(100);

  //**************************************************************
  // Check Sleep on power on mode
  // Sleep mode ---> 0x001
  // No Sleep mode   --->  0x000
  //**************************************************************
  if (vAxialMotor.getSleepOnPowerUpMode())
  {
    vAxialMotor.setSleepOnPowerUpMode(false);
    Write2FlashMemory_V = true;
  }
  else
  {
    Serial.println("// V motor power on calibrate immediate");
  }
  if (hAxialMotor.getSleepOnPowerUpMode())
  {
    hAxialMotor.setSleepOnPowerUpMode(false);
    Write2FlashMemory_H = true;
  }
  else
  {
    Serial.println("// H motor power on calibrate immediate");
  }

  if (Write2FlashMemory_H || PID_init)
  {
    hAxialMotor.setFirstEndstop(165);
    hAxialMotor.saveSettingsToFlash();
  }
  if (Write2FlashMemory_V || PID_init)
  {
    vAxialMotor.setFirstEndstop(165);
    vAxialMotor.saveSettingsToFlash();
  }

  //***************************************************

  hAxialMotor.resetMotor();
  vAxialMotor.resetMotor();

  delay(1000);
  bool waitHoming = false;
  while (!waitHoming)
  {
    waitHoming = vAxialMotor.finishedHoming() && hAxialMotor.finishedHoming();
    delay(100);
    // Serial.printf("v: %d, h: %d\r\n", vAxialMotor.getEncoderReading(),
    //               hAxialMotor.getEncoderReading());
  }

  delay(2000);
  Serial.printf("// Encoder Reading: v: %d, h: %d\tv target: \r\n",
                vAxialMotor.getEncoderReading(),
                hAxialMotor.getEncoderReading(),
                vAxialMotor.getCurrentLocation());

  // hAxialMotor.setFirstEndstop(1000);
  // vAxialMotor.setFirstEndstop(1000);

  // hAxialMotor.getFirstEndstop();
  // vAxialMotor.getFirstEndstop();

  // hAxialMotor.loadFactoryData();
  // vAxialMotor.writeRecommendPID_Data(2);
  vAxialMotor.get_P_Gain();
  vAxialMotor.get_I_Gain();
  // vAxialMotor.get_I_IdleGain();
  vAxialMotor.get_D_Gain();

  // hAxialMotor.writeRecommendPID_Data(1);
  hAxialMotor.get_P_Gain();
  hAxialMotor.get_I_Gain();
  // hAxialMotor.set_I_IdleGain(1234);
  // hAxialMotor.get_I_IdleGain();
  hAxialMotor.get_D_Gain();

  // while (1)
  // {
  //   TitanRead(vAxialMotor.I2C_addr, 0x0f, rx_data, 6);
  //   Serial.print("I_gain:");
  //   Serial.printf("0x%04X\r\n",rx_data[0] << 8 + rx_data[1]);
  //   delay(1000);
  // }

  // hAxialMotor.gotoAbsoluteLocationAtSpeed(1000, targetSpeed);
  // vAxialMotor.gotoAbsoluteLocationAtSpeed(500, targetSpeed);
  // hAxialMotor.gotoAbsoluteLocationAtSpeed(500, targetSpeed);

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

  vAxialMotor.gotoAbsoluteLocationAtSpeed(ScanStartPos_V, targetSpeed);
  while (vAxialMotor.isMotorMoving() != 0)
  {
    // Serial.printf("V: %d\r\n", vAxialMotor.getEncoderReading());
  }
  hAxialMotor.gotoAbsoluteLocationAtSpeed(ScanStartPos_H, targetSpeed);
  while (hAxialMotor.isMotorMoving() != 0)
  {
    // Serial.printf("H: %d\r\n", hAxialMotor.getEncoderReading());
  }
  Serial.println("Ready for test, send \"$R\" command to start test");

  // while (1) {
  //   hAxialMotor.gotoAbsoluteLocationAtSpeed(100, targetSpeed);
  //   Serial.println(millis());
  //   delay(1000);
  // }
}

void loop()
{
  processSerialCMD();
  if (scanCounter < 50 && startScanTest)
  {
    // Serial.printf("// Encoder Reading: v: %d, h: %d\n",
    //               vAxialMotor.getEncoderReading(),
    //               hAxialMotor.getEncoderReading());
    scanCounter++;
    if (scanCounter == 1)
    {
      hAxialMotor.getMotorFirmwareVersion();
      vAxialMotor.getMotorFirmwareVersion();

      vAxialMotor.get_P_Gain();
      vAxialMotor.get_I_Gain();
      vAxialMotor.get_D_Gain();

      hAxialMotor.get_P_Gain();
      hAxialMotor.get_I_Gain();
      hAxialMotor.get_D_Gain();

      Serial.printf(
          "// Motor Speed Setup(count/second. 65536 counts per revolution): %d\r\n",
          targetSpeed);
      Serial.println("--------------------- Motor test data logging start from here ---------------------");
      Serial.printf(
          "// Direction&MotorType\tScanCounter\tEncoderReading\tTime(ms)\r\n");
    }

    checkI2CBus();
    delay(1000);
    mainGlobalTimer1 = millis() + 15000;

    vAxialMotor.gotoAbsoluteLocationAtSpeed(
        AccelectrationRange + ScanRange + DecelectrationRange + ScanStartPos_V, targetSpeed);
    // Serial.printf("Motor running status at start: %d\n",
    //               vAxialMotor.isMotorMoving());
    tempCooridinate = vAxialMotor.getEncoderReading();
    tempTimer1 = millis() + 200;
    delay(200);
    while (vAxialMotor.isMotorMoving() != 0)
    {
      //Forward, V=1
      Serial.printf("1, %d, %d, %d\r\n", scanCounter,
                    vAxialMotor.getEncoderReading() - tempCooridinate,
                    millis() - tempTimer1);
      if (millis() > mainGlobalTimer1)
      {
        Serial.println(
            "//F Motor Moving Status time out Error ---------------------");
        Serial.printf("//Motor is Running: %d\n", vAxialMotor.isMotorMoving());

        break;
      }
      // delay(2);
    }
    delay(1000);
    checkI2CBus();

    mainGlobalTimer1 = millis() + 15000;

    vAxialMotor.gotoAbsoluteLocationAtSpeed(ScanStartPos_V, 0xFFFF - targetSpeed);
    delay(100);
    // Serial.printf("Motor running status in between: %d\n",
    //               vAxialMotor.isMotorMoving());
    tempCooridinate = vAxialMotor.getEncoderReading();
    tempTimer1 = millis() + 200;
    delay(200);

    while (vAxialMotor.isMotorMoving() != 0)
    {
      // delay(2);
      //Return, V=2
      Serial.printf("2, %d, %d, %d\r\n", scanCounter,
                    tempCooridinate - vAxialMotor.getEncoderReading(),
                    millis() - tempTimer1);

      if (millis() > mainGlobalTimer1)
      {
        Serial.println(
            "//R Motor Moving Status time out Error ---------------------");
        Serial.printf("//Motor is Running: %d\n", vAxialMotor.isMotorMoving());
        break;
      }
    }
    // Serial.printf("Motor running status return: %d\n",
    //               vAxialMotor.isMotorMoving());

    delay(1000);
    checkI2CBus();
    mainGlobalTimer1 = millis() + 15000;
    // Serial.printf("Sleeping: %d\n", vAxialMotor.getIsSleeping());

    hAxialMotor.gotoAbsoluteLocationAtSpeed(
        AccelectrationRange + ScanRange + DecelectrationRange + ScanStartPos_H, targetSpeed);
    // Serial.printf("Motor running status at start: %d\n",
    //               hAxialMotor.isMotorMoving());

    tempCooridinate = hAxialMotor.getEncoderReading();
    tempTimer1 = millis() + 200;
    delay(200);
    while (hAxialMotor.isMotorMoving() != 0)
    {
      //Forward, H = 3
      Serial.printf("3, %d, %d, %d\r\n", scanCounter,
                    hAxialMotor.getEncoderReading() - tempCooridinate,
                    millis() - tempTimer1);
      if (millis() > mainGlobalTimer1)
      {
        Serial.println(
            "//F Motor Moving Status time out Error ---------------------");
        Serial.printf("//Motor is Running: %d\n", hAxialMotor.isMotorMoving());

        break;
      }
      // delay(2);
    }

    delay(1000);
    checkI2CBus();
    mainGlobalTimer1 = millis() + 15000;
    hAxialMotor.gotoAbsoluteLocationAtSpeed(ScanStartPos_H, 0xFFFF - targetSpeed);
    delay(100);
    // Serial.printf("Motor running status in between: %d\n",
    //               hAxialMotor.isMotorMoving());
    tempCooridinate = hAxialMotor.getEncoderReading();
    tempTimer1 = millis() + 200;
    delay(200);
    while (hAxialMotor.isMotorMoving() != 0)
    {
      // delay(2);
      //Return, H=4
      Serial.printf("4, %d, %d, %d\r\n", scanCounter,
                    tempCooridinate - hAxialMotor.getEncoderReading(),
                    millis() - tempTimer1);

      if (millis() > mainGlobalTimer1)
      {
        Serial.println(
            "//R Motor Moving Status time out Error ---------------------");
        Serial.printf("Motor is Running: %d\n", hAxialMotor.isMotorMoving());
        break;
      }
    }
    // Serial.printf("Motor running status return: %d\n",
    //               hAxialMotor.isMotorMoving());
    Serial.printf("// << ----- Scan %d ended ----->>\r\n", scanCounter);

    // delay(1000);
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

void TitanWrite(byte address, byte command, byte *tx_data, int n_bytes)
{
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(tx_data, n_bytes);
  Wire.endTransmission();
}

void TitanRead(int address, byte command, byte *rx_data, int n_bytes)
{
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();

  Wire.requestFrom(address, n_bytes);
  while (Wire.available())
  {
    *rx_data++ = Wire.read();
  }
}

//
// Serial command need to start from "$" followed by motor type "V" or "P" upper case only to change PID
// "$S" Stop motor Run
// "$C" Clear test counter
// "$R" Start test Run
// "$W" Wrire data to flash memory W followed by motor type
// "$GVP" Get motor Gain
// "RSV" reset V motor
// "EH", "EV" Get encoder readings
// "GAHxxx", Go to abs location
// "GRHxxx", Go to releative location

void processSerialCMD()
{
  while (Serial.available())
  {
    char tempRead = Serial.read();
    // Serial.print(tempRead);
    if (tempRead == '$')
    {
      serialBitCounter = 0;
      tempRead = Serial.read();
    }
    if (tempRead != '\r' && tempRead != '\n')
    {
      receivedSerialCMD[serialBitCounter++] = tempRead;
      if (serialBitCounter > 9)
      {
        serialBitCounter = 0;
      }
      // Serial.println(receivedSerialCMD);
    }
    else if (tempRead == '\r')
    {
      receivedSerialCMD[serialBitCounter] = '\0';
      // Serial.printf("New Command: %s\r\n", receivedSerialCMD);
      if (receivedSerialCMD[0] == 'P')
      {
        uint16_t newPGain = atoi(receivedSerialCMD + 2);
        Serial.printf("P Command: %s, data: %d\r\n", receivedSerialCMD, newPGain);
        if (receivedSerialCMD[1] == 'V')
        {
          vAxialMotor.set_P_Gain(newPGain);
          vAxialMotor.get_P_Gain();
        }
        if (receivedSerialCMD[1] == 'H')
        {
          hAxialMotor.set_P_Gain(newPGain);
          hAxialMotor.get_P_Gain();
        }
      }

      if (receivedSerialCMD[0] == 'D')
      {
        uint16_t newPGain = atoi(receivedSerialCMD + 2);
        if (receivedSerialCMD[1] == 'V')
        {
          vAxialMotor.set_D_Gain(newPGain);
          vAxialMotor.get_D_Gain();
        }
        if (receivedSerialCMD[1] == 'H')
        {
          hAxialMotor.set_D_Gain(newPGain);
          hAxialMotor.get_D_Gain();
        }
      }

      if (receivedSerialCMD[0] == 'I')
      {
        uint16_t newPGain = atoi(receivedSerialCMD + 2);
        if (receivedSerialCMD[1] == 'V')
        {
          vAxialMotor.set_I_Gain(newPGain);
          vAxialMotor.get_I_Gain();
        }
        if (receivedSerialCMD[1] == 'H')
        {
          hAxialMotor.set_I_Gain(newPGain);
          hAxialMotor.get_I_Gain();
        }
      }

      if (receivedSerialCMD[0] == 'S')
      {
        startScanTest = false;
      }

      if (receivedSerialCMD[0] == 'R')
      {
        if (receivedSerialCMD[1] == 'S')
        {
          if (receivedSerialCMD[2] == 'V')
          {
            vAxialMotor.resetMotor();
          }
          else if (receivedSerialCMD[2] == 'H')
          {
            hAxialMotor.resetMotor();
          }
          else if (receivedSerialCMD[2] == 'T')
          {
            // reset Teensy controller
            SCB_AIRCR = 0x05FA0004;
            // NVIC_SystemReset();
          }
        }
        else
        {
          startScanTest = true;
        }
      }

      if (receivedSerialCMD[0] == 'C')
      {
        scanCounter = 0;
      }

      if (receivedSerialCMD[0] == 'G')
      {
        if (receivedSerialCMD[1] == 'H')
        {
          if (receivedSerialCMD[2] == 'P')
          {
            hAxialMotor.get_P_Gain();
          }
          if (receivedSerialCMD[2] == 'I')
          {
            hAxialMotor.get_I_Gain();
          }
          if (receivedSerialCMD[2] == 'D')
          {
            hAxialMotor.get_D_Gain();
          }
        }

        if (receivedSerialCMD[1] == 'V')
        {
          if (receivedSerialCMD[2] == 'P')
          {
            vAxialMotor.get_P_Gain();
          }
          if (receivedSerialCMD[2] == 'I')
          {
            vAxialMotor.get_I_Gain();
          }
          if (receivedSerialCMD[2] == 'D')
          {
            vAxialMotor.get_D_Gain();
          }
        }
      }

      if (receivedSerialCMD[0] == 'W')
      {
        if (receivedSerialCMD[1] == 'V')
        {
          vAxialMotor.saveSettingsToFlash();
        }
        if (receivedSerialCMD[1] == 'H')
        {
          hAxialMotor.saveSettingsToFlash();
        }
      }

      if (receivedSerialCMD[0] == 'E')
      {
        if (receivedSerialCMD[1] == 'V')
        {
          Serial.printf("V Encoder: %d\r\n", vAxialMotor.getEncoderReading());
        }
        if (receivedSerialCMD[1] == 'H')
        {
          Serial.printf("H Encoder: %d\r\n", hAxialMotor.getEncoderReading());
        }
      }

      if (receivedSerialCMD[0] == 'G')
      {
        uint16_t newLocation = atoi(receivedSerialCMD + 3);
        if (receivedSerialCMD[1] == 'A')
        {
          if (receivedSerialCMD[2] == 'H')
          {
            hAxialMotor.gotoAbsoluteLocationAtSpeed(newLocation, targetSpeed);
          }
          if (receivedSerialCMD[2] == 'V')
          {
            vAxialMotor.gotoAbsoluteLocationAtSpeed(newLocation, targetSpeed);
          }
        }
        else if (receivedSerialCMD[1] == 'R')
        {
          if (receivedSerialCMD[2] == 'H')
          {
            hAxialMotor.gotoRelativeLocationAtSpeed(newLocation, targetSpeed);
          }
          if (receivedSerialCMD[2] == 'V')
          {
            vAxialMotor.gotoRelativeLocationAtSpeed(newLocation, targetSpeed);
          }
        }
      }
    }
  }
}

bool checkI2CBus()
{
  vAxialMotor.ClearRxData();
  if (vAxialMotor.getEncoderReading())
  {
    return true;
  }
  else
  {
    Wire.end();
    Serial.println("// I2C bus is not responding, the I2C controller has been reset.");
    delay(100);
    Wire.begin();
    delay(100);
    return false;
  }
}