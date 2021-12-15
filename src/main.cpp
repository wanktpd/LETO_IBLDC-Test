#include <Arduino.h>
#include <Wire.h>

//#define LENGTH 240
#define LENGTH 230

byte tx_data[4];
byte rx_data[6];
int dev_H_addr = 0x50 >> 1; // JP1 in zero position => 7 bit address of 0x28
int dev_V_addr = 0x52 >> 1; // JP1 in zero position => 7 bit address of 0x28
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

void TitanWrite(byte address, byte command, byte *tx_data, int n_bytes);
void TitanRead(int address, byte command, byte *rx_data, int n_bytes);

void setup() {

  // pinMode(toggle_pin,INPUT);
  Wire.begin();
  Serial.begin(115200);
  Serial.print("Hello World\r\n");

  // wait for motor startup sequence to finish
  delay(600);

  Serial.print("Resetting motor...\r\n");
  // send the reset command (0x01)
  TitanWrite(dev_H_addr, 0x01, tx_data, 0);
  TitanWrite(dev_V_addr, 0x01, tx_data, 0);

  delay(3000);

  // read firmware version (cmd: 0x1b, 4 bytes to receive)
  TitanRead(dev_V_addr, 0x1b, rx_data, 4);

  // print received data in human readable form
  Serial.print("Motor V FW version: ");
  Serial.print(rx_data[0]);
  Serial.print(".");
  Serial.print(rx_data[1]);
  Serial.print(".");
  Serial.print((rx_data[2] << 8) | rx_data[3]);
  Serial.print("\r\n");

  // read firmware version (cmd: 0x1b, 4 bytes to receive)
  TitanRead(dev_H_addr, 0x1b, rx_data, 4);

  // print received data in human readable form
  Serial.print("Motor H FW version: ");
  Serial.print(rx_data[0]);
  Serial.print(".");
  Serial.print(rx_data[1]);
  Serial.print(".");
  Serial.print((rx_data[2] << 8) | rx_data[3]);
  Serial.print("\r\n");

  // is the motor sleeping? (cmd 0x30)
  TitanRead(dev_H_addr, 0x1c, rx_data, 1);
  if (rx_data[0] == 0x01) {
    // send the wake up command (0x1c)
    TitanWrite(dev_H_addr, 0x1c, tx_data, 0);
  }

  // wait for homing/calibration to complete
  bool finished_homing = false;
  while (!finished_homing) {
    TitanRead(dev_H_addr, 0x02, rx_data, 1);
    if (rx_data[0] == 0x01) {
      Serial.print("Homing complete \r\n");
      finished_homing = true;
    } else if (rx_data[0] == 0x02) {
      Serial.print("Homing failed!\r\n");
      return;
    }
    delay(100);
  }

  /*
  tx_data[0] = 0x0c;
  tx_data[1] = 0xcc;
  TitanWrite(dev_H_addr, 0x07, tx_data, 2);  */

  tx_data[0] = 0x01;
  tx_data[1] = 0x00;
  TitanWrite(dev_H_addr, 0x07, tx_data, 2);

  // delay(2500);
  delay(2500);

  Serial.print("Test starting:\r\n");
  // pin_read = digitalRead(toggle_pin);

  while (1) {
    while (index_pos < LENGTH) {
      while (micros() - start_micros < 1000) {
        ;
      }
      start_micros = micros();

      TitanRead(dev_H_addr, 0x2b, rx_data, 6);
      // TitanRead(dev_H_addr, 0x1e, rx_data, 2);
      // encoder_array[index] = ((unsigned long)rx_data[0]<<24)|((unsigned
      // long)rx_data[1]<<16)|((unsigned long)rx_data[2]<<8)|(unsigned
      // long)rx_data[3];
      sin_array[index_pos] =
          ((unsigned int)rx_data[0] << 8) | ((unsigned int)rx_data[1]);
      cos_array[index_pos] =
          ((unsigned int)rx_data[2] << 8) | ((unsigned int)rx_data[3]);
      sector_array[index_pos] =
          ((unsigned int)rx_data[4] << 8) | ((unsigned int)rx_data[5]);
      if (sector_array[index_pos] >= 2196) {
        index_pos = 0;
        break;
      }
      index_pos++;
    }

    final_sector = sector_array[LENGTH - 1];

    /*reverse velocity*/
    tx_data[0] = 0xf0;
    tx_data[1] = 0x00;
    TitanWrite(dev_H_addr, 0x07, tx_data, 2);

    for (index_pos = 0; index_pos < LENGTH; index_pos++) {
      if (sector_array[index_pos] == final_sector) {
        // get out of loop if on last sector since it won't be a whole sector of
        // data
        break;
      }
      Serial.print(sin_array[index_pos]);
      Serial.print("\t");
      Serial.print(cos_array[index_pos]);
      Serial.print("\t");
      Serial.print(sector_array[index_pos]);
      Serial.print("\r\n");
      // delay(50);
      delay(1);
    }
    index_pos = 0;
    delay(250);

    /*after outputting, forward velocity again*/
    tx_data[0] = 0x04;
    tx_data[1] = 0x00;
    TitanWrite(dev_H_addr, 0x07, tx_data, 2);

    while (current_sector != final_sector) {
      while (micros() - start_micros < 1000) {
        ;
      }
      start_micros = micros();

      TitanRead(dev_H_addr, 0x2b, rx_data, 6);
      current_sector =
          ((unsigned int)rx_data[4] << 8) | ((unsigned int)rx_data[5]);
    }
  }

  /*
    current_sector = 0xffff;

    while(current_sector < initial_sector || wrapped == 0)
    {
      start_micros = micros();
      while(index < LENGTH)
      {
        while(micros()-start_micros < 1000)
        {
          ;
        }
        start_micros = micros();

        TitanRead(dev_H_addr, 0x2b, rx_data, 6);
        //TitanRead(dev_H_addr, 0x1e, rx_data, 2);
        //encoder_array[index] = ((unsigned long)rx_data[0]<<24)|((unsigned
    long)rx_data[1]<<16)|((unsigned long)rx_data[2]<<8)|(unsigned
    long)rx_data[3]; sin_array[index] = ((unsigned int)rx_data[0]<<8)|((unsigned
    int)rx_data[1]); cos_array[index] = ((unsigned int)rx_data[2]<<8)|((unsigned
    int)rx_data[3]); sector_array[index] = ((unsigned
    int)rx_data[4]<<8)|((unsigned int)rx_data[5]); if(index == 0 && test_started
    == 0)
        {
          test_started = 1;
          initial_sector = sector_array[index];
        }
        index++;
      }

      for(index = 0;index < LENGTH;index++)
      {
        Serial.print(sin_array[index]);
        Serial.print("\t");
        Serial.print(cos_array[index]);
        Serial.print("\t");
        Serial.print(sector_array[index]);
        Serial.print("\r\n");
        delay(50);

      }
      if(wrap_check == 0 && sector_array[0] > sector_array[LENGTH-1])
      {
        wrap_check = 1;
        wrapped = 1;
      }
      final_sector = sector_array[LENGTH-1];
      current_sector = 0xffff;
      while(current_sector != final_sector)
      {
        while(micros()-start_micros < 1000)
        {
          ;
        }
        start_micros = micros();

        TitanRead(dev_H_addr, 0x2b, rx_data, 6);
        current_sector = ((unsigned int)rx_data[4]<<8)|((unsigned
    int)rx_data[5]);
      }
      Serial.print("\r\n");
      index = 0;
    }


    TitanWrite(dev_H_addr, 0x01, tx_data, 0);  */
}

void loop() {
  /*
  TitanRead(dev_H_addr, 0x60, rx_data, 4);
  Serial.print(micros());
  Serial.print("\t");
  Serial.print(((unsigned long)rx_data[0]<<24)|((unsigned
  long)rx_data[1]<<16)|((unsigned long)rx_data[2]<<8)|(unsigned
  long)rx_data[3]); Serial.print("\r\n"); delay(500);*/

  /*
  if(pin_read == digitalRead(toggle_pin))
  {
    ;
  }
  else
  {
    pin_read = digitalRead(toggle_pin);
    start_millis = millis();
    TitanRead(dev_H_addr, 0x81, rx_data, 2);
    if(millis() - start_millis > 500)
    {
      //i2c message took too long
      Serial.print("I2C fallen over\r\n");
      while(1)
      {
        ;
      }
    }
    index++;
    if(index > 1000)
    {
      index = 0;
      Serial.print(".");
      index_loop_count ++;
      if(index_loop_count > 60)
      {
        index_loop_count = 0;
        minute_count ++;
        Serial.print("\tminutes = ");
        Serial.print(minute_count);
        Serial.print("\r\n");
      }
    }

  }

    */
}

void TitanWrite(byte address, byte command, byte *tx_data, int n_bytes) {
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(tx_data, n_bytes);
  Wire.endTransmission();
}

void TitanRead(int address, byte command, byte *rx_data, int n_bytes) {
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();

  Wire.requestFrom(address, n_bytes);
  while (Wire.available()) {
    *rx_data++ = Wire.read();
  }
}