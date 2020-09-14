/*
  xdrv_34_wemos_motor_v1.ino - Support for I2C WEMOS motor shield (6612FNG)

  Copyright (C) 2020  Denis Sborets and Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_I2C
#ifdef USE_WEMOS_MOTOR_V1
/*********************************************************************************************\
 * WEMOS_MOTOR_V1 - DC motor driver shield (6612FNG) v 1.0.0
 *
 * I2C Address: 0x30
 *
 * Command format:
 * driver44 <command>,<motor>,<direction>{,<duty>}
 * command:
 *    RESET - reset a motor shield
 *    SETMOTOR - seter motor state
 * motor:
 *    0 - motor A
 *    1 - motor B
 * direction:
 *    0 - short break
 *    1 - CCW
 *    2 - CW
 *    3 - stop
 *    4 - standby
 * duty (optional):
 *    0 - 100% (100% by default)
\*********************************************************************************************/

#define XDRV_34              34
#define XI2C_44              44          // See I2CDEVICES.md

#ifndef WEMOS_MOTOR_V1_ADDR
#define WEMOS_MOTOR_V1_ADDR  0x30        // Default I2C address 0x30
#endif
#ifndef WEMOS_MOTOR_V1_FREQ
#define WEMOS_MOTOR_V1_FREQ  1000        // Default frequency
#endif

#define MOTOR_A 0
#define MOTOR_B 1

#define SHORT_BRAKE 0
#define CCW 1
#define CW 2
#define STOP 3
#define STANDBY 4

struct WMOTORV1 {
  bool detected = false;
  uint8_t motor;
} WMotorV1;

void WMotorV1Detect(void)
{
  if (I2cSetDevice(WEMOS_MOTOR_V1_ADDR)) {
    WMotorV1.detected = true;
    I2cSetActiveFound(WEMOS_MOTOR_V1_ADDR, "WEMOS_MOTOR_V1");
    WMotorV1Reset();
  }
}

void WMotorV1Reset(void)
{
//  Wire.begin();
  WMotorV1SetFrequency(WEMOS_MOTOR_V1_FREQ);
}

void WMotorV1SetFrequency(uint32_t freq)
{
  Wire.beginTransmission(WEMOS_MOTOR_V1_ADDR);
  Wire.write(((byte)(freq >> 16)) & (byte)0x0f);
  Wire.write((byte)(freq >> 16));
  Wire.write((byte)(freq >> 8));
  Wire.write((byte)freq);
  Wire.endTransmission(); // stop transmitting
//  delay(100);
}

void WMotorV1SetMotor(uint8_t motor, uint8_t dir, float pwm_val)
{
  Wire.beginTransmission(WEMOS_MOTOR_V1_ADDR);
  Wire.write(motor | (byte)0x10);
  Wire.write(dir);

  uint16_t _pwm_val = uint16_t(pwm_val * 100);
  if (_pwm_val > 10000) {
    _pwm_val = 10000;
  }

  Wire.write((byte)(_pwm_val >> 8));
  Wire.write((byte)_pwm_val);
  Wire.endTransmission();
//  delay(100);
}

bool WMotorV1Command(void)
{
  uint8_t args_count = 0;

  if (XdrvMailbox.data_len > 0) {
    args_count = 1;
  } else {
    return false;
  }

  for (uint32_t idx = 0; idx < XdrvMailbox.data_len; idx++) {
    if (' ' == XdrvMailbox.data[idx]) {
      XdrvMailbox.data[idx] = ',';
    }
    if (',' == XdrvMailbox.data[idx]) {
      args_count++;
    }
  }
  UpperCase(XdrvMailbox.data, XdrvMailbox.data);

  char *command = strtok(XdrvMailbox.data, ",");

  if (strcmp(command, "RESET") == 0) {
    WMotorV1Reset();
    Response_P(PSTR("{\"WEMOS_MOTOR_V1\":{\"RESET\":\"OK\"}}"));
    return true;
  }

  if (strcmp(command, "SETMOTOR") == 0) {
    if (args_count >= 3) {

      int motor = atoi(strtok(NULL, ","));
      int dir = atoi(strtok(NULL, ","));
      int duty = 100;
      if (args_count == 4) {
        duty = atoi(strtok(NULL, ","));
      }

      WMotorV1SetMotor(motor, dir, duty);
      Response_P(PSTR("{\"WEMOS_MOTOR_V1\":{\"SETMOTOR\":\"OK\"}}"));
      return true;
    }
  }
  return false;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv34(uint8_t function)
{
  if (!I2cEnabled(XI2C_44)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    WMotorV1Detect();
  }
  else if (WMotorV1.detected) {
    switch (function) {
      case FUNC_COMMAND_DRIVER:
        if (XI2C_44 == XdrvMailbox.index) {
          result = WMotorV1Command();
        }
        break;
    }
  }
  return result;
}

#endif // USE_WEMOS_MOTOR_V1
#endif // USE_IC2
