/*
  support_button.ino - button support for Tasmota

  Copyright (C) 2020  Theo Arends

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

#define BUTTON_V1
#ifdef BUTTON_V1
/*********************************************************************************************\
 * Button support
\*********************************************************************************************/

#define MAX_BUTTON_COMMANDS  5  // Max number of button commands supported
const char kCommands[] PROGMEM =
  D_CMND_WIFICONFIG " 2|" D_CMND_WIFICONFIG " 2|" D_CMND_WIFICONFIG " 2|" D_CMND_RESTART " 1|" D_CMND_UPGRADE " 1";

struct BUTTON {
  unsigned long debounce = 0;                // Button debounce timer
  uint16_t hold_timer[MAX_KEYS] = { 0 };     // Timer for button hold
  uint16_t dual_code = 0;                    // Sonoff dual received code

  uint8_t last_state[MAX_KEYS] = { NOT_PRESSED, NOT_PRESSED, NOT_PRESSED, NOT_PRESSED };  // Last button states
  uint8_t window_timer[MAX_KEYS] = { 0 };    // Max time between button presses to record press count
  uint8_t press_counter[MAX_KEYS] = { 0 };   // Number of button presses within Button.window_timer

  uint8_t dual_receive_count = 0;            // Sonoff dual input flag
  uint8_t no_pullup_mask = 0;                // key no pullup flag (1 = no pullup)
  uint8_t inverted_mask = 0;                 // Key inverted flag (1 = inverted)
  uint8_t present = 0;                       // Number of buttons found flag
  uint8_t adc = 99;                          // ADC0 button number
} Button;

/********************************************************************************************/

void ButtonPullupFlag(uint8 button_bit)
{
  bitSet(Button.no_pullup_mask, button_bit);
}

void ButtonInvertFlag(uint8 button_bit)
{
  bitSet(Button.inverted_mask, button_bit);
}

void ButtonInit(void)
{
  Button.present = 0;
  for (uint32_t i = 0; i < MAX_KEYS; i++) {
    if (pin[GPIO_KEY1 +i] < 99) {
      Button.present++;
      pinMode(pin[GPIO_KEY1 +i], bitRead(Button.no_pullup_mask, i) ? INPUT : ((16 == pin[GPIO_KEY1 +i]) ? INPUT_PULLDOWN_16 : INPUT_PULLUP));
    }
#ifndef USE_ADC_VCC
    else if ((99 == Button.adc) && ((ADC0_BUTTON == my_adc0) || (ADC0_BUTTON_INV == my_adc0))) {
      Button.present++;
      Button.adc = i;
    }
#endif  // USE_ADC_VCC
  }
}

uint8_t ButtonSerial(uint8_t serial_in_byte)
{
  if (Button.dual_receive_count) {
    Button.dual_receive_count--;
    if (Button.dual_receive_count) {
      Button.dual_code = (Button.dual_code << 8) | serial_in_byte;
      serial_in_byte = 0;
    } else {
      if (serial_in_byte != 0xA1) {
        Button.dual_code = 0;                // 0xA1 - End of Sonoff dual button code
      }
    }
  }
  if (0xA0 == serial_in_byte) {              // 0xA0 - Start of Sonoff dual button code
    serial_in_byte = 0;
    Button.dual_code = 0;
    Button.dual_receive_count = 3;
  }

  return serial_in_byte;
}

/*********************************************************************************************\
 * Button handler with single press only or multi-press and hold on all buttons
 *
 * ButtonDebounce (50) - Debounce time in mSec
 * SetOption1 (0)      - If set do not execute config commands
 * SetOption11 (0)     - If set perform single press action on double press and reverse
 * SetOption13 (0)     - If set act on single press only
 * SetOption32 (40)    - Max button hold time in Seconds
 * SetOption40 (0)     - Number of 0.1 seconds until hold is discarded if SetOption1 1 and SetOption13 0
\*********************************************************************************************/

void ButtonHandler(void)
{
  if (uptime < 4) { return; }                                   // Block GPIO for 4 seconds after poweron to workaround Wemos D1 / Obi RTS circuit

  uint8_t hold_time_extent = IMMINENT_RESET_FACTOR;             // Extent hold time factor in case of iminnent Reset command
  uint16_t loops_per_second = 1000 / Settings.button_debounce;  // ButtonDebounce (50)
  char scmnd[20];

//  uint8_t maxdev = (devices_present > MAX_KEYS) ? MAX_KEYS : devices_present;
//  for (uint32_t button_index = 0; button_index < maxdev; button_index++) {
  for (uint32_t button_index = 0; button_index < MAX_KEYS; button_index++) {
    uint8_t button = NOT_PRESSED;
    uint8_t button_present = 0;

    if (!button_index && ((SONOFF_DUAL == my_module_type) || (CH4 == my_module_type))) {
      button_present = 1;
      if (Button.dual_code) {
        AddLog_P2(LOG_LEVEL_DEBUG, PSTR(D_LOG_APPLICATION D_BUTTON " " D_CODE " %04X"), Button.dual_code);
        button = PRESSED;
        if (0xF500 == Button.dual_code) {                      // Button hold
          Button.hold_timer[button_index] = (loops_per_second * Settings.param[P_HOLD_TIME] / 10) -1;  // SetOption32 (40)
          hold_time_extent = 1;
        }
        Button.dual_code = 0;
      }
    }
    else if (pin[GPIO_KEY1 +button_index] < 99) {
      button_present = 1;
      button = (digitalRead(pin[GPIO_KEY1 +button_index]) != bitRead(Button.inverted_mask, button_index));
    }
#ifndef USE_ADC_VCC
    if (Button.adc == button_index) {
      button_present = 1;
      if (ADC0_BUTTON_INV == my_adc0) {
        button = (AdcRead(1) < 128);
      }
      else if (ADC0_BUTTON == my_adc0) {
        button = (AdcRead(1) > 128);
      }
    }
#endif  // USE_ADC_VCC

    if (button_present) {
      XdrvMailbox.index = button_index;
      XdrvMailbox.payload = button;
      if (XdrvCall(FUNC_BUTTON_PRESSED)) {
        // Serviced
      }
      else if (SONOFF_4CHPRO == my_module_type) {
        if (Button.hold_timer[button_index]) { Button.hold_timer[button_index]--; }

        bool button_pressed = false;
        if ((PRESSED == button) && (NOT_PRESSED == Button.last_state[button_index])) {
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR(D_LOG_APPLICATION D_BUTTON "%d " D_LEVEL_10), button_index +1);
          Button.hold_timer[button_index] = loops_per_second;
          button_pressed = true;
        }
        if ((NOT_PRESSED == button) && (PRESSED == Button.last_state[button_index])) {
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR(D_LOG_APPLICATION D_BUTTON "%d " D_LEVEL_01), button_index +1);
          if (!Button.hold_timer[button_index]) { button_pressed = true; }  // Do not allow within 1 second
        }
        if (button_pressed) {
          if (!SendKey(KEY_BUTTON, button_index +1, POWER_TOGGLE)) {  // Execute Toggle command via MQTT if ButtonTopic is set
            ExecuteCommandPower(button_index +1, POWER_TOGGLE, SRC_BUTTON);  // Execute Toggle command internally
          }
        }
      }
      else {
        if ((PRESSED == button) && (NOT_PRESSED == Button.last_state[button_index])) {
          if (Settings.flag.button_single) {                   // SetOption13 (0) - Allow only single button press for immediate action
            AddLog_P2(LOG_LEVEL_DEBUG, PSTR(D_LOG_APPLICATION D_BUTTON "%d " D_IMMEDIATE), button_index +1);
            if (!SendKey(KEY_BUTTON, button_index +1, POWER_TOGGLE)) {  // Execute Toggle command via MQTT if ButtonTopic is set
              ExecuteCommandPower(button_index +1, POWER_TOGGLE, SRC_BUTTON);  // Execute Toggle command internally
            }
          } else {
            Button.press_counter[button_index] = (Button.window_timer[button_index]) ? Button.press_counter[button_index] +1 : 1;
            AddLog_P2(LOG_LEVEL_DEBUG, PSTR(D_LOG_APPLICATION D_BUTTON "%d " D_MULTI_PRESS " %d"), button_index +1, Button.press_counter[button_index]);
            Button.window_timer[button_index] = loops_per_second / 2;  // 0.5 second multi press window
          }
          blinks = 201;
        }

        if (NOT_PRESSED == button) {
          Button.hold_timer[button_index] = 0;
        } else {
          Button.hold_timer[button_index]++;
          if (Settings.flag.button_single) {                   // SetOption13 (0) - Allow only single button press for immediate action
            if (Button.hold_timer[button_index] == loops_per_second * hold_time_extent * Settings.param[P_HOLD_TIME] / 10) {  // SetOption32 (40) - Button held for factor times longer
//              Settings.flag.button_single = 0;
              snprintf_P(scmnd, sizeof(scmnd), PSTR(D_CMND_SETOPTION "13 0"));  // Disable single press only
              ExecuteCommand(scmnd, SRC_BUTTON);
            }
          } else {
            if (Settings.flag.button_restrict) {               // SetOption1 (0) - Button restriction
              if (Settings.param[P_HOLD_IGNORE] > 0) {         // SetOption40 (0) - Do not ignore button hold
                if (Button.hold_timer[button_index] > loops_per_second * Settings.param[P_HOLD_IGNORE] / 10) {
                  Button.hold_timer[button_index] = 0;         // Reset button hold counter to stay below hold trigger
                  Button.press_counter[button_index] = 0;      // Discard button press to disable functionality
                  DEBUG_CORE_LOG(PSTR("BTN: " D_BUTTON "%d cancel by " D_CMND_SETOPTION "40 %d"), button_index +1, Settings.param[P_HOLD_IGNORE]);
                }
              }
              if (Button.hold_timer[button_index] == loops_per_second * Settings.param[P_HOLD_TIME] / 10) {  // SetOption32 (40) - Button hold
                Button.press_counter[button_index] = 0;
                SendKey(KEY_BUTTON, button_index +1, POWER_HOLD);  // Execute Hold command via MQTT if ButtonTopic is set
              }
            } else {
              if (Button.hold_timer[button_index] == loops_per_second * hold_time_extent * Settings.param[P_HOLD_TIME] / 10) {  // SetOption32 (40) - Button held for factor times longer
                Button.press_counter[button_index] = 0;
                snprintf_P(scmnd, sizeof(scmnd), PSTR(D_CMND_RESET " 1"));
                ExecuteCommand(scmnd, SRC_BUTTON);
              }
            }
          }
        }

        if (!Settings.flag.button_single) {                    // SetOption13 (0) - Allow multi-press
          if (Button.window_timer[button_index]) {
            Button.window_timer[button_index]--;
          } else {
            if (!restart_flag && !Button.hold_timer[button_index] && (Button.press_counter[button_index] > 0) && (Button.press_counter[button_index] < MAX_BUTTON_COMMANDS +3)) {
              bool single_press = false;
              if (Button.press_counter[button_index] < 3) {    // Single or Double press
                if ((SONOFF_DUAL_R2 == my_module_type) || (SONOFF_DUAL == my_module_type) || (CH4 == my_module_type)) {
                  single_press = true;
                } else {
                  single_press = (Settings.flag.button_swap +1 == Button.press_counter[button_index]);  // SetOption11 (0)
                  if ((1 == Button.present) && (2 == devices_present)) {  // Single Button with two devices only
                    if (Settings.flag.button_swap) {           // SetOption11 (0)
                      Button.press_counter[button_index] = (single_press) ? 1 : 2;
                    }
                  } else {
                    Button.press_counter[button_index] = 1;
                  }
                }
              }
#if defined(USE_LIGHT) && defined(ROTARY_V1)
              if (!((0 == button_index) && RotaryButtonPressed())) {
#endif
                if (single_press && SendKey(KEY_BUTTON, button_index + Button.press_counter[button_index], POWER_TOGGLE)) {  // Execute Toggle command via MQTT if ButtonTopic is set
                  // Success
                } else {
                  if (Button.press_counter[button_index] < 3) {  // Single or Double press
                    if (WifiState() > WIFI_RESTART) {          // Wifimanager active
                      restart_flag = 1;
                    } else {
                      ExecuteCommandPower(button_index + Button.press_counter[button_index], POWER_TOGGLE, SRC_BUTTON);  // Execute Toggle command internally
                    }
                  } else {                                     // 3 - 7 press
                    if (!Settings.flag.button_restrict) {      // SetOption1 (0)
                      GetTextIndexed(scmnd, sizeof(scmnd), Button.press_counter[button_index] -3, kCommands);
                      ExecuteCommand(scmnd, SRC_BUTTON);
                    }
                  }
                }
#if defined(USE_LIGHT) && defined(ROTARY_V1)
              }
#endif
              Button.press_counter[button_index] = 0;
            }
          }
        }
      }
    }
    Button.last_state[button_index] = button;
  }
}

void ButtonLoop(void)
{
  if (Button.present) {
    if (TimeReached(Button.debounce)) {
      SetNextTimeInterval(Button.debounce, Settings.button_debounce);  // ButtonDebounce (50)
      ButtonHandler();
    }
  }
}

#endif  // BUTTON_V1
