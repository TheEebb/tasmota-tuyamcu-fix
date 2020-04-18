/*
  xdrv_23_zigbee_1_headers.ino - zigbee support for Tasmota

  Copyright (C) 2020  Theo Arends and Stephan Hadinger

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

#ifdef USE_ZIGBEE

// contains some definitions for functions used before their declarations

void ZigbeeZCLSend_Raw(uint16_t dtsAddr, uint16_t groupaddr, uint16_t clusterId, uint8_t endpoint, uint8_t cmdId, bool clusterSpecific, const uint8_t *msg, size_t len, bool needResponse, uint8_t transacId);


// Get an JSON attribute, with case insensitive key search
JsonVariant &getCaseInsensitive(const JsonObject &json, const char *needle) {
  // key can be in PROGMEM
  if ((nullptr == &json) || (nullptr == needle) || (0 == pgm_read_byte(needle))) {
    return *(JsonVariant*)nullptr;
  }

  for (auto kv : json) {
    const char *key = kv.key;
    JsonVariant &value = kv.value;

    if (0 == strcasecmp_P(key, needle)) {
      return value;
    }
  }
  // if not found
  return *(JsonVariant*)nullptr;
}

uint32_t parseHex(const char **data, size_t max_len = 8) {
  uint32_t ret = 0;
  for (uint32_t i = 0; i < max_len; i++) {
    int8_t v = hexValue(**data);
    if (v < 0) { break; }     // non hex digit, we stop parsing
    ret = (ret << 4) | v;
    *data += 1;
  }
  return ret;
}

#endif // USE_ZIGBEE
