/*
  xdrv_23_zigbee_converters.ino - zigbee support for Tasmota

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

typedef struct Z_CommandConverter {
  const char * tasmota_cmd;
  uint16_t     cluster;
  uint8_t      cmd;         // normally 8 bits, 0xFF means it's a parameter
  uint8_t      direction;   // direction of the command. 0x01 client->server, 0x02 server->client, 0x03 both
  const char * param;
} Z_CommandConverter;

typedef struct Z_XYZ_Var {    // Holds values for vairables X, Y and Z
  uint32_t    x = 0;
  uint32_t    y = 0;
  uint32_t    z = 0;
  uint8_t     x_type = 0;     // 0 = no value, 1 = 1 bytes, 2 = 2 bytes
  uint8_t     y_type = 0;
  uint8_t     z_type = 0;
} Z_XYZ_Var;

ZF(AddGroup) ZF(ViewGroup) ZF(GetGroup) ZF(GetAllGroups) ZF(RemoveGroup) ZF(RemoveAllGroups)
ZF(AddScene) ZF(ViewScene) ZF(RemoveScene) ZF(RemoveAllScenes) ZF(RecallScene) ZF(StoreScene) ZF(GetSceneMembership)
//ZF(Power) ZF(Dimmer) 
ZF(DimmerUp) ZF(DimmerDown) ZF(DimmerStop)
ZF(ResetAlarm) ZF(ResetAllAlarms)
//ZF(Hue) ZF(Sat) ZF(CT)
ZF(HueSat) ZF(Color)
ZF(ShutterOpen) ZF(ShutterClose) ZF(ShutterStop) ZF(ShutterLift) ZF(ShutterTilt) ZF(Shutter)
//ZF(Occupancy)
ZF(DimmerMove) ZF(DimmerStep)
ZF(HueMove) ZF(HueStep) ZF(SatMove) ZF(SatStep) ZF(ColorMove) ZF(ColorStep)
ZF(ArrowClick) ZF(ArrowHold) ZF(ArrowRelease) ZF(ZoneStatusChange)

ZF(xxxx00) ZF(xxxx) ZF(01xxxx) ZF(00) ZF(01) ZF() ZF(xxxxyy) ZF(001902) ZF(011902) ZF(xxyyyy) ZF(xx)
ZF(xx000A00) ZF(xx0A00) ZF(xxyy0A00) ZF(xxxxyyyy0A00) ZF(xxxx0A00) ZF(xx0A)
ZF(xx190A00) ZF(xx19) ZF(xx190A) ZF(xxxxyyyy) ZF(xxxxyyzz) ZF(xxyyzzzz) ZF(xxyyyyzz)

// Cluster specific commands
// Note: the table is both for sending commands, but also displaying received commands
// - tasmota_cmd: the human-readable name of the command as entered or displayed, use '|' to split into multiple commands when displayed
// - cluster: cluster number of the command
// - cmd: the command number, of 0xFF if it's actually a variable to be assigned from 'xx'
// - direction: the direction of the command (bit field). 0x01=from client to server (coord to device), 0x02= from server to client (response), 0x80=needs specific decoding
// - param: the paylod template, x/y/z are substituted with arguments, little endian. For command display, payload must match until x/y/z character or until the end of the paylod. '??' means ignore.
const Z_CommandConverter Z_Commands[] PROGMEM = {
  // Group adress commands
  { Z(AddGroup),       0x0004, 0x00, 0x01,   Z(xxxx00) },       // Add group id, group name is not supported
  { Z(ViewGroup),      0x0004, 0x01, 0x01,   Z(xxxx) },         // Ask for the group name
  { Z(GetGroup),       0x0004, 0x02, 0x01,   Z(01xxxx) },       // Get one group membership
  { Z(GetAllGroups),   0x0004, 0x02, 0x01,   Z(00) },           // Get all groups membership
  { Z(RemoveGroup),    0x0004, 0x03, 0x01,   Z(xxxx) },         // Remove one group
  { Z(RemoveAllGroups),0x0004, 0x04, 0x01,   Z() },             // Remove all groups
  // Scenes
  //{ "AddScene",       0x0005, 0x00, 0x01,   "xxxxyy0100" },
  { Z(ViewScene),      0x0005, 0x01, 0x01,   Z(xxxxyy) },
  { Z(RemoveScene),    0x0005, 0x02, 0x01,   Z(xxxxyy) },
  { Z(RemoveAllScenes),0x0005, 0x03, 0x01,   Z(xxxx) },
  { Z(RecallScene),    0x0005, 0x05, 0x01,   Z(xxxxyy) },
  { Z(GetSceneMembership),0x0005, 0x06, 0x01,   Z(xxxx) },
  // Light & Shutter commands
  { Z(Power),          0x0006, 0xFF, 0x01,   Z() },             // 0=Off, 1=On, 2=Toggle
  { Z(Dimmer),         0x0008, 0x04, 0x01,   Z(xx0A00) },       // Move to Level with On/Off, xx=0..254 (255 is invalid)
  { Z(DimmerUp),       0x0008, 0x06, 0x01,   Z(001902) },       // Step up by 10%, 0.2 secs
  { Z(DimmerDown),     0x0008, 0x06, 0x01,   Z(011902) },       // Step down by 10%, 0.2 secs
  { Z(DimmerStop),     0x0008, 0x03, 0x01,   Z() },             // Stop any Dimmer animation
  { Z(ResetAlarm),     0x0009, 0x00, 0x01,   Z(xxyyyy) },       // Reset alarm (alarm code + cluster identifier)
  { Z(ResetAllAlarms), 0x0009, 0x01, 0x01,   Z() },             // Reset all alarms
  { Z(Hue),            0x0300, 0x00, 0x01,   Z(xx000A00) },     // Move to Hue, shortest time, 1s
  { Z(Sat),            0x0300, 0x03, 0x01,   Z(xx0A00) },       // Move to Sat
  { Z(HueSat),         0x0300, 0x06, 0x01,   Z(xxyy0A00) },     // Hue, Sat
  { Z(Color),          0x0300, 0x07, 0x01,   Z(xxxxyyyy0A00) }, // x, y (uint16)
  { Z(CT),             0x0300, 0x0A, 0x01,   Z(xxxx0A00) },     // Color Temperature Mireds (uint16)
  { Z(ShutterOpen),    0x0102, 0x00, 0x01,   Z() },
  { Z(ShutterClose),   0x0102, 0x01, 0x01,   Z() },
  { Z(ShutterStop),    0x0102, 0x02, 0x01,   Z() },
  { Z(ShutterLift),    0x0102, 0x05, 0x01,   Z(xx) },            // Lift percentage, 0%=open, 100%=closed
  { Z(ShutterTilt),    0x0102, 0x08, 0x01,   Z(xx) },            // Tilt percentage
  { Z(Shutter),        0x0102, 0xFF, 0x01,   Z() },
  // Blitzwolf PIR
  { Z(Occupancy),      0xEF00, 0x01, 0x82,   Z()},                // Specific decoder for Blitzwolf PIR, empty name means special treatment
  // Decoders only - normally not used to send, and names may be masked by previous definitions
  { Z(Dimmer),         0x0008, 0x00, 0x01,   Z(xx) },
  { Z(DimmerMove),     0x0008, 0x01, 0x01,   Z(xx0A) },
  { Z(DimmerStep),     0x0008, 0x02, 0x01,   Z(xx190A00) },
  { Z(DimmerMove),     0x0008, 0x05, 0x01,   Z(xx0A) },
  { Z(DimmerUp),       0x0008, 0x06, 0x01,   Z(00) },
  { Z(DimmerDown),     0x0008, 0x06, 0x01,   Z(01) },
  { Z(DimmerStop),     0x0008, 0x07, 0x01,   Z() },
  { Z(HueMove),        0x0300, 0x01, 0x01,   Z(xx19) },
  { Z(HueStep),        0x0300, 0x02, 0x01,   Z(xx190A00) },
  { Z(SatMove),        0x0300, 0x04, 0x01,   Z(xx19) },
  { Z(SatStep),        0x0300, 0x05, 0x01,   Z(xx190A) },
  { Z(ColorMove),      0x0300, 0x08, 0x01,   Z(xxxxyyyy) },
  { Z(ColorStep),      0x0300, 0x09, 0x01,   Z(xxxxyyyy0A00) },
  // Tradfri
  { Z(ArrowClick),     0x0005, 0x07, 0x01,   Z(xx) },         // xx == 0x01 = left, 0x00 = right
  { Z(ArrowHold),      0x0005, 0x08, 0x01,   Z(xx) },         // xx == 0x01 = left, 0x00 = right
  { Z(ArrowRelease),   0x0005, 0x09, 0x01,   Z() },
  // IAS - Intruder Alarm System + leak/fire detection
  { Z(ZoneStatusChange),0x0500, 0x00, 0x82,  Z(xxxxyyzz) },   // xxxx = zone status, yy = extended status, zz = zone id, Delay is ignored
  // responses for Group cluster commands
  { Z(AddGroup),       0x0004, 0x00, 0x82,   Z(xxyyyy) },       // xx = status, yy = group id
  { Z(ViewGroup),      0x0004, 0x01, 0x82,   Z(xxyyyy) },       // xx = status, yy = group id, name ignored
  { Z(GetGroup),       0x0004, 0x02, 0x82,   Z(xxyyzzzz) },     // xx = capacity, yy = count, zzzz = first group id, following groups ignored
  { Z(RemoveGroup),    0x0004, 0x03, 0x82,   Z(xxyyyy) },       // xx = status, yy = group id
  // responses for Scene cluster commands
  { Z(AddScene),       0x0005, 0x00, 0x82,   Z(xxyyyyzz) },     // xx = status, yyyy = group id, zz = scene id
  { Z(ViewScene),      0x0005, 0x01, 0x82,   Z(xxyyyyzz) },     // xx = status, yyyy = group id, zz = scene id
  { Z(RemoveScene),    0x0005, 0x02, 0x82,   Z(xxyyyyzz) },     // xx = status, yyyy = group id, zz = scene id
  { Z(RemoveAllScenes),0x0005, 0x03, 0x82,   Z(xxyyyy) },     // xx = status, yyyy = group id
  { Z(StoreScene),     0x0005, 0x04, 0x82,   Z(xxyyyyzz) },     // xx = status, yyyy = group id, zz = scene id
  { Z(GetSceneMembership),0x0005, 0x06, 0x82,Z(xxyyzzzz) },     // specific
};

#define ZLE(x) ((x) & 0xFF), ((x) >> 8)     // Little Endian

// Below are the attributes we wand to read from each cluster
const uint8_t CLUSTER_0006[] = { ZLE(0x0000) };    // Power
const uint8_t CLUSTER_0008[] = { ZLE(0x0000) };    // CurrentLevel
const uint8_t CLUSTER_0009[] = { ZLE(0x0000) };    // AlarmCount
const uint8_t CLUSTER_0300[] = { ZLE(0x0000), ZLE(0x0001), ZLE(0x0003), ZLE(0x0004), ZLE(0x0007), ZLE(0x0008) };    // Hue, Sat, X, Y, CT, ColorMode

// This callback is registered after a cluster specific command and sends a read command for the same cluster
int32_t Z_ReadAttrCallback(uint16_t shortaddr, uint16_t groupaddr, uint16_t cluster, uint8_t endpoint, uint32_t value) {
  size_t         attrs_len = 0;
  const uint8_t* attrs = nullptr;

  switch (cluster) {
    case 0x0006:                              // for On/Off
      attrs = CLUSTER_0006;
      attrs_len = sizeof(CLUSTER_0006);
      break;
    case 0x0008:                              // for Dimmer
      attrs = CLUSTER_0008;
      attrs_len = sizeof(CLUSTER_0008);
      break;
    case 0x0009:                              // for Alarms
      attrs = CLUSTER_0009;
      attrs_len = sizeof(CLUSTER_0009);
      break;
    case 0x0300:                              // for Lights
      attrs = CLUSTER_0300;
      attrs_len = sizeof(CLUSTER_0300);
      break;
  }
  if (attrs) {
    ZigbeeZCLSend_Raw(shortaddr, groupaddr, cluster, endpoint, ZCL_READ_ATTRIBUTES, false, attrs, attrs_len, true /* we do want a response */, zigbee_devices.getNextSeqNumber(shortaddr));
  }
}

// set a timer to read back the value in the future
void zigbeeSetCommandTimer(uint16_t shortaddr, uint16_t groupaddr, uint16_t cluster, uint8_t endpoint) {
  uint32_t wait_ms = 0;

  switch (cluster) {
    case 0x0006:        // for On/Off
    case 0x0009:        // for Alamrs
      wait_ms = 200;    // wait 0.2 s
      break;
    case 0x0008:        // for Dimmer
    case 0x0300:        // for Color
      wait_ms = 1050;   // wait 1.0 s
      break;
    case 0x0102:        // for Shutters
      wait_ms = 10000;  // wait 10.0 s
      break;
  }
  if (wait_ms) {
    zigbee_devices.setTimer(shortaddr, groupaddr, wait_ms, cluster, endpoint, Z_CAT_NONE, 0 /* value */, &Z_ReadAttrCallback);
  }
}

// returns true if char is 'x', 'y' or 'z'
inline bool isXYZ(char c) {
  return (c >= 'x') && (c <= 'z');
}

// returns the Hex value of a digit [0-9A-Fa-f]
// return: 0x00-0x0F
// or -1 if cannot be parsed
inline int8_t hexValue(char c) {
  if ((c >= '0') && (c <= '9')) {
    return c - '0';
  }
  if ((c >= 'A') && (c <= 'F')) {
    return 10 + c - 'A';
  }
  if ((c >= 'a') && (c <= 'f')) {
    return 10 + c - 'a';
  }
  return -1;
}

// Parse a Big Endian suite of max_len digits, or stops when a non-hex digit is found
uint32_t parseHex_P(const char **data, size_t max_len = 8) {
  uint32_t ret = 0;
  for (uint32_t i = 0; i < max_len; i++) {
    int8_t v = hexValue(pgm_read_byte(*data));
    if (v < 0) { break; }     // non hex digit, we stop parsing
    ret = (ret << 4) | v;
    *data += 1;
  }
  return ret;
}

// Parse a model like "xxyy00"
// and fill x, y and z values
// Little Endian encoding
// On exit, xyz is updated, and x_type, y_type, z_type contain the number of bytes read for each
void parseXYZ(const char *model, const SBuffer &payload, struct Z_XYZ_Var *xyz) {
  const char *p = model;    // pointer to the model character
  uint32_t   v = 0;         // index in the payload bytes buffer
  char c = pgm_read_byte(p);  // cur char
  while (c) {
    char c1 = pgm_read_byte(p+1);   // next char
    if (!c1) { break; }   // unexpected end of model
    if (isXYZ(c) && (c == c1) && (v < payload.len())) {    // if char is [x-z] and followed by same char
      uint8_t val = payload.get8(v);
      switch (c) {
        case 'x':
          xyz->x = xyz->x | (val << (xyz->x_type * 8));
          xyz->x_type++;
          break;
        case 'y':
          xyz->y = xyz->y | (val << (xyz->y_type * 8));
          xyz->y_type++;
          break;
        case 'z':
          xyz->z = xyz->z | (val << (xyz->z_type * 8));
          xyz->z_type++;
          break;
      }
    }
    p += 2;
    v++;
    c = pgm_read_byte(p);
  }
}

// works on big endiand hex only
// Returns if found:
//  - cluster number
//  - command number or 0xFF if command is part of the variable part
//  - the payload in the form of a HEX string with x/y/z variables
void sendHueUpdate(uint16_t shortaddr, uint16_t groupaddr, uint16_t cluster, uint8_t cmd, bool direction) {
  if (direction) { return; }    // no need to update if server->client

  int32_t z_cat = -1;
  uint32_t wait_ms = 0;

  switch (cluster) {
    case 0x0006:
      z_cat = Z_CAT_READ_0006;
      wait_ms = 200;    // wait 0.2 s
      break;
    case 0x0008:
      z_cat = Z_CAT_READ_0008;
      wait_ms = 1050;   // wait 1.0 s
      break;
    case 0x0102:
      z_cat = Z_CAT_READ_0102;
      wait_ms = 10000;   // wait 10.0 s
      break;
    case 0x0300:
      z_cat = Z_CAT_READ_0300;
      wait_ms = 1050;   // wait 1.0 s
      break;
    default:
      break;
  }
  if (z_cat >= 0) {
    uint8_t endpoint = 0;
    if (!groupaddr) {
      endpoint = zigbee_devices.findFirstEndpoint(shortaddr);
    }
    if ((endpoint) || (groupaddr)) {   // send only if we know the endpoint
      zigbee_devices.setTimer(shortaddr, groupaddr, wait_ms, cluster, endpoint, z_cat, 0 /* value */, &Z_ReadAttrCallback);
    }
  }
}


// Parse a cluster specific command, and try to convert into human readable
void convertClusterSpecific(JsonObject& json, uint16_t cluster, uint8_t cmd, bool direction, const SBuffer &payload) {
  size_t hex_char_len = payload.len()*2+2;
  char *hex_char = (char*) malloc(hex_char_len);
  if (!hex_char) { return; }
  ToHex_P((unsigned char*)payload.getBuffer(), payload.len(), hex_char, hex_char_len);

  const __FlashStringHelper* command_name = nullptr;
  uint8_t conv_direction;
  Z_XYZ_Var xyz;

//AddLog_P2(LOG_LEVEL_INFO, PSTR(">>> len = %d - %02X%02X%02X"), payload.len(), payload.get8(0), payload.get8(1), payload.get8(2));
  for (uint32_t i = 0; i < sizeof(Z_Commands) / sizeof(Z_Commands[0]); i++) {
    const Z_CommandConverter *conv = &Z_Commands[i];
    uint16_t conv_cluster = pgm_read_word(&conv->cluster);
    if (conv_cluster == cluster) {
      // cluster match
      uint8_t conv_cmd = pgm_read_byte(&conv->cmd);
      conv_direction = pgm_read_byte(&conv->direction);
      if ((0xFF == conv_cmd) || (cmd == conv_cmd)) {
          // cmd match
        if ((direction && (conv_direction & 0x02)) || (!direction && (conv_direction & 0x01))) {
          // check if we have a match for params too
          // Match if:
          //  - payload exactly matches conv->param (conv->param may be longer)
          //  - payload matches conv->param until 'x', 'y' or 'z'
          const char * p = conv->param;
  //AddLog_P2(LOG_LEVEL_INFO, PSTR(">>>++1 param = %s"), p);
          bool match = true;
          for (uint8_t i = 0; i < payload.len(); i++) {
            const char c1 = pgm_read_byte(p);
            const char c2 = pgm_read_byte(p+1);
  //AddLog_P2(LOG_LEVEL_INFO, PSTR(">>>++2 c1 = %c, c2 = %c"), c1, c2);
            if ((0x00 == c1) || isXYZ(c1)) {
              break;
            }
            const char * p2 = p;
            uint32_t nextbyte = parseHex_P(&p2, 2);
  //AddLog_P2(LOG_LEVEL_INFO, PSTR(">>>++3 parseHex_P = %02X"), nextbyte);
            if (nextbyte != payload.get8(i)) {
              match = false;
              break;
            }
            p += 2;
          }
          if (match) {
            command_name = (const __FlashStringHelper*) conv->tasmota_cmd;
            parseXYZ(conv->param, payload, &xyz);
            if (0xFF == conv_cmd) {
              // shift all values
              xyz.z = xyz.y;
              xyz.z_type = xyz.y_type;
              xyz.y = xyz.x;
              xyz.y_type = xyz.x_type;
              xyz.x = cmd;
              xyz.x_type = 1;   // 1 byte
            }
            break;
          }
        }
      }
    }
  }

  // always report attribute in raw format
  // Format: "0001!06": "00" = "<cluster>!<cmd>": "<payload>" for commands to devices
  // Format: "0004<00": "00" = "<cluster><<cmd>": "<payload>" for commands to devices
  char attrid_str[12];
  snprintf_P(attrid_str, sizeof(attrid_str), PSTR("%04X%c%02X"), cluster, direction ? '<' : '!', cmd);
  json[attrid_str] = hex_char;
  free(hex_char);

  if (command_name) {
    // Now try to transform into a human readable format
    // if (direction & 0x80) then specific transform
    if (conv_direction & 0x80) {
      // TODO need to create a specific command
      // IAS
      String command_name2 = String(command_name);
      if ((cluster == 0x0500) && (cmd == 0x00)) {
        // "ZoneStatusChange"
        json[command_name] = xyz.x;
        json[command_name2 + F("Ext")] = xyz.y;
        json[command_name2 + F("Zone")] = xyz.z;
      } else if ((cluster == 0x0004) && ((cmd == 0x00) || (cmd == 0x01) || (cmd == 0x03))) {
        // AddGroupResp or ViewGroupResp (group name ignored) or RemoveGroup
        json[command_name] = xyz.y;
        json[command_name2 + F("Status")] = xyz.x;
        json[command_name2 + F("StatusMsg")] = getZigbeeStatusMessage(xyz.x);
      } else if ((cluster == 0x0004) && (cmd == 0x02)) {
        // GetGroupResp
        json[command_name2 + F("Capacity")] = xyz.x;
        json[command_name2 + F("Count")] = xyz.y;
        JsonArray &arr = json.createNestedArray(command_name);
        for (uint32_t i = 0; i < xyz.y; i++) {
          arr.add(payload.get16(2 + 2*i));
        }
      } else if ((cluster == 0x0005) && ((cmd == 0x00) || (cmd == 0x02) || (cmd == 0x03))) {
        // AddScene or RemoveScene or StoreScene
        json[command_name2 + F("Status")] = xyz.x;
        json[command_name2 + F("StatusMsg")] = getZigbeeStatusMessage(xyz.x);
        json[F("GroupId")] = xyz.y;
        json[F("SceneId")] = xyz.z;
      } else if ((cluster == 0x0005) && (cmd == 0x01)) {
        // ViewScene
        json[command_name2 + F("Status")] = xyz.x;
        json[command_name2 + F("StatusMsg")] = getZigbeeStatusMessage(xyz.x);
        json[F("GroupId")] = xyz.y;
        json[F("SceneId")] = xyz.z;
        String scene_payload = json[attrid_str];
        json[F("ScenePayload")] = scene_payload.substring(8); // remove first 8 characters
      } else if ((cluster == 0x0005) && (cmd == 0x03)) {
        // RemoveAllScenes
        json[command_name2 + F("Status")] = xyz.x;
        json[command_name2 + F("StatusMsg")] = getZigbeeStatusMessage(xyz.x);
        json[F("GroupId")] = xyz.y;
      } else if ((cluster == 0x0005) && (cmd == 0x06)) {
        // GetSceneMembership
        json[command_name2 + F("Status")] = xyz.x;
        json[command_name2 + F("StatusMsg")] = getZigbeeStatusMessage(xyz.x);
        json[F("Capacity")] = xyz.y;
        json[F("GroupId")] = xyz.z;
        String scene_payload = json[attrid_str];
        json[F("ScenePayload")] = scene_payload.substring(8); // remove first 8 characters
      }
    } else {
      if (0 == xyz.x_type) {
        json[command_name] = true;    // no parameter
      } else if (0 == xyz.y_type) {
        json[command_name] = xyz.x;       // 1 parameter
      } else {
        // multiple answers, create an array
        JsonArray &arr = json.createNestedArray(command_name);
        arr.add(xyz.x);
        arr.add(xyz.y);
        if (xyz.z_type) {
          arr.add(xyz.z);
        }
      }
    }
  }
}

// Find the command details by command name
// Only take commands outgoing, i.e. direction == 0
// If not found:
//  - returns nullptr
const __FlashStringHelper* zigbeeFindCommand(const char *command, uint16_t *cluster, uint16_t *cmd) {
  for (uint32_t i = 0; i < sizeof(Z_Commands) / sizeof(Z_Commands[0]); i++) {
    const Z_CommandConverter *conv = &Z_Commands[i];
    uint8_t conv_direction = pgm_read_byte(&conv->direction);
    uint8_t conv_cmd = pgm_read_byte(&conv->cmd);
    uint16_t conv_cluster = pgm_read_word(&conv->cluster);
    if ((conv_direction & 0x01) && (0 == strcasecmp_P(command, conv->tasmota_cmd))) {
      *cluster = conv_cluster;
      *cmd = conv_cmd;
      return (const __FlashStringHelper*) conv->param;
    }
  }

  return nullptr;
}

// take the lower 4 bits and turn it to an hex char
inline char hexDigit(uint32_t h) {
  uint32_t nybble = h & 0x0F;
  return (nybble > 9) ? 'A' - 10 + nybble : '0' + nybble;
}

// replace all xx/yy/zz substrings with unsigned ints, and the corresponding len (8, 16 or 32 bits)
String zigbeeCmdAddParams(const char *zcl_cmd_P, uint32_t x, uint32_t y, uint32_t z) {
  size_t len = strlen_P(zcl_cmd_P);
  char zcl_cmd[len+1];
  strcpy_P(zcl_cmd, zcl_cmd_P);     // copy into RAM

  char *p = zcl_cmd;
  while (*p) {
    if (isXYZ(*p) && (*p == *(p+1))) {    // if char is [x-z] and followed by same char
      uint8_t val;
      switch (*p) {
        case 'x':
          val = x & 0xFF;
          x = x >> 8;
          break;
        case 'y':
          val = y & 0xFF;
          y = y >> 8;
          break;
        case 'z':
          val = z & 0xFF;
          z = z >> 8;
          break;
      }
      *p = hexDigit(val >> 4);
      *(p+1) = hexDigit(val);
      p++;
    }
    p++;
  }
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SendZCLCommand_P: zcl_cmd = %s"), zcl_cmd);

  return String(zcl_cmd);
}

const char kZ_Alias[] PROGMEM = "OFF|" D_OFF "|" D_FALSE "|" D_STOP  "|" "OPEN" "|"           // 0
                                "ON|"  D_ON  "|" D_TRUE  "|" D_START "|" "CLOSE" "|"          // 1
                                "TOGGLE|" D_TOGGLE "|"                                        // 2
                                "ALL" ;                                                       // 255

const uint8_t kZ_Numbers[] PROGMEM = { 0,0,0,0,0,
                                       1,1,1,1,1,
                                       2,2,
                                       255 };

// Convert an alias like "On" to the corresponding number
uint32_t ZigbeeAliasOrNumber(const char *state_text) {
  char command[16];
  int state_number = GetCommandCode(command, sizeof(command), state_text, kZ_Alias);
  if (state_number >= 0) {
    // found an alias, get its value
    return pgm_read_byte(kZ_Numbers + state_number);
  } else {
    // no alias found, convert it as number
    return strtoul(state_text, nullptr, 0);
  }
}

#endif // USE_ZIGBEE
