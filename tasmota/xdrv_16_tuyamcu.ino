/*
  xdrv_16_tuyamcu.ino - Tuya MCU support for Tasmota

  Copyright (C) 2020  digiblur, Joel Stein and Theo Arends

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

#ifdef USE_LIGHT
#ifdef USE_TUYA_MCU

#define XDRV_16                16
#define XNRG_32                32   // Needs to be the last XNRG_xx

#ifndef TUYA_DIMMER_ID
#define TUYA_DIMMER_ID         0
#endif

#define TUYA_CMD_HEARTBEAT     0x00
#define TUYA_CMD_QUERY_PRODUCT 0x01
#define TUYA_CMD_MCU_CONF      0x02
#define TUYA_CMD_WIFI_STATE    0x03
#define TUYA_CMD_WIFI_RESET    0x04
#define TUYA_CMD_WIFI_SELECT   0x05
#define TUYA_CMD_SET_DP        0x06
#define TUYA_CMD_STATE         0x07
#define TUYA_CMD_QUERY_STATE   0x08
#define TUYA_CMD_SET_TIME      0x1C
#define TUYA_CMD_INVALID       0xFF   // Not an actual command - reserved for invalid

#define TUYA_LOW_POWER_CMD_WIFI_STATE   0x02
#define TUYA_LOW_POWER_CMD_WIFI_RESET   0x03
#define TUYA_LOW_POWER_CMD_WIFI_CONFIG  0x04
#define TUYA_LOW_POWER_CMD_STATE        0x05

#define TUYA_TYPE_BOOL         0x01
#define TUYA_TYPE_VALUE        0x02
#define TUYA_TYPE_STRING       0x03
#define TUYA_TYPE_ENUM         0x04

#define TUYA_BUFFER_SIZE       256
#define TUYA_TX_QUEUE_SIZE     16     // Queue bytes = TUYA_TX_QUEUE_SIZE * sizeof(struct TUYA_TX_BLOCK). (Min: 1)
#define TUYA_RX_TIMEOUT        300    // Wait up to 300ms (6 * 50ms default tasmota sleep) for response from MCU. (Max: 256 * Settings.sleep, 0 = Unlimited)
#define TUYA_RX_WAIT_TIME      3      // Approx the amount of time (ms needed to process and send 15 bytes (the non-string response)
#define TUYA_INIT_SLEEP        10     // Millis to sleep main loop during init phase

#include <TasmotaSerial.h>

TasmotaSerial *TuyaSerial = nullptr;

#if (__GNUC__ || __clang__)     // gcc or llvm (mac & linux)
typedef struct __attribute__ ((packed)) TUYA_TX_BLOCK {
#else
#pragma pack(push, 1)           // ms c++
typedef struct TUYA_TX_BLOCK {
#endif
  uint8_t comm_mode;
  uint8_t timeout;        // Used for comm modes with reply
  uint8_t cmd_type;
  union {
    struct {
      uint8_t data_type;
      uint8_t dpid;
    };
    uint16_t length;
  };
  union {
    uint32_t value;       // Tuya docs define all value data has four bytes
    uint8_t* data;        // Pointer to byte data. Strings not supported by queue- only during pre-send
  };
} TUYA_TX_BLOCK;

#if (__GNUC__ || __clang__)     // gcc or llvm (mac & linux)
typedef struct __attribute__ ((packed)) TuyaValueDataFrame {
#else
typedef struct TuyaValueDataFrame {
#endif
	uint8_t header[2];
	uint8_t version;
	uint8_t command;
	uint8_t data_length[2];
	struct {
		uint8_t dpid;
		uint8_t data_type;
		uint8_t function_length[2];
		union {
			uint8_t  command1;
			uint8_t  command4[4];
      uint8_t* commandp;
		};
	} function;
} TuyaValueDataFrame;
#if !(__GNUC__ || __clang__)    // ms c++
#pragma pack(pop)
#endif

 // Code assumes 32b processor. Needs to be updated if tasmota ported to 64b or other
static_assert(sizeof(struct TUYA_TX_BLOCK) == 9, "TUYA_TX_BLOCK is not packed correctly. Check compiling flags");
static_assert(sizeof(struct TuyaValueDataFrame) == 14, "TuyaValueDataFrame is not packed correctly. Check compiling flags");

struct TUYA {
  uint16_t new_dim = 0;                  // Tuya dimmer value temp
  bool ignore_dim = false;               // Flag to skip serial send to prevent looping when processing inbound states from the faceplate interaction
  bool slave_mode = true;                // Flag to operate WiFi module as slave to MCU
  uint8_t mcu_stage = 0;                 // Prevent command delivery to MCU until MCU is initialized
  uint8_t esp_mode = 0;                  // Current operating mode for this module
  uint8_t cmd_status = 0;                // Current status of serial-read
  uint8_t cmd_checksum = 0;              // Checksum of tuya command
  uint16_t data_len = 0;                 // Data lenght of command
  uint8_t wifi_state = -2;               // Keep MCU wifi-status in sync with WifiState()
  uint8_t heartbeat_timer = 0;           // 10 second heartbeat timer for tuya module
#ifdef USE_ENERGY_SENSOR
  uint32_t lastPowerCheckTime = 0;       // Time when last power was checked
#endif // USE_ENERGY_SENSOR
  char *buffer = nullptr;                // Serial receive buffer
  int byte_counter = 0;                  // Index in serial receive buffer
  bool low_power_mode = false;           // Normal or Low power mode protocol <-- TODO: replace with operating mode
  bool send_success_next_second = false; // Second command success in low power mode
  uint32_t ignore_dimmer_cmd_timeout = 0;// Time until which received dimmer commands should be ignored

  struct {
    uint8_t data_status = 0;             // Flag to mark when data transmission has occured during this loop
    // Flag to send data during pre-send stage. Currently used to send strings because we''re not storing them in queue
    TUYA_TX_BLOCK* pre_send_data = nullptr;
    TUYA_TX_BLOCK last_tx;               // Current TX data block
  } comm;
  struct {                               // command transmit queue
    TUYA_TX_BLOCK *blocks = nullptr;
    int8_t head = 0;
    int8_t tail = -1;
  } queue;
} Tuya;

enum TuyaCommDataStatus {
  TUYA_COMM_DATA_STATUS_AVAILABLE = 0,
  TUYA_COMM_DATA_STATUS_SENT,
  TUYA_COMM_DATA_STATUS_BLIND_RECEIVE,
  TUYA_COMM_DATA_STATUS_TIMEOUT,
  TUYA_COMM_DATA_STATUS_FAILED,
};

enum TuyaCommunicationMode {
  TUYA_COMM_MODE_NONE = 0,
  TUYA_COMM_REQUIRE_REPLY,    // Reply required in the format expected
  TUYA_COMM_OPTIONAL_REPLY,   // Reply may contain any type of data or even no reply given. Used when reply should be processed regardless of contents
  TUYA_COMM_IGNORE_REPLY,     // Reply is expected but data is invalid. Response will not be processed but still available to mqtt
  TUYA_COMM_NO_REPLY          // Reply is not expected. Any response is considered to be coincidental and unrelated to sent data.
                              // > No reply can be used where send, receive, and processing can occur in any order without any dependencies
};

enum TuyaMCUStates {
  TUYA_MCU_STATE_NONE = 0,
  TUYA_MCU_STATE_HEARTBEAT,
  TUYA_MCU_STATE_QUERY,
  TUYA_MCU_STATE_MODE,
  TUYA_MCU_STATE_WIFI,
  TUYA_MCU_STATE_SYNC,
  TUYA_MCU_STATE_READY
};

// Tuya -> MCU Init sequence corespondence command table
// Options can be set to allow different mapping based on device
#define TUYA_CMD_RESP_TABLE_SIZE    9
static uint8_t TUYA_CMD_RESP_TABLE[TUYA_CMD_RESP_TABLE_SIZE * 2] = {
                                                      // Wifi Command             Reply type    MCU Command
                                                      //                          
  TUYA_CMD_HEARTBEAT,       TUYA_COMM_REQUIRE_REPLY,  // TUYA_CMD_HEARTBEAT       required      TUYA_CMD_HEARTBEAT
  TUYA_CMD_QUERY_PRODUCT,   TUYA_COMM_REQUIRE_REPLY,  // TUYA_CMD_QUERY_PRODUCT   required      TUYA_CMD_QUERY_PRODUCT         
  TUYA_CMD_MCU_CONF,        TUYA_COMM_REQUIRE_REPLY,  // TUYA_CMD_MCU_CONF        required      TUYA_CMD_MCU_CONF
  TUYA_CMD_WIFI_STATE,      TUYA_COMM_OPTIONAL_REPLY, // TUYA_CMD_WIFI_STATE      optional      TUYA_CMD_WIFI_STATE
  TUYA_CMD_INVALID,         TUYA_COMM_MODE_NONE,      // Not Applicable
  TUYA_CMD_INVALID,         TUYA_COMM_MODE_NONE,      // Not Applicable
  TUYA_CMD_STATE,           TUYA_COMM_REQUIRE_REPLY,  // TUYA_CMD_SET_DP          required      TUYA_CMD_STATE
  TUYA_CMD_INVALID,         TUYA_COMM_MODE_NONE,      // Not Applicable
  TUYA_CMD_STATE,           TUYA_COMM_REQUIRE_REPLY   // TUYA_CMD_QUERY_STATE     required      TUYA_CMD_STATE
};

static const uint8_t TUYA_MODE_MCU_SEQ_INIT[] = {
  TUYA_MCU_STATE_HEARTBEAT,
  TUYA_MCU_STATE_QUERY,
  TUYA_MCU_STATE_MODE,
  TUYA_MCU_STATE_WIFI,
  TUYA_MCU_STATE_SYNC,
  TUYA_MCU_STATE_READY
};

static const uint8_t TUYA_MODE_MCU_SEQ_SYNC[] = {
  TUYA_MCU_STATE_HEARTBEAT,
  TUYA_MCU_STATE_WIFI,
  TUYA_MCU_STATE_SYNC,
  TUYA_MCU_STATE_READY
};

enum TuyaOperatingMode {
  TUYA_MODE_NORMAL_POWER_INIT,
  TUYA_MODE_NORMAL_POWER_SYNC,
  TUYA_MODE_NORMAL_POWER_READY,
  TUYA_MODE_LOW_POWER_INIT,
  TUYA_MODE_LOW_POWER_SYNC,
  TUYA_MODE_LOW_POWER_READY
};

enum TuyaCommunicationDataReply {
  TUYA_COMM_DATA_NONE = 0,
  TUYA_COMM_DATA_INCOMPLETE,
  TUYA_COMM_DATA_INVALID,
  TUYA_COMM_DATA_INTERNAL,
  TUYA_COMM_DATA_DP_MATCH,
  TUYA_COMM_DATA_DP_NOMATCH,
};

// enum TuyaSupportedFunctions {
//   TUYA_MCU_FUNC_NONE,
//   TUYA_MCU_FUNC_SWT1 = 1,           // Buttons
//   TUYA_MCU_FUNC_SWT2,
//   TUYA_MCU_FUNC_SWT3,
//   TUYA_MCU_FUNC_SWT4,
//   TUYA_MCU_FUNC_REL1 = 11,           // Relays
//   TUYA_MCU_FUNC_REL2,
//   TUYA_MCU_FUNC_REL3,
//   TUYA_MCU_FUNC_REL4,
//   TUYA_MCU_FUNC_REL5,
//   TUYA_MCU_FUNC_REL6,
//   TUYA_MCU_FUNC_REL7,
//   TUYA_MCU_FUNC_REL8,
//   TUYA_MCU_FUNC_DIMMER = 21,
//   TUYA_MCU_FUNC_POWER = 31,
//   TUYA_MCU_FUNC_CURRENT,
//   TUYA_MCU_FUNC_VOLTAGE,
//   TUYA_MCU_FUNC_BATTERY_STATE,
//   TUYA_MCU_FUNC_BATTERY_PERCENTAGE,
//   TUYA_MCU_FUNC_REL1_INV = 41,           // Inverted Relays
//   TUYA_MCU_FUNC_REL2_INV,
//   TUYA_MCU_FUNC_REL3_INV,
//   TUYA_MCU_FUNC_REL4_INV,
//   TUYA_MCU_FUNC_REL5_INV,
//   TUYA_MCU_FUNC_REL6_INV,
//   TUYA_MCU_FUNC_REL7_INV,
//   TUYA_MCU_FUNC_REL8_INV,
//   TUYA_MCU_FUNC_LOWPOWER_MODE = 51,
//   TUYA_MCU_FUNC_LAST = 255
// };

void TuyaSendCmd(uint8_t cmd, uint8_t payload[] = nullptr, uint16_t payload_len = 0);
bool TuyaProcessSendState(uint8_t data_type, uint8_t dpid, uint32_t value, uint8_t comm_mode, uint timeout = TUYA_RX_TIMEOUT);
bool TuyaProcessSendString(uint8_t dpid, char* data, uint8_t comm_mode, uint timeout = TUYA_RX_TIMEOUT);

const char kTuyaCommand[] PROGMEM = "|"  // No prefix
  D_CMND_TUYA_MCU "|" D_CMND_TUYA_MCU_SEND_STATE;

void (* const TuyaCommand[])(void) PROGMEM = {
  &CmndTuyaMcu, &CmndTuyaSend
};

/*

TuyaSend<x> dpId,data

TuyaSend0 -> Sends TUYA_CMD_QUERY_STATE
TuyaSend1 11,1 -> Sends boolean (Type 1) data 0/1 to dpId 11 (Max data length 1 byte)
TuyaSend2 11,100 -> Sends integer (Type 2) data 100 to dpId 11 (Max data length 4 bytes)
TuyaSend2 11,0xAABBCCDD -> Sends 4 bytes (Type 2) data to dpId 11 (Max data length 4 bytes)
TuyaSend3 11,ThisIsTheData -> Sends the supplied string (Type 3) to dpId 11 ( Max data length not-known)
TuyaSend4 11,1 -> Sends enum (Type 4) data 0/1/2/3/4/5 to dpId 11 (Max data length 1 bytes)

= Optional Parameters =

TuyaSend<x> dpId,data,[comm mode],[timeout]
Example: TuyaSend1 11,1,3,100 -> Send to dpId 11 a boolean as true (1) with ignore reply (3) and 100ms timeout

- Comm mode:  1 -> require reply (wait until timeout for dpid matching response)
              2 -> optional reply (wait until timeout for any type of reply)
              3 -> ignore reply (discard any received reply)
              4 -> no reply (reply not expected. additional data can be sent immediately without waiting)
              (If comm mode exceeds 4, default reply is automatically chosen)
- Timeout:    number of milliseconds to wait for response
*/


void CmndTuyaSend(void) {
  if (XdrvMailbox.index > 4) {
    return;
  }
  if (XdrvMailbox.index == 0) {
    TuyaRequestState();
  } else {
    if (XdrvMailbox.data_len > 0) {
      char *p;
      char *data;
      uint8_t i = 0;
      uint8_t dpId = 0;
      uint comm_mode = TUYA_COMM_MODE_NONE + 1;
      uint timeout = TUYA_RX_TIMEOUT;
      for (char *str = strtok_r(XdrvMailbox.data, ", ", &p); str && i < 4; str = strtok_r(nullptr, ", ", &p)) {
        switch (i) {
          case 0:
            dpId = strtoul(str, nullptr, 0);
            break;
          case 1:
            data = str;
            break;
          case 2:
            comm_mode = strtoul(str, nullptr, 0);
            break;
          case 3:
            timeout = strtoul(str, nullptr, 0);
            break;
        }
        i++;
      }

      if (1 == XdrvMailbox.index) {
        if (comm_mode > TUYA_COMM_MODE_NONE) comm_mode = TuyaCommDefaultTypeMode(TUYA_TYPE_BOOL);
        TuyaProcessSendState(TUYA_TYPE_BOOL, dpId, strtoul(data, nullptr, 0), comm_mode, timeout);
      } else if (2 == XdrvMailbox.index) {
        if (comm_mode > TUYA_COMM_MODE_NONE) comm_mode = TuyaCommDefaultTypeMode(TUYA_TYPE_VALUE);
        TuyaProcessSendState(TUYA_TYPE_VALUE, dpId, strtoull(data, nullptr, 0), comm_mode, timeout);
      } else if (3 == XdrvMailbox.index) {
        if (comm_mode > TUYA_COMM_MODE_NONE) comm_mode = TuyaCommDefaultTypeMode(TUYA_TYPE_STRING);
        TuyaProcessSendString(dpId, data, comm_mode, timeout);
      } else if (4 == XdrvMailbox.index) {
        if (comm_mode > TUYA_COMM_MODE_NONE) comm_mode = TuyaCommDefaultTypeMode(TUYA_TYPE_ENUM);
        TuyaProcessSendState(TUYA_TYPE_ENUM, dpId, strtoul(data, nullptr, 0), comm_mode, timeout);
      }
    }
  }
  ResponseCmndDone();
}

/*

TuyaMcu fnid,dpid

*/

void CmndTuyaMcu(void) {
  if (XdrvMailbox.data_len > 0) {
    char *p;
    uint8_t i = 0;
    uint8_t parm[3] = { 0 };
    for (char *str = strtok_r(XdrvMailbox.data, ", ", &p); str && i < 2; str = strtok_r(nullptr, ", ", &p)) {
      parm[i] = strtoul(str, nullptr, 0);
      i++;
    }

    if (TuyaFuncIdValid(parm[0])) {
      TuyaAddMcuFunc(parm[0], parm[1]);
      restart_flag = 2;
    } else {
      AddLog_P2(LOG_LEVEL_ERROR, PSTR("TYA: TuyaMcu Invalid function id=%d"), parm[0]);
    }

  }

  Response_P(PSTR("{\"" D_CMND_TUYA_MCU "\":["));
  bool added = false;
  for (uint8_t i = 0; i < MAX_TUYA_FUNCTIONS; i++) {
    if (Settings.tuya_fnid_map[i].fnid != 0) {
      if (added) {
        ResponseAppend_P(PSTR(","));
      }
      ResponseAppend_P(PSTR("{\"fnId\":%d,\"dpId\":%d}" ), Settings.tuya_fnid_map[i].fnid, Settings.tuya_fnid_map[i].dpid);
      added = true;
    }
  }
  ResponseAppend_P(PSTR("]}"));
}

/*********************************************************************************************\
 * Queue Functions
\*********************************************************************************************/

int TuyaQueueSize() {
  return ((Tuya.queue.tail < Tuya.queue.head) ? TUYA_TX_QUEUE_SIZE - (Tuya.queue.head - Tuya.queue.tail) : Tuya.queue.tail - Tuya.queue.head) + 1;
}

inline bool TuyaQueueEmpty() {
  return (Tuya.queue.tail < 0);
}

inline bool TuyaQueueFull() {
  return (!TuyaQueueEmpty() && TuyaQueueSize() == TUYA_TX_QUEUE_SIZE);
}

inline void TuyaQueueReset() {
  Tuya.queue.head = 0;
  Tuya.queue.tail = -1;
}

inline int TuyaQueuePrevPos(int pos) {
  return (pos > 0) ? pos - 1 : TUYA_TX_QUEUE_SIZE - 1;
}

inline int TuyaQueueNextPos(int pos) {
  return (pos < TUYA_TX_QUEUE_SIZE - 1) ? pos + 1 : 0;
}

bool TuyaQueuePushDP(uint8_t data_type, uint8_t dpid, uint32_t value, uint8_t comm_mode, uint8_t timeout) {
  if (TuyaQueueFull()) return false;

  int pos = TuyaQueueNextPos(Tuya.queue.tail);
  TUYA_TX_BLOCK& block = Tuya.queue.blocks[pos];
  switch (data_type)
  {
    case TUYA_TYPE_BOOL:
    case TUYA_TYPE_VALUE:
    case TUYA_TYPE_ENUM:
      block.value = value;
      break;
    // Unsupported cases - we cannot queue nothing or strings. return false and handle it immediately if possible
    default:
      return false;
  }
  block.cmd_type = TUYA_CMD_SET_DP;
  block.dpid = dpid;
  block.data_type = data_type;
  block.comm_mode = comm_mode;
  block.timeout = timeout;
  // don''t move marker until we actually do something
  Tuya.queue.tail = pos;

  return true;
}

// Can only queue payloads with a max length of 4B. Currently Tuya spec only has commands with max 2B.
// Note: It is possible to send DP data using this method but function command will be empty because of 4B limit and comm mode must not be required
bool TuyaQueuePushCmd(uint8_t cmd_type, uint8_t payload[], uint16_t payload_len, uint8_t comm_mode, uint8_t timeout) {
  if (TuyaQueueFull() || payload_len > sizeof(TUYA_TX_BLOCK().value)) return false;

  int pos = TuyaQueueNextPos(Tuya.queue.tail);
  TUYA_TX_BLOCK& block = Tuya.queue.blocks[pos];
  block.cmd_type = cmd_type;
  block.comm_mode = comm_mode;
  block.timeout = timeout;
  block.length = payload_len;
  memcpy(block.data, payload, payload_len);
  // don''t move marker until we actually do something
  Tuya.queue.tail = pos;

  return true;
}

bool TuyaQueuePushFrontCmd(uint8_t cmd_type, uint8_t payload[], uint16_t payload_len, uint8_t comm_mode, uint8_t timeout, bool overwrite = false) {
  if ((!overwrite && TuyaQueueFull()) || payload_len > sizeof(TUYA_TX_BLOCK().value)) return false;

  int pos = TuyaQueuePrevPos(Tuya.queue.head);
  TUYA_TX_BLOCK& block = Tuya.queue.blocks[pos];
  block.cmd_type = cmd_type;
  block.comm_mode = comm_mode;
  block.timeout = timeout;
  block.length = payload_len;
  memcpy(block.data, payload, payload_len);
  // don''t move marker until we actually do something
  if (Tuya.queue.tail < 0 || Tuya.queue.tail == pos) Tuya.queue.tail = TuyaQueuePrevPos(Tuya.queue.tail);
  Tuya.queue.head = pos;

  return true;
}

struct TUYA_TX_BLOCK* TuyaQueuePeek() {
  if (TuyaQueueEmpty()) return nullptr;

  return &(Tuya.queue.blocks[Tuya.queue.head]);
}

struct TUYA_TX_BLOCK* TuyaQueuePop() {
  if (TuyaQueueEmpty()) return nullptr;

  TUYA_TX_BLOCK* block = &(Tuya.queue.blocks[Tuya.queue.head]);

  if (Tuya.queue.head != Tuya.queue.tail)
    Tuya.queue.head = TuyaQueueNextPos(Tuya.queue.head);
  else
    TuyaQueueReset();

  return block;
}

/*********************************************************************************************\
 * Communicaton Helper Functions
\*********************************************************************************************/

inline uint TuyaEndianConvert16(char* value)
{
  return ((uint)value[0] << 8 | (uint)value[1]);
}

inline uint TuyaEndianConvert32(char* value)
{
  return ((uint)value[0] << 24 | (uint)value[1] << 16 | (uint)value[2] << 8 | (uint)value[3]);
}

// If no commMode specified use default for each data type
uint8_t TuyaCommDefaultTypeMode(uint8_t data_type) {
  if (Tuya.slave_mode)
  {
    // Slave (cooperative) mode defaults - Everything controlled by MCU or MCU does not support bulk tx
    switch (data_type)
    {
      case TUYA_TYPE_BOOL:
      case TUYA_TYPE_VALUE:
      case TUYA_TYPE_ENUM:
        return TUYA_COMM_REQUIRE_REPLY;
      case TUYA_TYPE_STRING:
        return TUYA_COMM_NO_REPLY;
      default:
        return TUYA_COMM_MODE_NONE;
    }
  }
  else
  {
    // Master mode defaults - WiFi module has buttons wired to it (Same as previous tuyamcu driver)
    switch (data_type)
    {
      case TUYA_TYPE_BOOL:
      case TUYA_TYPE_VALUE:
      case TUYA_TYPE_ENUM:
        return TUYA_COMM_NO_REPLY;
      case TUYA_TYPE_STRING:
        return TUYA_COMM_NO_REPLY;
      default:
        return TUYA_COMM_MODE_NONE;
    }
  }
}

uint8_t TuyaCommSettingsCommandResponse(uint8_t cmd_type) {
  if (cmd_type < TUYA_CMD_RESP_TABLE_SIZE)
    return TUYA_CMD_RESP_TABLE[cmd_type * 2];
  else
    return TUYA_CMD_INVALID;
}

uint8_t TuyaCommSettingsCommandMode(uint8_t cmd_type) {
  if (cmd_type < TUYA_CMD_RESP_TABLE_SIZE)
    return TUYA_CMD_RESP_TABLE[(cmd_type * 2) + 1];
  else
    return TUYA_COMM_MODE_NONE;
}

inline void TuyaCommResetTxState(uint8_t data_state = TUYA_COMM_DATA_STATUS_AVAILABLE) {
  Tuya.comm.data_status = data_state;
  Tuya.comm.last_tx.comm_mode = TUYA_COMM_MODE_NONE;
}

bool TuyaCommReceiveMatchDP(uint8_t data_type, uint8_t dpid) {
  bool status = true;
  // Are we expecting a reply?
  if (Tuya.comm.data_status == TUYA_COMM_DATA_STATUS_SENT) {
    switch (Tuya.comm.last_tx.comm_mode) {
      case TUYA_COMM_REQUIRE_REPLY:
        status = (Tuya.comm.last_tx.data_type == data_type && Tuya.comm.last_tx.dpid == dpid);
        break;
      case TUYA_COMM_IGNORE_REPLY:
        TuyaCommResetTxState();
        status = false;
        break;
    }
    // If ok, reset send status. Otherwise data is resent next cycle
    if (status)
      TuyaCommResetTxState();
  } else {
    // If we receive a dp packet that does not correspond to a send, mark it as a blind receive
    TuyaCommResetTxState(TUYA_COMM_DATA_STATUS_BLIND_RECEIVE);
  }

  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("MatchDP data status:%d, comm mode:%d, match:%d"), Tuya.comm.data_status, Tuya.comm.last_tx.comm_mode, status);

  return status;
}

inline bool TuyaCommSendAvailable() {
  return ((Tuya.comm.data_status != TUYA_COMM_DATA_STATUS_SENT || Tuya.comm.last_tx.comm_mode == TUYA_COMM_NO_REPLY) &&
    Tuya.comm.data_status != TUYA_COMM_DATA_STATUS_BLIND_RECEIVE);
}

inline bool TuyaCommPreSendAvailable() {
  return (Tuya.comm.pre_send_data == nullptr);
}

inline uint8_t TuyaCommLastSendStatus() {
  return Tuya.comm.data_status;
}

bool TuyaCommSendCommand(uint8_t cmd_type, uint8_t payload[], uint16_t payload_len, uint8_t comm_mode, uint8_t timeout) {
  TuyaSendCmd(cmd_type, payload, payload_len);
  Tuya.comm.data_status = TUYA_COMM_DATA_STATUS_SENT;
  Tuya.comm.last_tx.timeout = timeout;
  Tuya.comm.last_tx.comm_mode = comm_mode;
  Tuya.comm.last_tx.cmd_type = cmd_type;
  Tuya.comm.last_tx.length = payload_len;
  Tuya.comm.last_tx.data = nullptr;

  return true;
}

bool TuyaCommSendState(uint8_t data_type, uint8_t dpid, uint32_t value, uint8_t comm_mode, uint8_t timeout) {
  bool status = TuyaSendState(dpid, data_type, (uint8_t*)(&value));
  if (status) {
    Tuya.comm.data_status = TUYA_COMM_DATA_STATUS_SENT;
    Tuya.comm.last_tx.timeout = timeout;
    Tuya.comm.last_tx.data_type = data_type;
    Tuya.comm.last_tx.comm_mode = comm_mode;
    Tuya.comm.last_tx.dpid = dpid;
    Tuya.comm.last_tx.value = value;
  } else {
    Tuya.comm.data_status = TUYA_COMM_DATA_STATUS_FAILED;
  }
  return status;
}

bool TuyaCommSendString(uint8_t dpid, char* data, uint8_t comm_mode, uint8_t timeout) {
  TuyaSendString(dpid, data);
  Tuya.comm.data_status = TUYA_COMM_DATA_STATUS_SENT;
  Tuya.comm.last_tx.timeout = timeout;
  Tuya.comm.last_tx.data_type = TUYA_TYPE_STRING;
  Tuya.comm.last_tx.comm_mode = comm_mode;
  Tuya.comm.last_tx.dpid = dpid;
  Tuya.comm.last_tx.data = nullptr;

  return true;
}

bool TuyaCommWaitReceiveBeforeSend(struct TUYA_TX_BLOCK& block) {
  AddLog_P(LOG_LEVEL_DEBUG, "TYA: TuyaCommWaitReceiveBeforeSend");

  Tuya.comm.pre_send_data = &block;
  bool result = TuyaSerialInput((int)block.timeout * (int)ssleep / (int)TUYA_RX_WAIT_TIME);
  Tuya.comm.pre_send_data = nullptr;

  return result;
}

/*********************************************************************************************\
 * Internal Functions
\*********************************************************************************************/

inline uint8_t TuyaCalcTimeoutCycles(uint timeout_ms) {
  uint loops = (timeout_ms / (uint)ssleep) + 1; // Add an additional loop because 0 = unlimited
  return (loops < 256) ? (uint8_t)loops : 256;
}

bool TuyaProcessSendState(uint8_t data_type, uint8_t dpid, uint32_t value, uint8_t comm_mode, uint timeout) {
  if (TuyaCommSendAvailable() && TuyaMCUCurrentState() == TUYA_MCU_STATE_READY)
    return TuyaCommSendState(data_type, dpid, value, comm_mode, TuyaCalcTimeoutCycles(timeout));
  else {
    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Send not available. Add to queue. Type=%d,dpId=%d,value=%d"), data_type, dpid, value);

    return TuyaQueuePushDP(data_type, dpid, value, comm_mode, TuyaCalcTimeoutCycles(timeout));
  }
}

bool TuyaProcessSendString(uint8_t dpid, char* data, uint8_t comm_mode, uint timeout) {
  if (TuyaMCUCurrentState() != TUYA_MCU_STATE_READY)
    return false;
  
  if (!TuyaCommSendAvailable()) {
    if (!TuyaCommPreSendAvailable())  // Check in case there is a recursive loop around (rules, backlog, etc)
      return false;
    TUYA_TX_BLOCK block = { .comm_mode = comm_mode, .timeout = TuyaCalcTimeoutCycles(timeout), .cmd_type = TUYA_CMD_SET_DP,
      {{ .data_type = TUYA_TYPE_STRING, .dpid = dpid }},
      { .data = ((uint8_t*)data) } };
    return TuyaCommWaitReceiveBeforeSend(block);
  }
  else
    return TuyaCommSendString(dpid, data, comm_mode, TuyaCalcTimeoutCycles(timeout));
}

bool TuyaProcessSendCommand(uint8_t cmd_type, uint8_t payload[], uint16_t payload_len, uint8_t comm_mode, uint timeout, bool frontOfQueue, bool overwrite) {
  if (cmd_type == TUYA_CMD_SET_DP || cmd_type > TUYA_CMD_QUERY_STATE)
    return false;

  if (TuyaCommSendAvailable())
    return TuyaCommSendCommand(cmd_type, payload, payload_len, comm_mode, TuyaCalcTimeoutCycles(timeout));
  else {
    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Send not available. Add to queue. Cmd=%d"), cmd_type);

    return (frontOfQueue) ? TuyaQueuePushFrontCmd(cmd_type, payload, payload_len, comm_mode, TuyaCalcTimeoutCycles(timeout), overwrite) :
      TuyaQueuePushCmd(cmd_type, payload, payload_len, comm_mode, TuyaCalcTimeoutCycles(timeout));
  } 
}

inline bool TuyaProcessSendCommandFirst(uint8_t cmd_type, uint8_t payload[], uint16_t payload_len, uint8_t comm_mode, uint timeout = TUYA_RX_TIMEOUT,
  bool overwrite = true)
{
  return TuyaProcessSendCommand(cmd_type, payload, payload_len, comm_mode, timeout, true, overwrite);
}

inline bool TuyaProcessSendCommand(uint8_t cmd_type, uint8_t payload[], uint16_t payload_len, uint8_t comm_mode, uint timeout = TUYA_RX_TIMEOUT) {
  return TuyaProcessSendCommand(cmd_type, payload, payload_len, comm_mode, timeout, false, false);
}

bool TuyaProcessSendState(uint8_t data_type, uint8_t dpid, uint32_t value) {
  uint8_t comm_mode = TuyaCommDefaultTypeMode(data_type);
  return (comm_mode == TUYA_COMM_MODE_NONE) ? false : TuyaProcessSendState(data_type, dpid, value, comm_mode);
}

bool TuyaProcessSendString(uint8_t dpid, char* data) {
  uint8_t comm_mode = TuyaCommDefaultTypeMode(TUYA_TYPE_STRING);
  return (comm_mode == TUYA_COMM_MODE_NONE) ? false : TuyaProcessSendString(dpid, data, comm_mode);
}

bool TuyaProcessSendCommandFirst(uint8_t cmd_type, uint8_t payload[] = nullptr, uint16_t payload_len = 0) {
  uint8_t comm_mode = TuyaCommSettingsCommandMode(cmd_type);
  return (comm_mode == TUYA_COMM_MODE_NONE) ? false : TuyaProcessSendCommandFirst(cmd_type, payload, payload_len, comm_mode);
}

bool TuyaProcessSendCommand(uint8_t cmd_type, uint8_t payload[] = nullptr, uint16_t payload_len = 0) {
  uint8_t comm_mode = TuyaCommSettingsCommandMode(cmd_type);
  return (comm_mode == TUYA_COMM_MODE_NONE) ? false : TuyaProcessSendCommand(cmd_type, payload, payload_len, comm_mode);
}

bool TuyaProcessPreSend() {   // Should only be called by serial input function
  TUYA_TX_BLOCK* block = Tuya.comm.pre_send_data;
  if (block == nullptr || !TuyaCommSendAvailable())
    return false;

  return TuyaCommSendString(block->dpid, (char*)(block->data), block->comm_mode, block->timeout);
}

bool TuyaProcessNormalPostSend() {  // Should only be called by TuyaProcessPostSend
  bool status = false;
  TUYA_TX_BLOCK* block;
  while (TuyaCommSendAvailable() && (block = TuyaQueuePop()) != nullptr) {
    // If block cannot be sent, skip and send next block
    switch (block->cmd_type) {
      case TUYA_CMD_SET_DP:
        status = TuyaCommSendState(block->data_type, block->dpid, block->value, block->comm_mode, block->timeout);
        break;
      default:
        status = TuyaCommSendCommand(block->cmd_type, block->data, block->length, block->comm_mode, block->timeout);
    }
    if (!status)
      continue;

    if (block->comm_mode == TUYA_COMM_NO_REPLY)
      delay(5); // Wait before sending next packet. Also resets watchdog
    else
      break;
  }

  return status;  // returns true if at least one block was sent
}

uint8_t TuyaMCUStateToCommand(uint8_t mcu_state) {
  switch (mcu_state) {
    case TUYA_MCU_STATE_HEARTBEAT:
      return TUYA_CMD_HEARTBEAT;
    case TUYA_MCU_STATE_QUERY:
      return TUYA_CMD_QUERY_PRODUCT;
    case TUYA_MCU_STATE_MODE:
      return TUYA_CMD_MCU_CONF;
    case TUYA_MCU_STATE_WIFI:
      return TUYA_CMD_WIFI_STATE;
    case TUYA_MCU_STATE_SYNC:
      return TUYA_CMD_QUERY_STATE;
  }
  return TUYA_CMD_INVALID;
}

uint8_t TuyaMCULastStage(uint8_t esp_mode) {
  switch (esp_mode) {
    case TUYA_MODE_NORMAL_POWER_INIT:
    case TUYA_MODE_LOW_POWER_INIT:
      return sizeof(TUYA_MODE_MCU_SEQ_INIT) - 1;
    case TUYA_MODE_NORMAL_POWER_SYNC:
    case TUYA_MODE_LOW_POWER_SYNC:
      return sizeof(TUYA_MODE_MCU_SEQ_SYNC) - 1;
  }
  return 0;
}

inline uint8_t TuyaMCULastStage() {
  return TuyaMCULastStage(Tuya.esp_mode);
}

uint8_t TuyaMCUGetStage(uint8_t esp_mode, uint8_t mcu_stage, int offset = 0) {
  int off_stage;
  if ((off_stage = (int)mcu_stage + offset) < 0)
    off_stage = 0;
  int last_stage = TuyaMCULastStage(esp_mode);
  return (off_stage <= last_stage) ? off_stage : last_stage;
}

void TuyaMCUSetStage(uint8_t esp_mode, uint8_t mcu_stage) {
  switch (esp_mode) {
    case TUYA_MODE_NORMAL_POWER_INIT:
    case TUYA_MODE_LOW_POWER_INIT:
    case TUYA_MODE_NORMAL_POWER_SYNC:
    case TUYA_MODE_LOW_POWER_SYNC:
      Tuya.esp_mode = esp_mode;
      Tuya.mcu_stage = TuyaMCUGetStage(esp_mode, mcu_stage);
      break;
    case TUYA_MODE_NORMAL_POWER_READY:
    case TUYA_MODE_LOW_POWER_READY:
      Tuya.esp_mode = esp_mode;
      Tuya.mcu_stage = 0;
  }
}

uint8_t TuyaMCUGetStateInternal(uint8_t esp_mode, uint8_t mcu_stage)  // Should not be called directly
{
  switch (esp_mode) {
    case TUYA_MODE_NORMAL_POWER_INIT:
    case TUYA_MODE_LOW_POWER_INIT:
      return TUYA_MODE_MCU_SEQ_INIT[mcu_stage];
    case TUYA_MODE_NORMAL_POWER_SYNC:
    case TUYA_MODE_LOW_POWER_SYNC:
      return TUYA_MODE_MCU_SEQ_SYNC[mcu_stage];
    case TUYA_MODE_NORMAL_POWER_READY:
    case TUYA_MODE_LOW_POWER_READY:
      return TUYA_MCU_STATE_READY;
  }
  return TUYA_MCU_STATE_NONE;
}

inline uint8_t TuyaMCUGetState(uint8_t esp_mode, uint8_t mcu_stage) {
  return TuyaMCUGetStateInternal(esp_mode, TuyaMCUGetStage(esp_mode, mcu_stage));
}

inline uint8_t TuyaMCUCurrentState() {
  return TuyaMCUGetStateInternal(Tuya.esp_mode, Tuya.mcu_stage);
}

inline uint8_t TuyaMCUFirstStage() {
  return 0;
}

inline uint8_t TuyaMCUNextStage() {
  return TuyaMCUGetStage(Tuya.esp_mode, Tuya.mcu_stage, 1);
}

/*
inline uint8_t TuyaMCUFirstState(uint8_t esp_mode) {
  return TuyaMCUGetState(esp_mode, 0);
}

inline uint8_t TuyaMCUNextState(uint8_t esp_mode, uint8_t mcu_stage) {
  return TuyaMCUGetState(esp_mode, mcu_stage, 1);
}
*/

bool TuyaProcessInitPostSendQueue() {  // Should only be called by TuyaProcessInitPostSend
  bool status = false;
  TUYA_TX_BLOCK* block;

  while (TuyaCommSendAvailable() && (block = TuyaQueuePeek()) != nullptr && block->cmd_type == TuyaMCUStateToCommand(TuyaMCUCurrentState())) {
    // If block cannot be sent, skip and send next block
    switch (block->cmd_type) {
      case TUYA_CMD_SET_DP:
        AddLog_P(LOG_LEVEL_DEBUG, "TuyaProcessInitPostSendQueue send state");

        status = TuyaCommSendState(block->data_type, block->dpid, block->value, block->comm_mode, block->timeout);
        break;
      default:
        AddLog_P(LOG_LEVEL_DEBUG, "TuyaProcessInitPostSendQueue send command");

        status = TuyaCommSendCommand(block->cmd_type, block->data, block->length, block->comm_mode, block->timeout);
    }
    // Pop sent block
    TuyaQueuePop();
    // If send unsuccessful continue to next block
    if (!status)
      continue;

    if (block->comm_mode == TUYA_COMM_NO_REPLY)
      delay(5); // Wait before sending next packet. Also resets watchdog
    else
      break;
  }

  return status;  // returns true if at least one block was sent
}

bool TuyaProcessInitNeedsRetryLastSend() {
  switch (TuyaCommLastSendStatus()) {
    // If last command failed or timed out retry again
    case TUYA_COMM_DATA_STATUS_TIMEOUT:
    case TUYA_COMM_DATA_STATUS_FAILED:
      AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Retry sending command"));
      return TuyaNormalPowerModeSendPacketInternal(TuyaMCUStateToCommand(TuyaMCUCurrentState()));
  }
  return false;
}

bool TuyaProcessInitPostSend() {
  if (!TuyaCommSendAvailable())
    return false;

  if (TuyaProcessInitPostSendQueue())       // Send any queued commands first
    return true;

  if (TuyaProcessInitNeedsRetryLastSend())  // If last send failed, retry again
    return false;

  uint8_t last_stage = TuyaMCULastStage();
  uint8_t next_stage = TuyaMCUNextStage();

  uint8_t next_cmd;
  for (next_cmd = TuyaCommSettingsCommandResponse(TuyaMCUStateToCommand(TuyaMCUGetState(Tuya.esp_mode, next_stage)));
    next_cmd == TUYA_CMD_INVALID && next_stage < last_stage;
    next_cmd = TuyaCommSettingsCommandResponse(TuyaMCUStateToCommand(TuyaMCUGetState(Tuya.esp_mode, ++next_stage)))) { }

  AddLog_P2(LOG_LEVEL_DEBUG, "TuyaProcessInitPostSend - mode:%d, next stage:%d, next state:%d", Tuya.esp_mode, next_stage, TuyaMCUGetState(Tuya.esp_mode, next_stage));

  if (next_stage >= last_stage)
    // If MCU ready, run completion routine
    TuyaSetReadyMCU();
  else if (TuyaNormalPowerModeSendPacketInternal(next_cmd)) {
    // If send successful update mcu state
    Tuya.mcu_stage = next_stage;
    return true;
  }
  else {
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Error! Could not send command to MCU"));
    return false;
  }
    
    // Otherwise retry sending last command
    // return TuyaNormalPowerModeSendPacketInternal(TuyaMCUStateToCommand(TuyaMCUCurrentState()));
}

bool TuyaProcessPostSend() {  // Should only be called during serial input sequence
  switch (Tuya.esp_mode) {
    case TUYA_MODE_NORMAL_POWER_READY:
    case TUYA_MODE_LOW_POWER_READY:
      return TuyaProcessNormalPostSend();
    default:
      return TuyaProcessInitPostSend();
  }
}

uint8_t TuyaProcessMatchDPReceivePacket(char* data, int length) {
  if (TuyaMCUCurrentState() != TUYA_MCU_STATE_READY) // If MCU not initialized all data received is considered internal
    return TUYA_COMM_DATA_INTERNAL;
  if (length < 6)   // Check to see if data is at least minimum size of any Tuya packet
    return TUYA_COMM_DATA_INCOMPLETE;
  TuyaValueDataFrame* frame = (TuyaValueDataFrame*)data;
  if (frame->command != TUYA_CMD_STATE)   // Check if command is DP response
    return TUYA_COMM_DATA_INTERNAL;
  uint data_length = TuyaEndianConvert16((char*)(frame->data_length));
  if (data_length < 5 || data_length + 7 > length)  // 5B is smallest size of dp data; 7B = header + checksum
    return TUYA_COMM_DATA_INCOMPLETE;
  uint function_length = TuyaEndianConvert16((char*)((frame->function).function_length));
  if (data_length != function_length + 4) // data length must match function length + function header
    return TUYA_COMM_DATA_INVALID;

  switch (frame->function.data_type) {
    case TUYA_TYPE_BOOL:
    case TUYA_TYPE_ENUM:
      if (function_length != 1)
        return TUYA_COMM_DATA_INVALID;
      break;

    case TUYA_TYPE_VALUE:
      if (function_length != 4)
        return TUYA_COMM_DATA_INVALID;
      break;
  
    case TUYA_TYPE_STRING:
      // String send and replies are arbitrary. No guarantee of length. Send and receive types must match (Cannot send string / receive enum)
      break;
  
    default:
      return TUYA_COMM_DATA_DP_NOMATCH;
  }

  return ((TuyaCommReceiveMatchDP(frame->function.data_type, frame->function.dpid)) ? TUYA_COMM_DATA_DP_MATCH : TUYA_COMM_DATA_DP_NOMATCH);
}

bool TuyaProcessMatchInternalReceivePacket(char* data, int length) {
  if (length < 6)   // Check to see if data is at least minimum size of any Tuya packet
    return false;
  
  if (TuyaMCUCurrentState() == TUYA_MCU_STATE_READY) {  // Do not match internal packets in run mode. Just reset send flags
    TuyaCommResetTxState();
    return true;
  }

  uint8_t src_cmd = TuyaMCUStateToCommand(TuyaMCUGetState(Tuya.esp_mode, Tuya.mcu_stage));
  if (src_cmd == TUYA_CMD_INVALID) {  // If we encounter an invalid source command mark it as received
    TuyaCommResetTxState();
    return false;
  }

  uint8_t dst_cmd = TuyaCommSettingsCommandResponse(src_cmd);
  uint8_t dst_mode = TuyaCommSettingsCommandMode(src_cmd);
  TuyaValueDataFrame* frame = (TuyaValueDataFrame*)data;
  switch (dst_mode) {
    case TUYA_COMM_REQUIRE_REPLY:
      if (frame->command == dst_cmd) {
        TuyaCommResetTxState();
        return true;
      }
      break;
    case TUYA_COMM_IGNORE_REPLY:
      TuyaCommResetTxState();
      break;
    default:
      TuyaCommResetTxState();
      return true;
  }
  return false;
}




void TuyaAddMcuFunc(uint8_t fnId, uint8_t dpId) {
  bool added = false;

  if (fnId == 0 || dpId == 0) { // Delete entry
    for (uint8_t i = 0; i < MAX_TUYA_FUNCTIONS; i++) {
      if ((dpId > 0 && Settings.tuya_fnid_map[i].dpid == dpId) || (fnId > TUYA_MCU_FUNC_NONE && Settings.tuya_fnid_map[i].fnid == fnId)) {
        Settings.tuya_fnid_map[i].fnid = TUYA_MCU_FUNC_NONE;
        Settings.tuya_fnid_map[i].dpid = 0;
        break;
      }
    }
  } else { // Add or update
    for (uint8_t i = 0; i < MAX_TUYA_FUNCTIONS; i++) {
      if (Settings.tuya_fnid_map[i].dpid == dpId || Settings.tuya_fnid_map[i].dpid == 0 || Settings.tuya_fnid_map[i].fnid == fnId || Settings.tuya_fnid_map[i].fnid == 0) {
        if (!added) { // Update entry if exisiting entry or add
          Settings.tuya_fnid_map[i].fnid = fnId;
          Settings.tuya_fnid_map[i].dpid = dpId;
          added = true;
        } else if (Settings.tuya_fnid_map[i].dpid == dpId || Settings.tuya_fnid_map[i].fnid == fnId) { // Remove existing entry if added to empty place
          Settings.tuya_fnid_map[i].fnid = TUYA_MCU_FUNC_NONE;
          Settings.tuya_fnid_map[i].dpid = 0;
        }
      }
    }
  }
  UpdateDevices();
}

void UpdateDevices() {
  for (uint8_t i = 0; i < MAX_TUYA_FUNCTIONS; i++) {
    uint8_t fnId = Settings.tuya_fnid_map[i].fnid;
    if (fnId > TUYA_MCU_FUNC_NONE && Settings.tuya_fnid_map[i].dpid > 0) {

      if (fnId >= TUYA_MCU_FUNC_REL1 && fnId <= TUYA_MCU_FUNC_REL8) { //Relay
        bitClear(rel_inverted, fnId - TUYA_MCU_FUNC_REL1);
      } else if (fnId >= TUYA_MCU_FUNC_REL1_INV && fnId <= TUYA_MCU_FUNC_REL8_INV) { // Inverted Relay
        bitSet(rel_inverted, fnId - TUYA_MCU_FUNC_REL1_INV);
      }

    }
  }
}

inline bool TuyaFuncIdValid(uint8_t fnId) {
  return (fnId >= TUYA_MCU_FUNC_SWT1 && fnId <= TUYA_MCU_FUNC_SWT4) ||
  (fnId >= TUYA_MCU_FUNC_REL1 && fnId <= TUYA_MCU_FUNC_REL8) ||
    fnId == TUYA_MCU_FUNC_DIMMER ||
    (fnId >= TUYA_MCU_FUNC_POWER && fnId <= TUYA_MCU_FUNC_VOLTAGE) ||
    (fnId >= TUYA_MCU_FUNC_REL1_INV && fnId <= TUYA_MCU_FUNC_REL8_INV) ||
    (fnId == TUYA_MCU_FUNC_LOWPOWER_MODE);
}

uint8_t TuyaGetFuncId(uint8_t dpid) {
  for (uint8_t i = 0; i < MAX_TUYA_FUNCTIONS; i++) {
    if (Settings.tuya_fnid_map[i].dpid == dpid) {
      return Settings.tuya_fnid_map[i].fnid;
    }
  }
  return TUYA_MCU_FUNC_NONE;
}

uint8_t TuyaGetDpId(uint8_t fnId) {
  for (uint8_t i = 0; i < MAX_TUYA_FUNCTIONS; i++) {
    if (Settings.tuya_fnid_map[i].fnid == fnId) {
      return Settings.tuya_fnid_map[i].dpid;
    }
  }
  return 0;
}

// Function does the actual sending of data
void TuyaSendCmd(uint8_t cmd, uint8_t payload[], uint16_t payload_len)
{
  uint8_t checksum = (0xFF + cmd + (payload_len >> 8) + (payload_len & 0xFF));
  TuyaSerial->write(0x55);                  // Tuya header 55AA
  TuyaSerial->write(0xAA);
  TuyaSerial->write((uint8_t)0x00);         // version 00
  TuyaSerial->write(cmd);                   // Tuya command
  TuyaSerial->write(payload_len >> 8);      // following data length (Hi)
  TuyaSerial->write(payload_len & 0xFF);    // following data length (Lo)
  snprintf_P(log_data, sizeof(log_data), PSTR("TYA: Serial TX bytes \"55aa00%02x%02x%02x"), cmd, payload_len >> 8, payload_len & 0xFF);
  for (uint32_t i = 0; i < payload_len; ++i) {
    TuyaSerial->write(payload[i]);
    checksum += payload[i];
    snprintf_P(log_data, sizeof(log_data), PSTR("%s%02x"), log_data, payload[i]);
  }
  TuyaSerial->write(checksum);
  TuyaSerial->flush();
  snprintf_P(log_data, sizeof(log_data), PSTR("%s%02x\""), log_data, checksum);
  AddLog(LOG_LEVEL_DEBUG);
}

// Used to send data that is not a string
bool TuyaSendState(uint8_t id, uint8_t type, uint8_t* value)
{
  uint16_t payload_len = 4;
  uint8_t payload_buffer[8];
  payload_buffer[0] = id;
  payload_buffer[1] = type;
  switch (type) {
    case TUYA_TYPE_BOOL:
    case TUYA_TYPE_ENUM:
      payload_len += 1;
      payload_buffer[2] = 0x00;
      payload_buffer[3] = 0x01;
      payload_buffer[4] = value[0];
      break;
    case TUYA_TYPE_VALUE:
      payload_len += 4;
      payload_buffer[2] = 0x00;
      payload_buffer[3] = 0x04;
      payload_buffer[4] = value[3];
      payload_buffer[5] = value[2];
      payload_buffer[6] = value[1];
      payload_buffer[7] = value[0];
      break;
    default:  // Handle invalid cases by return without sending
      return false;
  }
  TuyaSendCmd(TUYA_CMD_SET_DP, payload_buffer, payload_len);

  return true;
}

/*
void TuyaSendBool(uint8_t id, bool value)
{
  TuyaSendState(id, TUYA_TYPE_BOOL, (uint8_t*)&value);
}

void TuyaSendValue(uint8_t id, uint32_t value)
{
  TuyaSendState(id, TUYA_TYPE_VALUE, (uint8_t*)(&value));
}

void TuyaSendEnum(uint8_t id, uint32_t value)
{
  TuyaSendState(id, TUYA_TYPE_ENUM, (uint8_t*)(&value));
}
*/

// Used to send string data
void TuyaSendString(uint8_t id, char data[]) {

  uint16_t len = strlen(data);
  uint16_t payload_len = 4 + len;
  uint8_t payload_buffer[payload_len];
  payload_buffer[0] = id;
  payload_buffer[1] = TUYA_TYPE_STRING;
  payload_buffer[2] = len >> 8;
  payload_buffer[3] = len & 0xFF;

  for (uint16_t i = 0; i < len; i++) {
    payload_buffer[4+i] = data[i];
  }

  TuyaSendCmd(TUYA_CMD_SET_DP, payload_buffer, payload_len);
}

bool TuyaStartModeChange(uint8_t esp_mode) {
  if (esp_mode >= sizeof(TuyaOperatingMode))
    return false;

  uint8_t mcu_cmd;
  int last_stage = TuyaMCULastStage();
  uint8_t mcu_stage = TuyaMCUFirstStage();
  // Check mcu state for availablity
  for (mcu_cmd = TuyaCommSettingsCommandResponse(TuyaMCUStateToCommand(TuyaMCUGetState(esp_mode, mcu_stage)));
    mcu_cmd == TUYA_CMD_INVALID && mcu_stage < last_stage;
    mcu_cmd = TuyaCommSettingsCommandResponse(TuyaMCUStateToCommand(TuyaMCUGetState(esp_mode, ++mcu_stage)))) { }
  if (mcu_stage < last_stage && TuyaProcessSendCommandFirst(mcu_cmd)) {
    AddLog_P2(LOG_LEVEL_DEBUG, "TYA: TuyaStartModeChange - mode:%d, stage:%d, sleep:%d", esp_mode, mcu_stage, TUYA_INIT_SLEEP);

    ssleep = TUYA_INIT_SLEEP;    // Temporarily change main loop sleep during mode change sequence
    Tuya.wifi_state = -2;       // Reset wifi state
    TuyaMCUSetStage(esp_mode, mcu_stage);
    return true;
  } else if (mcu_stage >= last_stage) {
    // Change mode to ready
    Tuya.esp_mode = (Tuya.esp_mode > TUYA_MODE_NORMAL_POWER_READY) ? TUYA_MODE_LOW_POWER_READY : TUYA_MODE_NORMAL_POWER_READY;

    AddLog_P2(LOG_LEVEL_DEBUG, "TYA: TuyaStartModeChange - mode:%d, stage:%d, sleep:%d", Tuya.esp_mode, Tuya.mcu_stage, Settings.sleep);

    ssleep = Settings.sleep;     // If mcu is ready reset sleep time

    return true;
  } else {    // Cannot send command return false
    return false;
  }
}

inline bool TuyaSetInitMCU() {
  return TuyaStartModeChange(TUYA_MODE_NORMAL_POWER_INIT);
}

inline bool TuyaSetSyncMCU() {
  return TuyaStartModeChange(TUYA_MODE_NORMAL_POWER_SYNC);
}

inline bool TuyaSetReadyMCU() {
  return TuyaStartModeChange(TUYA_MODE_NORMAL_POWER_READY);
}

bool TuyaSetPower(void)
{
  bool status = false;

  uint8_t rpower = XdrvMailbox.index;
  int16_t source = XdrvMailbox.payload;

  uint8_t dpid = TuyaGetDpId(TUYA_MCU_FUNC_REL1 + active_device - 1);
  if (dpid == 0) dpid = TuyaGetDpId(TUYA_MCU_FUNC_REL1_INV + active_device - 1);

  if (source != SRC_SWITCH && TuyaSerial) {  // ignore to prevent loop from pushing state from faceplate interaction

    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: TuyaSetPower - queue MCU new power state:%u"), rpower);

    //TuyaSendBool(dpid, bitRead(rpower, active_device-1) ^ bitRead(rel_inverted, active_device-1));
    //delay(20); // Hack when power is off and dimmer is set then both commands go too soon to Serial out.
    //status = true;

    status = TuyaProcessSendState(TUYA_TYPE_BOOL, dpid, bitRead(rpower, active_device-1) ^ bitRead(rel_inverted, active_device-1));
  }
  return status;
}

bool TuyaSetChannels(void)
{
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: TuyaSetChannels - grpflg:%s"), (XdrvMailbox.grpflg)?"Y":"N");
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: TuyaSetChannels - usridx:%s"), (XdrvMailbox.usridx)?"Y":"N");
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: TuyaSetChannels - command_code:%u"), XdrvMailbox.command_code);
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: TuyaSetChannels - index:%u"), XdrvMailbox.index);
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: TuyaSetChannels - payload:%d"), XdrvMailbox.payload);
  //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: TuyaSetChannels - topic:%s"), XdrvMailbox.topic);
  //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: TuyaSetChannels - command:%s"), XdrvMailbox.command);

  LightSerialDuty(((uint8_t*)XdrvMailbox.data)[0]);
  //delay(20); // Hack when power is off and dimmer is set then both commands go too soon to Serial out.
  return true;
}

void LightSerialDuty(uint16_t duty)
{
  uint8_t dpid = TuyaGetDpId(TUYA_MCU_FUNC_DIMMER);
  if (duty > 0 && !Tuya.ignore_dim && TuyaSerial && dpid > 0) {
    duty = changeUIntScale(duty, 0, 255, 0, Settings.dimmer_hw_max);
    if (duty < Settings.dimmer_hw_min) { duty = Settings.dimmer_hw_min; }  // dimming acts odd below 25(10%) - this mirrors the threshold set on the faceplate itself
    if (Tuya.new_dim != duty) {
      AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Send dim value=%d (id=%d)"), duty, dpid);
      Tuya.ignore_dimmer_cmd_timeout = millis() + 250; // Ignore serial received dim commands for the next 250ms

      //TuyaSendBool(dpid, bitRead(1, active_device-1) ^ bitRead(rel_inverted, active_device-1));
      //delay(250);

      //delay(50);
      //TuyaSendValue(dpid, Tuya.new_dim);
      //delay(50);

      //TuyaSendValue(dpid, duty);

      // Send dimmer value first
      TuyaProcessSendState(TUYA_TYPE_VALUE, dpid, duty);
      // Send power state second
      if (Light.old_power != Light.power) {
        uint8_t power_dpid = TuyaGetDpId(TUYA_MCU_FUNC_REL1 + active_device - 1);
        if (power_dpid == 0) power_dpid = TuyaGetDpId(TUYA_MCU_FUNC_REL1_INV + active_device - 1);
        TuyaProcessSendState(TUYA_TYPE_BOOL, power_dpid, bitRead(1, active_device-1) ^ bitRead(rel_inverted, active_device-1));
      }
    }
  } else if (dpid > 0) {
    Tuya.ignore_dim = false;  // reset flag
    duty = changeUIntScale(duty, 0, 255, 0, Settings.dimmer_hw_max);
    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Send dim skipped value=%d"), duty);  // due to 0 or already set
  } else {
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Cannot set dimmer. Dimmer Id unknown"));  //
  }
}

// Available as external call and used during init
void TuyaRequestState(void)
{
  if (TuyaSerial) {
    // Get current status of MCU
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Read MCU state"));
    TuyaProcessSendCommandFirst(TUYA_CMD_QUERY_STATE);
  }
}

void TuyaResetWifi(void)
{
  if (!Settings.flag.button_restrict) {  // SetOption1 - Control button multipress
    char scmnd[20];
    snprintf_P(scmnd, sizeof(scmnd), D_CMND_WIFICONFIG " %d", 2);
    ExecuteCommand(scmnd, SRC_BUTTON);
  }
}

void TuyaProcessStatePacket(void) {
  char scmnd[20];
  uint8_t dpidStart = 6;
  uint8_t fnId;
  uint16_t dpDataLen;

  while (dpidStart + 4 < Tuya.byte_counter) {
    dpDataLen = Tuya.buffer[dpidStart + 2] << 8 | Tuya.buffer[dpidStart + 3];
    fnId = TuyaGetFuncId(Tuya.buffer[dpidStart]);

    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: fnId=%d is set for dpId=%d"), fnId, Tuya.buffer[dpidStart]);
    // if (TuyaFuncIdValid(fnId)) {
      if (Tuya.buffer[dpidStart + 1] == 1) {  // Data Type 1

        if (fnId >= TUYA_MCU_FUNC_REL1 && fnId <= TUYA_MCU_FUNC_REL8) {
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: RX Relay-%d --> MCU State: %s Current State:%s"), fnId - TUYA_MCU_FUNC_REL1 + 1, Tuya.buffer[dpidStart + 4]?"On":"Off",bitRead(power, fnId - TUYA_MCU_FUNC_REL1)?"On":"Off");
/*
          if (Tuya.slave_mode) {
            AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Flag ignore dim value"));
            Tuya.ignore_dim = true;
          }
*/
          if ((power || Settings.light_dimmer > 0) && (Tuya.buffer[dpidStart + 4] != bitRead(power, fnId - TUYA_MCU_FUNC_REL1))) {
            ExecuteCommandPower(fnId - TUYA_MCU_FUNC_REL1 + 1, Tuya.buffer[dpidStart + 4], SRC_SWITCH);  // send SRC_SWITCH? to use as flag to prevent loop from inbound states from faceplate interaction
          }
        } else if (fnId >= TUYA_MCU_FUNC_REL1_INV && fnId <= TUYA_MCU_FUNC_REL8_INV) {
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: RX Relay-%d-Inverted --> MCU State: %s Current State:%s"), fnId - TUYA_MCU_FUNC_REL1_INV + 1, Tuya.buffer[dpidStart + 4]?"Off":"On",bitRead(power, fnId - TUYA_MCU_FUNC_REL1_INV) ^ 1?"Off":"On");
/*
          if (Tuya.slave_mode) {
            AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Flag ignore dim value"));
            Tuya.ignore_dim = true;
          }
*/
          if (Tuya.buffer[dpidStart + 4] != bitRead(power, fnId - TUYA_MCU_FUNC_REL1_INV) ^ 1) {
            ExecuteCommandPower(fnId - TUYA_MCU_FUNC_REL1_INV + 1, Tuya.buffer[dpidStart + 4] ^ 1, SRC_SWITCH);  // send SRC_SWITCH? to use as flag to prevent loop from inbound states from faceplate interaction
          }
        } else if (fnId >= TUYA_MCU_FUNC_SWT1 && fnId <= TUYA_MCU_FUNC_SWT4) {
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: RX Switch-%d --> MCU State: %d Current State:%d"),fnId - TUYA_MCU_FUNC_SWT1 + 1,Tuya.buffer[dpidStart + 4], SwitchGetVirtual(fnId - TUYA_MCU_FUNC_SWT1));
/*
          if (Tuya.slave_mode) {
            AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Flag ignore dim value"));
            Tuya.ignore_dim = true;
          }
*/
          if (SwitchGetVirtual(fnId - TUYA_MCU_FUNC_SWT1) != Tuya.buffer[dpidStart + 4]) {
            SwitchSetVirtual(fnId - TUYA_MCU_FUNC_SWT1, Tuya.buffer[dpidStart + 4]);
            SwitchHandler(1);
          }
        }

      }
      else if (Tuya.buffer[dpidStart + 1] == 2) {  // Data Type 2
        bool tuya_energy_enabled = (XNRG_32 == energy_flg);
        uint16_t packetValue = Tuya.buffer[dpidStart + 6] << 8 | Tuya.buffer[dpidStart + 7];
        // Dimmer function
        if (fnId == TUYA_MCU_FUNC_DIMMER) {
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: RX Dim State=%d"), packetValue);
          Tuya.new_dim = changeUIntScale(packetValue, 0, Settings.dimmer_hw_max, 0, 100);
          if (Tuya.ignore_dimmer_cmd_timeout < millis()) {
            if ((power || Settings.flag3.tuya_apply_o20) &&  // SetOption54 - Apply SetOption20 settings to Tuya device
                (Tuya.new_dim > 0) && (abs(Tuya.new_dim - Settings.light_dimmer) > 1)) {
              Tuya.ignore_dim = true;

              snprintf_P(scmnd, sizeof(scmnd), PSTR(D_CMND_DIMMER "3 %d"), Tuya.new_dim );

              AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Send cmd:%s"), scmnd);

              ExecuteCommand(scmnd, SRC_SWITCH);
            }
          }
        }
        // Energy monitor function
  #ifdef USE_ENERGY_SENSOR
        else if (tuya_energy_enabled && fnId == TUYA_MCU_FUNC_VOLTAGE) {
          Energy.voltage[0] = (float)packetValue / 10;
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Rx ID=%d Voltage=%d"), Tuya.buffer[dpidStart], packetValue);
        } else if (tuya_energy_enabled && fnId == TUYA_MCU_FUNC_CURRENT) {
          Energy.current[0] = (float)packetValue / 1000;
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Rx ID=%d Current=%d"), Tuya.buffer[dpidStart], packetValue);
        } else if (tuya_energy_enabled && fnId == TUYA_MCU_FUNC_POWER) {
          Energy.active_power[0] = (float)packetValue / 10;
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Rx ID=%d Active_Power=%d"), Tuya.buffer[dpidStart], packetValue);

          if (Tuya.lastPowerCheckTime != 0 && Energy.active_power[0] > 0) {
            Energy.kWhtoday += (float)Energy.active_power[0] * (Rtc.utc_time - Tuya.lastPowerCheckTime) / 36;
            EnergyUpdateToday();
          }
          Tuya.lastPowerCheckTime = Rtc.utc_time;
        }
  #endif // USE_ENERGY_SENSOR

      }
    // } else {
    //   AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Unknown FnId=%s for dpId=%s"), fnId, Tuya.buffer[6]);
    dpidStart += dpDataLen + 4;
  }
}

bool TuyaLowPowerModePacketProcess(void) {
  switch (Tuya.buffer[3]) {
    case TUYA_CMD_QUERY_PRODUCT:
      TuyaHandleProductInfoPacket();
      TuyaSetWifiLed();
      break;

    case TUYA_LOW_POWER_CMD_STATE:
      TuyaProcessStatePacket();
      Tuya.send_success_next_second = true;
      break;
  }

  return true;
}

void TuyaHandleProductInfoPacket(void) {
  uint16_t dataLength = Tuya.buffer[4] << 8 | Tuya.buffer[5];
  char *data = &Tuya.buffer[6];
  AddLog_P2(LOG_LEVEL_INFO, PSTR("TYA: MCU Product ID: %.*s"), dataLength, data);
}

// Low power delayed response to MCU command received
void TuyaSendLowPowerSuccessIfNeeded(void) {
  uint8_t success = 1;

  if (Tuya.send_success_next_second) {
    TuyaSendCmd(TUYA_LOW_POWER_CMD_STATE, &success, 1);
    Tuya.send_success_next_second = false;
  }
}

// return true if data is or needs to be sent, false if data should not be sent
// note: does not send false by default. any state that does not need data sent should conditionally return false
bool TuyaNormalPowerModeSendPacketInternal(uint8_t next_cmd) {

  AddLog_P2(LOG_LEVEL_DEBUG, "TuyaNormalPowerModeSendPacketInternal - next cmd:%d", next_cmd);

  switch (TuyaMCUCurrentState()) {
    case TUYA_MCU_STATE_HEARTBEAT:  // Stage 1 (init) or 1 (sync) 
      break;  // Send next default command

    case TUYA_MCU_STATE_QUERY:      // Stage 2 (init)
      break;  // Send next default command

    case TUYA_MCU_STATE_MODE:       // Stage 3 (init)
      TuyaSetWifiLed();
      return true;  // data already sent

    case TUYA_MCU_STATE_WIFI:       // Stage 4 (init) or 2 (sync)
      TuyaRequestState();
      return true; // data already sent

    case TUYA_MCU_STATE_SYNC:       // Stage 5 (init) or 3 (sync)
      break;  // Send next default command
  }
  // send default command
  return TuyaProcessSendCommandFirst(next_cmd);
}

bool TuyaNormalPowerModeTryHandleOtherPacket(uint8_t cmd) {
  switch (cmd) {
    // Handle wifi reset requests
    case TUYA_CMD_WIFI_RESET:
    case TUYA_CMD_WIFI_SELECT:
      AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: RX WiFi Reset"));
      TuyaResetWifi();
      break;

    // Handle wifi led status
    case TUYA_CMD_WIFI_STATE:
      AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: RX WiFi LED set ACK"));
      Tuya.wifi_state = TuyaGetTuyaWifiState();
      break;

    default:
      AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: RX unknown command"));
      return false;
  }

  return true;
}

bool TuyaNormalPowerModePacketProcessInternal() {


  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Internal packet type=%d"), Tuya.buffer[3]);


  switch (Tuya.buffer[3]) {
    case TUYA_CMD_HEARTBEAT:      // Stage 1 (init) or 3 (sync) 
      if (Tuya.buffer[6] == 0) {
        switch (Tuya.esp_mode) {
          case TUYA_MODE_NORMAL_POWER_READY:
            // Error during run mode - restart sync process
            AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Detected MCU restart"));
            TuyaSetSyncMCU();
            break;
          case TUYA_MODE_NORMAL_POWER_INIT:
            // Stage 1 (init)
            AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Detected MCU start"));
            break;
          case TUYA_MODE_NORMAL_POWER_SYNC:
            // Stage 3 (sync)
            AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Begin MCU sync"));
            break;
        }
      } else {
        AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Heartbeat"));
      }
      break;

    case TUYA_CMD_QUERY_PRODUCT:  // Stage 2
      TuyaHandleProductInfoPacket();
      break;

    case TUYA_CMD_MCU_CONF:       // Stage 3 (init)
      switch (Tuya.esp_mode) {
        case TUYA_MODE_NORMAL_POWER_INIT:
        case TUYA_MODE_NORMAL_POWER_SYNC:
          AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: RX MCU configuration Mode=%d"), Tuya.buffer[5]);

          if (Tuya.buffer[5] == 2) { // Processing by ESP module mode
            uint8_t led1_gpio = Tuya.buffer[6];
            uint8_t key1_gpio = Tuya.buffer[7];
            bool key1_set = false;
            bool led1_set = false;
            for (uint32_t i = 0; i < ARRAY_SIZE(Settings.my_gp.io); i++) {
              if (Settings.my_gp.io[i] == AGPIO(GPIO_LED1)) led1_set = true;
              else if (Settings.my_gp.io[i] == AGPIO(GPIO_KEY1)) key1_set = true;
            }
            if (!Settings.my_gp.io[led1_gpio] && !led1_set) {
              Settings.my_gp.io[led1_gpio] = AGPIO(GPIO_LED1);
              restart_flag = 2;
            }
            if (!Settings.my_gp.io[key1_gpio] && !key1_set) {
              Settings.my_gp.io[key1_gpio] = AGPIO(GPIO_KEY1);
              restart_flag = 2;
            }
          } else {
            AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Detected MCU cooperative mode"));
          }
      }
      break;

    case TUYA_CMD_STATE:          // Stage 4
      switch (Tuya.esp_mode) {
        case TUYA_MODE_NORMAL_POWER_INIT:
        case TUYA_MODE_NORMAL_POWER_SYNC:
          if (TuyaMCUGetState(Tuya.esp_mode, Tuya.mcu_stage) == TUYA_MCU_STATE_SYNC) {
            // Don''t process the packet- Reports available dpid with initial values and will mess up any stored settings
            AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: MCU Initialized"));
          }
      }
      break;
    
    default:
      return TuyaNormalPowerModeTryHandleOtherPacket(Tuya.buffer[3]);
  }

  return true;
}

bool TuyaNormalPowerModePacketProcess(void)
{
  switch (Tuya.buffer[3]) {
    case TUYA_CMD_STATE:
      TuyaProcessStatePacket();
      break;
    default:
      return TuyaNormalPowerModeTryHandleOtherPacket(Tuya.buffer[3]);
  }

  return true;
}

void printTuyaBufferToMqttBuffer()
{
  char hex_char[(Tuya.byte_counter * 2) + 2];
  //      uint16_t len = Tuya.buffer[4] << 8 | Tuya.buffer[5];
  uint16_t len = Tuya.data_len;
  Response_P(PSTR("{\"" D_JSON_TUYA_MCU_RECEIVED "\":{\"Data\":\"%s\",\"Cmnd\":%d"), ToHex_P((unsigned char *)Tuya.buffer, Tuya.byte_counter, hex_char, sizeof(hex_char)), Tuya.buffer[3]);

  if (len > 0) {
    ResponseAppend_P(PSTR(",\"CmndData\":\"%s\""), ToHex_P((unsigned char *)&Tuya.buffer[6], len, hex_char, sizeof(hex_char)));
    if (TUYA_CMD_STATE == Tuya.buffer[3]) {
      //55 AA 03 07 00 0D 01 04 00 01 02 02 02 00 04 00 00 00 1A 40
      // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19
      uint8_t dpidStart = 6;
      while (dpidStart + 4 < Tuya.byte_counter) {
        uint8_t dpId = Tuya.buffer[dpidStart];
        uint8_t dpDataType = Tuya.buffer[dpidStart + 1];
        uint16_t dpDataLen = Tuya.buffer[dpidStart + 2] << 8 | Tuya.buffer[dpidStart + 3];
        const unsigned char *dpData = (unsigned char *)&Tuya.buffer[dpidStart + 4];
        const char *dpHexData = ToHex_P(dpData, dpDataLen, hex_char, sizeof(hex_char));

        if (TUYA_CMD_STATE == Tuya.buffer[3]) {
          ResponseAppend_P(PSTR(",\"DpType%uId%u\":"), dpDataType, dpId);
          if (TUYA_TYPE_BOOL == dpDataType && dpDataLen == 1) {
            ResponseAppend_P(PSTR("%u"), dpData[0]);
          } else if (TUYA_TYPE_VALUE == dpDataType && dpDataLen == 4) {
            uint32_t dpValue = (uint32_t)dpData[0] << 24 | (uint32_t)dpData[1] << 16 | (uint32_t)dpData[2] << 8 | (uint32_t)dpData[3] << 0;
            ResponseAppend_P(PSTR("%u"), dpValue);
          } else if (TUYA_TYPE_STRING == dpDataType) {
            ResponseAppend_P(PSTR("\"%.*s\""), dpDataLen, dpData);
          } else if (TUYA_TYPE_ENUM == dpDataType && dpDataLen == 1) {
            ResponseAppend_P(PSTR("%u"), dpData[0]);
          } else {
            ResponseAppend_P(PSTR("\"0x%s\""), dpHexData);
          }
        }

        ResponseAppend_P(PSTR(",\"%d\":{\"DpId\":%d,\"DpIdType\":%d,\"DpIdData\":\"%s\""), dpId, dpId, dpDataType, dpHexData);
        if (TUYA_TYPE_STRING == dpDataType) {
          ResponseAppend_P(PSTR(",\"Type3Data\":\"%.*s\""), dpDataLen, dpData);
        }
        ResponseAppend_P(PSTR("}"));
        dpidStart += dpDataLen + 4;
      }
    }
  }

  ResponseAppend_P(PSTR("}}"));
}

/*********************************************************************************************\
 * API Functions
\*********************************************************************************************/

bool TuyaModuleSelected(void)
{
  if (!PinUsed(GPIO_TUYA_RX) || !PinUsed(GPIO_TUYA_TX)) {  // fallback to hardware-serial if not explicitly selected
    SetPin(1, AGPIO(GPIO_TUYA_TX));
    SetPin(3, AGPIO(GPIO_TUYA_RX));
    Settings.my_gp.io[1] = AGPIO(GPIO_TUYA_TX);
    Settings.my_gp.io[3] = AGPIO(GPIO_TUYA_RX);
    restart_flag = 2;
  }

  if (TuyaGetDpId(TUYA_MCU_FUNC_DIMMER) == 0 && TUYA_DIMMER_ID > 0) {
    TuyaAddMcuFunc(TUYA_MCU_FUNC_DIMMER, TUYA_DIMMER_ID);
  }

  bool relaySet = false;

  for (uint8_t i = 0 ; i < MAX_TUYA_FUNCTIONS; i++) {
    if ((Settings.tuya_fnid_map[i].fnid >= TUYA_MCU_FUNC_REL1 && Settings.tuya_fnid_map[i].fnid <= TUYA_MCU_FUNC_REL8 ) ||
    (Settings.tuya_fnid_map[i].fnid >= TUYA_MCU_FUNC_REL1_INV && Settings.tuya_fnid_map[i].fnid <= TUYA_MCU_FUNC_REL8_INV )) {
      relaySet = true;
      devices_present++;
    }
  }

  if (!relaySet) {
    TuyaAddMcuFunc(TUYA_MCU_FUNC_REL1, 1);
    devices_present++;
    SettingsSaveAll();
  }

  if (TuyaGetDpId(TUYA_MCU_FUNC_DIMMER) != 0) {
    light_type = LT_SERIAL1;
  } else {
    light_type = LT_BASIC;
  }

  if (TuyaGetDpId(TUYA_MCU_FUNC_LOWPOWER_MODE) != 0) {
    Tuya.low_power_mode = true;
    Settings.flag3.fast_power_cycle_disable = true;  // SetOption65 - Disable fast power cycle detection for device reset
  }

  UpdateDevices();
  return true;
}

void TuyaInit(void)
{
  int baudrate = 9600;
  if (Settings.flag4.tuyamcu_baudrate) { baudrate = 115200; }  // SetOption97 - Set Baud rate for TuyaMCU serial communication (0 = 9600 or 1 = 115200)

  Tuya.buffer = (char*)(malloc(TUYA_BUFFER_SIZE));
  Tuya.queue.blocks = (TUYA_TX_BLOCK*)(malloc(TUYA_TX_QUEUE_SIZE * sizeof(TUYA_TX_BLOCK)));
  if (Tuya.buffer != nullptr && Tuya.queue.blocks != nullptr) {
    TuyaQueueReset();
    TuyaSerial = new TasmotaSerial(Pin(GPIO_TUYA_RX), Pin(GPIO_TUYA_TX), 2);
    if (TuyaSerial->begin(baudrate)) {
      if (TuyaSerial->hardwareSerial()) { ClaimSerial(); }

      // Flush MCU buffers
      for (int i=0; i<8; i++)
        TuyaSerial->write(0x00);
      TuyaSerial->flush();
      delay(5);

      AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Request MCU configuration at %d baud rate"), baudrate);

      // Get MCU Configuration
//      TuyaSendCmd(TUYA_CMD_QUERY_PRODUCT);
      TuyaSetInitMCU();
    }
  }
  else
  {
    // Log is pre-alloc. Should be able to write even without additional free memory
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Error! Recompile me! Insufficient memory."));
  }
  Tuya.heartbeat_timer = 0; // init heartbeat timer when dimmer init is done
}

void TuyaSerialResetBuffer()
{
  Tuya.byte_counter = 0;
  Tuya.cmd_status = 0;
  Tuya.cmd_checksum = 0;
  Tuya.data_len = 0;
}

bool TuyaSerialTryReadPacket()
{
  while (TuyaSerial->available()) {
    yield();
    uint8_t serial_in_byte = TuyaSerial->read();

    if (serial_in_byte == 0x55) {            // Start TUYA Packet
      Tuya.cmd_status = 1;
      Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
      Tuya.cmd_checksum += serial_in_byte;
    }
    else if (Tuya.cmd_status == 1 && serial_in_byte == 0xAA) { // Only packtes with header 0x55AA are valid
      Tuya.cmd_status = 2;

      Tuya.byte_counter = 0;
      Tuya.buffer[Tuya.byte_counter++] = 0x55;
      Tuya.buffer[Tuya.byte_counter++] = 0xAA;
      Tuya.cmd_checksum = 0xFF;
    }
    else if (Tuya.cmd_status == 2) {
      if (Tuya.byte_counter == 5) { // Get length of data
        Tuya.data_len = ((uint16_t)Tuya.buffer[4] << 8 | (uint16_t)serial_in_byte);
        if (Tuya.data_len + 7 > TUYA_BUFFER_SIZE) // Assume data larger than buffer is invalid and start over
        {
          TuyaSerialResetBuffer();
          continue;
        }
        Tuya.cmd_status = 3;
      }
      Tuya.cmd_checksum += serial_in_byte;
      Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
    }
    // If we have a complete packet, return true
    else if ((Tuya.cmd_status == 3) && (Tuya.byte_counter == (6 + Tuya.data_len)) && (Tuya.cmd_checksum == serial_in_byte)) { // Compare checksum and process packet
      Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;

      return true;
    }                               // read additional packets from TUYA
    else if (Tuya.byte_counter < TUYA_BUFFER_SIZE - 1) {  // add char to string if it still fits
      Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
      Tuya.cmd_checksum += serial_in_byte;
    } else {
      TuyaSerialResetBuffer();  // Too many chars for buffer and no match - reset buffer
    }
  }
  // return false, we don''t have a complete packet yet
  return false;
}

bool TuyaSerialInput(int waitTimeoutCount)  // -1 = no wait, 0 = unlimited wait, otherwise wait until timeout count + 1
{
  bool status = false;
  do {    
    for (int loop_n = 0; loop_n <= waitTimeoutCount && !TuyaSerial->available(); ) {
      delay(TUYA_RX_WAIT_TIME);
      if (waitTimeoutCount > 0) loop_n++;
    }
    waitTimeoutCount = -1;  // In case there is more than one packet don''t continue timeout
    yield();

    if (TuyaSerialTryReadPacket())
    {
      AddLog_P(LOG_LEVEL_DEBUG, PSTR("Packet received"));

      // printTuyaBufferToMqttBuffer();          // <-- Temp
      // AddLog_P(LOG_LEVEL_DEBUG, mqtt_data);   // <-- Temp

      uint8_t replyType = TuyaProcessMatchDPReceivePacket(Tuya.buffer, Tuya.byte_counter);
      switch (replyType) {
        case TUYA_COMM_DATA_DP_NOMATCH:
        case TUYA_COMM_DATA_INVALID:
        case TUYA_COMM_DATA_INCOMPLETE:
          TuyaSerialResetBuffer();
          continue;
      }

      // AddLog_P(LOG_LEVEL_DEBUG, PSTR("Checkpoint 1"));

      if (Tuya.low_power_mode) {  // Original input processing sequence - need more information about low power devices

      // AddLog_P(LOG_LEVEL_DEBUG, PSTR("Checkpoint 2a"));

        printTuyaBufferToMqttBuffer();
        if (Settings.flag3.tuya_serial_mqtt_publish) {  // SetOption66 - Enable TuyaMcuReceived messages over Mqtt
          MqttPublishPrefixTopic_P(RESULT_OR_TELE, PSTR(D_JSON_TUYA_MCU_RECEIVED));
        } else {
          AddLog_P(LOG_LEVEL_DEBUG, mqtt_data);
        }
        XdrvRulesProcess();
        status = TuyaLowPowerModePacketProcess();
      }
      else
      {
        
      // AddLog_P(LOG_LEVEL_DEBUG, PSTR("Checkpoint 2b"));

        // If no raw data to mqtt print to log
        if (!Settings.flag3.tuya_serial_mqtt_publish) {
          printTuyaBufferToMqttBuffer();
          AddLog_P(LOG_LEVEL_DEBUG, mqtt_data);
        }

        if (Tuya.esp_mode != TUYA_MODE_NORMAL_POWER_READY || replyType == TUYA_COMM_DATA_INTERNAL) {

      // AddLog_P(LOG_LEVEL_DEBUG, PSTR("Checkpoint 3a"));

          if (TuyaProcessMatchInternalReceivePacket(Tuya.buffer, Tuya.byte_counter)) {
            status = TuyaNormalPowerModePacketProcessInternal();
          }

        } else {

      // AddLog_P(LOG_LEVEL_DEBUG, PSTR("Checkpoint 4"));

          TuyaProcessPreSend();   // Send data that requires immediate attention

      // AddLog_P(LOG_LEVEL_DEBUG, PSTR("Checkpoint 5"));

          XdrvRulesProcess();

      // AddLog_P(LOG_LEVEL_DEBUG, PSTR("Checkpoint 6"));

          status = TuyaNormalPowerModePacketProcess();

        }

      // AddLog_P(LOG_LEVEL_DEBUG, PSTR("Checkpoint 7"));

        // Separate printTuyaBufferToMqttBuffer here because mqtt buffer could have other data from calls made before we print
        if (Settings.flag3.tuya_serial_mqtt_publish) {  // SetOption66 - Enable TuyaMcuReceived messages over Mqtt
          printTuyaBufferToMqttBuffer();
          MqttPublishPrefixTopic_P(RESULT_OR_TELE, PSTR(D_JSON_TUYA_MCU_RECEIVED));
        }
      }
      // Reset buffer when packet processing is complete
      TuyaSerialResetBuffer();
      // Set status to true if we process at least one data packet
    }
  } while (TuyaSerial->available());
  
/*
  if (Tuya.byte_counter > 0)
  {
    char hex_char[(Tuya.byte_counter * 3) + 2];
    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Serial data unknown (%s)"), ToHex_P((uint8_t*)Tuya.buffer, Tuya.byte_counter, hex_char, sizeof(hex_char), ' '));
  }
*/

  return status;
}

inline bool TuyaSerialInput() { TuyaSerialInput(-1); }

bool TuyaButtonPressed(void)
{
  if (!XdrvMailbox.index && ((PRESSED == XdrvMailbox.payload) && (NOT_PRESSED == Button.last_state[XdrvMailbox.index]))) {
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Reset GPIO triggered"));
    TuyaResetWifi();
    return true;  // Reset GPIO served here
  }
  return false;   // Don't serve other buttons
}

uint8_t TuyaGetTuyaWifiState(void) {

  uint8_t wifi_state = 0x02;  // Tuya description: AP network configuration (The indicator blinks at 1500 ms intervals)
/*
  switch(WifiState()){
    case WIFI_MANAGER:
      wifi_state = 0x01;
      break;
    case WIFI_RESTART:
      wifi_state =  0x03;
      break;
  }
*/
  if (rules_flag.wifi_connected) {
    wifi_state = 0x04;    // Tuya description: The Wi-Fi is configured, and the device successfully connects to the router (The indicator is steady on)
  }

/*    // TODO: Something is wrong with querying MQTT status. Stalls out MQTT and Tasmota
  if (MqttIsConnected()) {
//    wifi_state = 0x04;
    wifi_state = 0x05;    // Tuya description: The device connects to the router and cloud (The indicator is steady on)
  }
*/

  return wifi_state;
}

void TuyaSetWifiLed(void)
{
  Tuya.wifi_state = TuyaGetTuyaWifiState();
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("TYA: Set WiFi LED %d (%d)"), Tuya.wifi_state, WifiState());

  if (Tuya.low_power_mode) {
    TuyaProcessSendCommandFirst(TUYA_LOW_POWER_CMD_WIFI_STATE, &Tuya.wifi_state, 1);
  } else {
    TuyaProcessSendCommandFirst(TUYA_CMD_WIFI_STATE, &Tuya.wifi_state, 1);
  }
}

void TuyaCheckRxTimeout() {
  // Reset comm state after blind receive
  if (Tuya.comm.data_status == TUYA_COMM_DATA_STATUS_BLIND_RECEIVE) {
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Blind receive reset"));
    TuyaCommResetTxState();
    return;
  }
  // Filter out excluded modes
  switch (Tuya.comm.last_tx.comm_mode) {
    case TUYA_COMM_MODE_NONE:
    case TUYA_COMM_NO_REPLY:
      return;
  }
  if (Tuya.comm.data_status == TUYA_COMM_DATA_STATUS_SENT) {
    if (Tuya.comm.last_tx.timeout == 1) {
      AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Receive timeout..."));
      TuyaCommResetTxState(TUYA_COMM_DATA_STATUS_TIMEOUT);
    }
    else if (Tuya.comm.last_tx.timeout > 1)
      Tuya.comm.last_tx.timeout--;
  }
}

#ifdef USE_TUYA_TIME
void TuyaSetTime(void) {
  if (!RtcTime.valid) { return; }

  uint16_t payload_len = 8;
  uint8_t payload_buffer[8];
  payload_buffer[0] = 0x01;
  payload_buffer[1] = RtcTime.year %100;
  payload_buffer[2] = RtcTime.month;
  payload_buffer[3] = RtcTime.day_of_month;
  payload_buffer[4] = RtcTime.hour;
  payload_buffer[5] = RtcTime.minute;
  payload_buffer[6] = RtcTime.second;
  payload_buffer[7] = RtcTime.day_of_week;

  TuyaSendCmd(TUYA_CMD_SET_TIME, payload_buffer, payload_len);
}
#endif //USE_TUYA_TIME

#ifdef USE_ENERGY_SENSOR
/*********************************************************************************************\
 * Energy Interface
\*********************************************************************************************/

bool Xnrg32(uint8_t function)
{
  bool result = false;

  if (TUYA_DIMMER == my_module_type) {
    if (FUNC_PRE_INIT == function) {
      if (TuyaGetDpId(TUYA_MCU_FUNC_POWER) != 0) {
        if (TuyaGetDpId(TUYA_MCU_FUNC_CURRENT) == 0) {
          Energy.current_available = false;
        }
        if (TuyaGetDpId(TUYA_MCU_FUNC_VOLTAGE) == 0) {
          Energy.voltage_available = false;
        }
        energy_flg = XNRG_32;
      }
    }
  }
  return result;
}
#endif  // USE_ENERGY_SENSOR

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv16(uint8_t function)
{
  bool result = false;

  if (TUYA_DIMMER == my_module_type) {
   switch (function) {
      case FUNC_LOOP:
        if (TuyaSerial) {
          // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_LOOP called");
        
          TuyaCheckRxTimeout();       // Wait for debugging complete
          result = TuyaSerialInput();
          TuyaProcessPostSend();      // Send data if available
          
          // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_LOOP finished");

          // AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Free Memory: %d"), system_get_free_heap_size()); // <--- Testing Memory Usage (~26k remaining)
        }
        break;
      case FUNC_MODULE_INIT:
        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_MODULE_INIT called");

        result = TuyaModuleSelected();

        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_MODULE_INIT finished");
        break;
      case FUNC_PRE_INIT:
        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_PRE_INIT called");

        TuyaInit();

        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_PRE_INIT finished");
        break;
      case FUNC_SET_DEVICE_POWER:
        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_SET_DEVICE_POWER called");

        result = TuyaSetPower();

        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_SET_DEVICE_POWER finished");
        break;
      case FUNC_BUTTON_PRESSED:
        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_BUTTON_PRESSED called");

        result = TuyaButtonPressed();

        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_BUTTON_PRESSED finished");
        break;
      case FUNC_EVERY_SECOND:
        if (TuyaSerial)
        {
          // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_EVERY_SECOND called");

          if (TuyaMCUCurrentState() == TUYA_MCU_STATE_READY && Tuya.wifi_state != TuyaGetTuyaWifiState()) { TuyaSetWifiLed(); }
          if (!Tuya.low_power_mode) {
            // Handle normal power functions
            Tuya.heartbeat_timer++;
            if (Tuya.heartbeat_timer > 10) {
              Tuya.heartbeat_timer = 0;
              // If MCU is not ready start from begining
              if (TuyaMCUCurrentState() != TUYA_MCU_STATE_READY) {
                TuyaSetInitMCU();
              } else
              // Otherwise send heartbeat
                //TuyaSendCmd(TUYA_CMD_HEARTBEAT);
                TuyaProcessSendCommandFirst(TUYA_CMD_HEARTBEAT);
            }
#ifdef USE_TUYA_TIME
            if (!(uptime % 60)) {
              TuyaSetTime();
            }
#endif  //USE_TUYA_TIME
          } else {
            // Handle low power functions
            TuyaSendLowPowerSuccessIfNeeded();
          }
        }
        
        // AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Sleep setting: %d"), Settings.sleep); // <--- Testing sleep setting
        // AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Free Memory: %d"), system_get_free_heap_size()); // <--- Testing Memory Usage (~26k remaining)

        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_EVERY_SECOND finished");
        break;
      case FUNC_EVERY_250_MSECOND:
        // AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Free Memory: %d"), system_get_free_heap_size()); // <--- Testing Memory Usage (~26k remaining)
        break;
      case FUNC_SET_CHANNELS:
        result = TuyaSetChannels();

        // AddLog_P(LOG_LEVEL_DEBUG, "TYA: FUNC_SET_CHANNELS called");

        break;
      case FUNC_COMMAND:
        // AddLog_P(LOG_LEVEL_DEBUG, PSTR("TYA: Command sent to a function in this module"));
        if (TuyaMCUCurrentState() == TUYA_MCU_STATE_READY)
          result = DecodeCommand(kTuyaCommand, TuyaCommand);
        break;
    }
  }
  return result;
}

#endif  // USE_TUYA_MCU
#endif  // USE_LIGHT
