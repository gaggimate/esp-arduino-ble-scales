#include "dot.h"
#include "remote_scales_plugin_registry.h"

// Timemore Dot — single-sensor BLE scale.
//
// Framed protocol (same as the official Timemore client, verified against a
// real Dot):
//   [A5 5A] [opcode] [cmdId] [len_hi len_lo] [payload...] [crc_hi crc_lo]
//      2       1       1          2              len            2
// Total frame length = len + 8.
//
// Weight notification: opcode 0x01, cmdId 0x01, payload_len 9. Payload bytes
// [0..3] = signed BE int32 grams * 10; the remaining payload (flow / secondary
// metric) is ignored by this driver.

const NimBLEUUID serviceUUID("FFF0");
const NimBLEUUID weightCharacteristicUUID("FFF1");
const NimBLEUUID commandCharacteristicUUID("FFF2");

// Tare command = A5 5A 03 0D 00 00 + CRC16/IBM (0x64D1), byte-identical to the
// official client's buildFrame(0x03, 0x0D). The Dot zeroes on this command.
static const uint8_t TARE_CMD[] = { 0xA5, 0x5A, 0x03, 0x0D, 0x00, 0x00, 0x64, 0xD1 };
// Timer commands = buildFrame(0x03, 0x02, {0x01|0x02|0x03}) with CRC16/IBM
// (payload 01=start, 02=stop, 03=reset), matching the official client's
// setTimer().
static const uint8_t TIMER_START_CMD[] = { 0xA5, 0x5A, 0x03, 0x02, 0x00, 0x01, 0x01, 0x18, 0x67 };
static const uint8_t TIMER_STOP_CMD[]  = { 0xA5, 0x5A, 0x03, 0x02, 0x00, 0x01, 0x02, 0x19, 0x27 };
static const uint8_t TIMER_RESET_CMD[] = { 0xA5, 0x5A, 0x03, 0x02, 0x00, 0x01, 0x03, 0xD9, 0xE6 };
// Post-connect init sequence (mirrors the official client): set unit to gram,
// set mode to standard, request battery. The Dot only reports weight after
// unit/mode are set.
static const uint8_t INIT_UNIT_CMD[]    = { 0xA5, 0x5A, 0x03, 0x06, 0x00, 0x01, 0x00, 0xE8, 0xA7 };
static const uint8_t INIT_MODE_CMD[]    = { 0xA5, 0x5A, 0x03, 0x08, 0x00, 0x02, 0x01, 0x00, 0xEB, 0x31 };
static const uint8_t INIT_BATTERY_CMD[] = { 0xA5, 0x5A, 0x02, 0x05, 0x00, 0x00, 0x5A, 0x51 };

static constexpr size_t FRAME_HEADER_LEN = 8;
// Generous upper bound — known frames are <=9 bytes of payload. A glitched
// notification with a bogus length would otherwise stall the parser while the
// internal buffer grew waiting for bytes that never arrive.
static constexpr uint16_t MAX_PAYLOAD_LEN = 64;
static constexpr uint8_t MAGIC_0 = 0xA5;
static constexpr uint8_t MAGIC_1 = 0x5A;

//-----------------------------------------------------------------------------------/
//---------------------------        PUBLIC       -----------------------------------/
//-----------------------------------------------------------------------------------/
TimemoreDotScales::TimemoreDotScales(const DiscoveredDevice& device) : RemoteScales(device) {}

bool TimemoreDotScales::connect() {
  if (RemoteScales::clientIsConnected()) {
    RemoteScales::log("Already connected\n");
    return true;
  }

  RemoteScales::log("Connecting to %s[%s]\n", RemoteScales::getDeviceName().c_str(), RemoteScales::getDeviceAddress().c_str());
  // After a previous session the Dot can hold its end of the link open briefly
  // on the peripheral side; the first BLE central connect can then fail. Retry
  // a few times with a short delay before giving up.
  bool linkUp = false;
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (RemoteScales::clientConnect()) { linkUp = true; break; }
    RemoteScales::clientCleanup();
    RemoteScales::log("clientConnect attempt %d failed, retrying\n", attempt + 1);
    delay(500);
  }
  if (!linkUp) {
    RemoteScales::log("clientConnect gave up after retries\n");
    return false;
  }

  // The real Dot requires no pairing/encryption — the official app connects
  // directly. Some units briefly hold the link open on their side after a
  // previous session, so retry the plain connection before giving up (done
  // above). A best-effort security attempt is harmless but never fatal.
  NimBLEClient* nimbleClient = NimBLEDevice::getClientByPeerAddress(NimBLEAddress(RemoteScales::getDeviceAddress()));
  if (nimbleClient != nullptr) {
    NimBLEDevice::setSecurityAuth(true, false, true);
    if (!nimbleClient->secureConnection()) {
      RemoteScales::log("secureConnection failed, continuing unencrypted\n");
    }
  }

  if (!performConnectionHandshake()) {
    return false;
  }
  if (!subscribeToNotifications()) {
    RemoteScales::log("FFF1 subscribe failed (notify and indicate)\n");
    clientCleanup();
    return false;
  }
  // The Dot only reports weight after being told the unit and mode; without
  // this init sequence the scale stays silent. Timings mirror the official
  // client (TimemoreScale::sendInitSequence). Blocking is fine here — connect()
  // runs from the caller's task, not a NimBLE stack callback.
  delay(500);
  commandCharacteristic->writeValue(INIT_UNIT_CMD, sizeof(INIT_UNIT_CMD), false);
  delay(200);
  commandCharacteristic->writeValue(INIT_MODE_CMD, sizeof(INIT_MODE_CMD), false);
  delay(100);
  commandCharacteristic->writeValue(INIT_BATTERY_CMD, sizeof(INIT_BATTERY_CMD), false);
  RemoteScales::setWeight(0.f);
  return true;
}

void TimemoreDotScales::disconnect() {
  RemoteScales::clientCleanup();
}

bool TimemoreDotScales::isConnected() {
  return RemoteScales::clientIsConnected();
}

void TimemoreDotScales::update() {
  if (markedForReconnection) {
    RemoteScales::log("Marked for reconnection. Will attempt to reconnect.\n");
    RemoteScales::clientCleanup();
    if (!connect()) {
      RemoteScales::log("Reconnect failed; will retry on next update\n");
      return; // leave markedForReconnection=true so the next update retries
    }
    markedForReconnection = false;
  }
}

bool TimemoreDotScales::tare() {
  if (!isConnected() || commandCharacteristic == nullptr) return false;
  if (!commandCharacteristic->writeValue(TARE_CMD, sizeof(TARE_CMD), false)) {
    RemoteScales::log("Tare write failed\n");
    return false;
  }
  return true;
}

void TimemoreDotScales::startTimer() {
  if (commandCharacteristic != nullptr) {
    commandCharacteristic->writeValue(TIMER_START_CMD, sizeof(TIMER_START_CMD), false);
  }
}

void TimemoreDotScales::stopTimer() {
  if (commandCharacteristic != nullptr) {
    commandCharacteristic->writeValue(TIMER_STOP_CMD, sizeof(TIMER_STOP_CMD), false);
  }
}

void TimemoreDotScales::resetTimer() {
  if (commandCharacteristic != nullptr) {
    commandCharacteristic->writeValue(TIMER_RESET_CMD, sizeof(TIMER_RESET_CMD), false);
  }
}

//-----------------------------------------------------------------------------------/
//---------------------------       PRIVATE       -----------------------------------/
//-----------------------------------------------------------------------------------/
void TimemoreDotScales::notifyCallback(
  NimBLERemoteCharacteristic* characteristic,
  uint8_t* data,
  size_t length,
  bool isNotify
) {
  dataBuffer.insert(dataBuffer.end(), data, data + length);
  // Drain frames; each iteration consumes one frame and returns whether more
  // remain in the buffer.
  while (decodeAndHandleNotification()) {
    // intentionally empty
  }
}

bool TimemoreDotScales::decodeAndHandleNotification() {
  // Resync to magic bytes — drop leading garbage.
  while (!dataBuffer.empty() && dataBuffer[0] != MAGIC_0) {
    dataBuffer.erase(dataBuffer.begin());
  }
  if (dataBuffer.size() < FRAME_HEADER_LEN) return false;
  if (dataBuffer[1] != MAGIC_1) {
    dataBuffer.erase(dataBuffer.begin());
    return !dataBuffer.empty();
  }

  uint16_t payloadLen = (static_cast<uint16_t>(dataBuffer[4]) << 8) | dataBuffer[5];
  if (payloadLen > MAX_PAYLOAD_LEN) {
    // Likely a glitched / desynced frame. Drop the magic byte and re-resync
    // rather than blocking the parser waiting for bytes that may never come.
    RemoteScales::log("Implausible payloadLen=%u, resyncing\n", payloadLen);
    dataBuffer.erase(dataBuffer.begin());
    return !dataBuffer.empty();
  }
  size_t frameLen = static_cast<size_t>(payloadLen) + FRAME_HEADER_LEN;
  if (dataBuffer.size() < frameLen) return false;

  uint8_t opcode = dataBuffer[2];
  uint8_t cmdId  = dataBuffer[3];

  if ((opcode == 0x01 || opcode == 0x02) && cmdId == 0x01 && payloadLen >= 8) {
    // Weight frame. Signed big-endian int32 at bytes [6..9], 0.1 g resolution.
    int32_t raw = (static_cast<int32_t>(dataBuffer[6]) << 24) |
                  (static_cast<int32_t>(dataBuffer[7]) << 16) |
                  (static_cast<int32_t>(dataBuffer[8]) << 8)  |
                   static_cast<int32_t>(dataBuffer[9]);
    RemoteScales::setWeight(raw / 10.0f);
  } else if ((opcode == 0x01 || opcode == 0x02) && cmdId == 0x05 && payloadLen >= 1) {
    // Battery frame. Payload byte 1 (fallback byte 0) is the percentage.
    if (dataBuffer[7] <= 100) {
      RemoteScales::setBatteryLevel(dataBuffer[7]);
    } else if (dataBuffer[6] <= 100) {
      RemoteScales::setBatteryLevel(dataBuffer[6]);
    }
  } else {
    RemoteScales::log("Unhandled frame op=%02X cmd=%02X len=%u\n",
                      opcode, cmdId, (unsigned)payloadLen);
  }

  dataBuffer.erase(dataBuffer.begin(), dataBuffer.begin() + frameLen);
  return !dataBuffer.empty();
}

bool TimemoreDotScales::performConnectionHandshake() {
  RemoteScales::log("Performing handshake\n");

  service = RemoteScales::clientGetService(serviceUUID);
  if (service == nullptr) {
    clientCleanup();
    return false;
  }

  weightCharacteristic = service->getCharacteristic(weightCharacteristicUUID);
  commandCharacteristic = service->getCharacteristic(commandCharacteristicUUID);
  if (weightCharacteristic == nullptr || commandCharacteristic == nullptr) {
    clientCleanup();
    return false;
  }
  return true;
}

bool TimemoreDotScales::subscribeToNotifications() {
  auto callback = [this](NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
    notifyCallback(characteristic, data, length, isNotify);
  };
  // Try notify first; fall back to indicate. Some NimBLE/peripheral combos
  // mis-report capability bits, so do not gate on canNotify().
  if (weightCharacteristic->subscribe(true, callback)) return true;
  return weightCharacteristic->subscribe(false, callback);
}

