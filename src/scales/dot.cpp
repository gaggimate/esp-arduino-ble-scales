#include "dot.h"
#include "remote_scales_plugin_registry.h"

// Timemore Dot — single-sensor BLE scale.
//
// Frame layout (big-endian):
//   [A5 5A] [class] [type] [len_hi len_lo] [payload...] [crc_hi crc_lo]
//      2       1      1         2              len            2
// Total frame length = len + 8.
//
// Weight notification: class=0x01, type=0x01, payload_len=0x0009.
// Frame bytes [6..9] = signed BE int32 grams * 10. Bytes [10..14] are
// additional payload (likely flow / secondary metric) the driver ignores;
// bytes [15..16] are the frame CRC trailer.

const NimBLEUUID serviceUUID("FFF0");
const NimBLEUUID weightCharacteristicUUID("FFF1");
const NimBLEUUID commandCharacteristicUUID("FFF2");

// Captured via iOS PacketLogger; CRC trailers are scale-specific and hardcoded
// from the capture rather than computed.
static const uint8_t TARE_CMD[]      = { 0xA5, 0x5A, 0x02, 0x04, 0x00, 0x00, 0x9A, 0x00 };
static const uint8_t HANDSHAKE_CMD[] = { 0xA5, 0x5A, 0x03, 0x0D, 0x00, 0x00, 0x64, 0xD1 };

static constexpr size_t FRAME_HEADER_LEN = 8;
// Generous upper bound — known frames are <=12 bytes of payload. A glitched
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

  // The Dot will not emit weight notifications until the link is encrypted.
  // Look the client back up by peer address since RemoteScales keeps it private.
  NimBLEClient* nimbleClient = NimBLEDevice::getClientByPeerAddress(RemoteScales::getDevice().getAddress());
  if (nimbleClient == nullptr || !nimbleClient->secureConnection()) {
    RemoteScales::log("secureConnection failed\n");
    clientCleanup();
    return false;
  }

  if (!performConnectionHandshake()) {
    return false;
  }
  if (!subscribeToNotifications()) {
    RemoteScales::log("FFF1 subscribe failed (notify and indicate)\n");
    clientCleanup();
    return false;
  }
  sendHandshake();
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
    RemoteScales::log("Marked for disconnection. Will attempt to reconnect.\n");
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
  if (!commandCharacteristic->writeValue(TARE_CMD, sizeof(TARE_CMD), true)) {
    RemoteScales::log("Tare write failed\n");
    return false;
  }
  // The scale ACKs the tare command but only actually zeros the reading after
  // receiving the status poll. Use waitResponse=true so a queue/GATT failure
  // surfaces here rather than being silently dropped.
  if (!commandCharacteristic->writeValue(HANDSHAKE_CMD, sizeof(HANDSHAKE_CMD), true)) {
    RemoteScales::log("Tare follow-up poll failed; scale will not zero\n");
    return false;
  }
  return true;
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

  uint8_t cls  = dataBuffer[2];
  uint8_t type = dataBuffer[3];

  if (cls == 0x01 && type == 0x01 && payloadLen == 9) {
    // Weight frame. Signed big-endian int32 at bytes [6..9], 0.1 g resolution.
    int32_t raw = (static_cast<int32_t>(dataBuffer[6]) << 24) |
                  (static_cast<int32_t>(dataBuffer[7]) << 16) |
                  (static_cast<int32_t>(dataBuffer[8]) << 8)  |
                   static_cast<int32_t>(dataBuffer[9]);
    RemoteScales::setWeight(raw / 10.0f);
  } else {
    RemoteScales::log("Unhandled frame cls=%02X type=%02X len=%u\n",
                      cls, type, (unsigned)payloadLen);
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

void TimemoreDotScales::sendHandshake() {
  if (commandCharacteristic == nullptr) return;
  commandCharacteristic->writeValue(HANDSHAKE_CMD, sizeof(HANDSHAKE_CMD), false);
}
