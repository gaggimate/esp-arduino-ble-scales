#pragma once
#include "remote_scales.h"
#include "remote_scales_plugin_registry.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <algorithm>
#include <cctype>
#include <vector>
#include <memory>

class TimemoreDotScales : public RemoteScales {

public:
  explicit TimemoreDotScales(const DiscoveredDevice& device);
  void update() override;
  bool connect() override;
  void disconnect() override;
  bool isConnected() override;
  bool tare() override;

  bool hasBatteryLevel() const override { return true; }
  bool hasTimerControl() const override { return true; }
  void startTimer() override;
  void stopTimer() override;
  void resetTimer() override;

private:
  bool markedForReconnection = false;

  NimBLERemoteService* service = nullptr;
  NimBLERemoteCharacteristic* weightCharacteristic = nullptr;
  NimBLERemoteCharacteristic* commandCharacteristic = nullptr;

  std::vector<uint8_t> dataBuffer;

  bool performConnectionHandshake();
  bool subscribeToNotifications();

  void notifyCallback(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify);
  bool decodeAndHandleNotification();
};

class TimemoreDotScalesPlugin {
public:
  static void apply() {
    auto plugin = RemoteScalesPlugin{
      .id = "plugin-timemore-dot",
      .handles = [](const DiscoveredDevice& device) { return TimemoreDotScalesPlugin::handles(device); },
      .initialise = [](const DiscoveredDevice& device) -> std::unique_ptr<RemoteScales> { return std::make_unique<TimemoreDotScales>(device); },
    };
    RemoteScalesPluginRegistry::getInstance()->registerPlugin(plugin);
  }

private:
  static bool handles(const DiscoveredDevice& device) {
    // Match on name: the Dot advertises as "TIMEMORE DOT" / "TIMEMORE_Dot"
    // (case-insensitive "dot", plus the TES017 model code). Outside pairing
    // mode the scale can advertise service UUIDs with no name at all, so
    // fall back to the 0xFFF0 service being present in the advertisement.
    const std::string& deviceName = device.getName();
    if (!deviceName.empty()) {
      std::string lower = deviceName;
      std::transform(lower.begin(), lower.end(), lower.begin(),
                     [](unsigned char c) { return std::tolower(c); });
      if (lower.find("dot") != std::string::npos ||
          lower.find("tes017") != std::string::npos) {
        return true;
      }
    }
    return device.advertisesService(NimBLEUUID("FFF0")) ||
           device.advertisesService(NimBLEUUID("0000fff0-0000-1000-8000-00805f9b34fb"));
  }
};
