#pragma once
#include "remote_scales.h"
#include "remote_scales_plugin_registry.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cctype>
#include <cstring>
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

private:
  bool markedForReconnection = false;

  NimBLERemoteService* service = nullptr;
  NimBLERemoteCharacteristic* weightCharacteristic = nullptr;
  NimBLERemoteCharacteristic* commandCharacteristic = nullptr;

  std::vector<uint8_t> dataBuffer;

  bool performConnectionHandshake();
  bool subscribeToNotifications();
  void sendHandshake();

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
    const std::string& deviceName = device.getName();
    if (deviceName.empty()) return false;
    if (deviceName.find("TIMEMORE_Dot") == 0) return true;

    // Basic 3.0 Link advertises as "Basic3 Link" (confirmed on real hardware,
    // model TES016) and reuses the Dot GATT protocol (FFF0/FFF1/FFF2), per
    // Beanconqueror's TimemoreBasicScale matcher.
    std::string lower(deviceName);
    for (char& c : lower) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return std::strstr(lower.c_str(), "basic3") != nullptr || std::strstr(lower.c_str(), "basic 3") != nullptr ||
           (std::strstr(lower.c_str(), "timemore") != nullptr && std::strstr(lower.c_str(), "basic") != nullptr);
  }
};
