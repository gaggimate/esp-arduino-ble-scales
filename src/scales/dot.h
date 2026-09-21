#pragma once
#include "remote_scales.h"
#include "remote_scales_plugin_registry.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
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
      .handles = [](const DiscoveredDevice& device) { return TimemoreDotScalesPlugin::handles(device) || matchesBasic3Link(device.getName()); },
      .initialise = [](const DiscoveredDevice& device) -> std::unique_ptr<RemoteScales> { return std::make_unique<TimemoreDotScales>(device); },
    };
    RemoteScalesPluginRegistry::getInstance()->registerPlugin(plugin);
  }

  // Basic 3.0 Link advertises as "Basic3 Link" (confirmed on real hardware,
  // model TES016) and reuses the Dot GATT protocol (FFF0/FFF1/FFF2), per
  // Beanconqueror's TimemoreBasicScale matcher.
  static bool matchesBasic3Link(const std::string& deviceName) {
    std::string lower(deviceName);
    for (char& c : lower) {
      if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return contains(lower, "basic3") || contains(lower, "basic 3") ||
           (contains(lower, "timemore") && contains(lower, "basic"));
  }

  // std::string::contains needs C++23; the toolchains GaggiMate builds with are older.
  static bool contains(std::string_view text, const char* needle) {
    return text.find(needle) != std::string::npos;
  }

private:
  static bool handles(const DiscoveredDevice& device) {
    const std::string& deviceName = device.getName();
    return !deviceName.empty() && deviceName.find("TIMEMORE_Dot") == 0;
  }
};
