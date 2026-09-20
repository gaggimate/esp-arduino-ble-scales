#pragma once
#include "remote_scales.h"
#include "remote_scales_plugin_registry.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <algorithm>
#include <cctype>
#include <memory>
#include <vector>

// Timemore Dot (single-sensor scale, GATT service FFF0).
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
  NimBLERemoteService* service = nullptr;
  NimBLERemoteCharacteristic* weightCharacteristic = nullptr;
  NimBLERemoteCharacteristic* commandCharacteristic = nullptr;

  std::vector<uint8_t> rxBuffer;
  bool crcMismatchLogged = false;

  bool openLink();
  bool secureLink();
  bool discoverCharacteristics();
  bool subscribe();
  void configure();

  bool sendCommand(uint8_t type, const uint8_t* payload = nullptr, size_t length = 0);
  void onNotify(const uint8_t* data, size_t length);
  bool consumeFrame();
  void handleFrame(uint8_t frameClass, uint8_t type, const uint8_t* payload, size_t length);
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
  // Advertised as "TIMEMORE_Dot..."; some units use the model code "TES017" instead.
  static bool handles(const DiscoveredDevice& device) {
    std::string name = device.getName();
    if (name.rfind("TIMEMORE_Dot", 0) == 0) return true;
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
    return name.find("tes017") != std::string::npos;
  }
};
