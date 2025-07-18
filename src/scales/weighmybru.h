#pragma once
#include "remote_scales.h"
#include "remote_scales_plugin_registry.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEScan.h>
#include <vector>
#include <memory>

enum class WeighMyBrewMessageType : uint8_t {
  SYSTEM = 0x0A,
  WEIGHT = 0x0B
};

class WeighMyBrewScales : public RemoteScales {

public:
  WeighMyBrewScales(const DiscoveredDevice& device);
  void update() override;
  bool connect() override;
  void disconnect() override;
  bool isConnected() override;
  bool tare() override;

private:
  std::string weightUnits;
  float time;
  uint8_t battery;

  uint32_t lastHeartbeat = 0;

  bool markedForReconnection = false;

  NimBLERemoteService* service;
  NimBLERemoteCharacteristic* weightCharacteristic;
  NimBLERemoteCharacteristic* commandCharacteristic;

  std::vector<uint8_t> dataBuffer;

  bool performConnectionHandshake();
  void subscribeToNotifications();

  void sendMessage(WeighMyBrewMessageType msgType, const uint8_t* payload, size_t length, bool waitResponse = false);
  void sendEvent(const uint8_t* payload, size_t length);
  void sendHeartbeat();
  void sendNotificationRequest();
  void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
  bool decodeAndHandleNotification();
};

class WeighMyBrewScalePlugin {
public:
  static void apply() {
    RemoteScalesPlugin plugin = RemoteScalesPlugin{
      .id = "plugin-weighmybrew",
      .handles = [](const DiscoveredDevice& device) { return WeighMyBrewScalePlugin::handles(device); },
      .initialise = [](const DiscoveredDevice& device) -> std::unique_ptr<RemoteScales> { return std::make_unique<WeighMyBrewScales>(device); },
    };
    RemoteScalesPluginRegistry::getInstance()->registerPlugin(plugin);
  }
private:
  static bool handles(const DiscoveredDevice& device) {
    const std::string& deviceName = device.getName();
    return !deviceName.empty() && (deviceName.find("WeighMyBru") == 0);
  }
};
