#pragma once
#include <functional>

#include "MessageHandler.h"

namespace tinyrobotics {

/**
 * @brief MessageHandler implementation using user-provided callbacks.
 *
 * Allows setting custom callbacks for each supported message type.
 */
class CallbackMessageHandler : public MessageHandler {
 public:
  using ValueCallback = std::function<bool(const Message<float>&, void*)>;
  using CoordinateCallback =
      std::function<bool(const Message<Coordinate<float>>&, void*)>;
  using GPSCoordinateCallback =
      std::function<bool(const Message<GPSCoordinate>&, void*)>;

  CallbackMessageHandler() = default;

  bool onMessage(const Message<float>& msg) override {
    if (valueCallback_) return valueCallback_(msg, ref);
    return false;
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    if (coordinateCallback_) return coordinateCallback_(msg, ref);
    return false;
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    if (gpsCoordinateCallback_) return gpsCoordinateCallback_(msg, ref);
    return false;
  }

  void setValueCallback(ValueCallback cb, void* reference) {
    valueCallback_ = cb;
    ref = reference;
  }
  void setCoordinateCallback(CoordinateCallback cb, void* reference) {
    coordinateCallback_ = cb;
    ref = reference;
  }
  void setGPSCoordinateCallback(GPSCoordinateCallback cb, void* reference) {
    gpsCoordinateCallback_ = cb;
    ref = reference;
  }

 protected:
  void* ref = nullptr;
  ValueCallback valueCallback_;
  CoordinateCallback coordinateCallback_;
  GPSCoordinateCallback gpsCoordinateCallback_;

};

}  // namespace tinyrobotics
