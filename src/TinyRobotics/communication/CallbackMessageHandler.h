#pragma once
#include "MessageHandler.h"
#include <functional>

namespace tinyrobotics {

/**
 * @brief MessageHandler implementation using user-provided callbacks.
 *
 * Allows setting custom callbacks for each supported message type.
 */
class CallbackMessageHandler : public MessageHandler {
 public:
  using ValueCallback = std::function<bool(const Message<float>&)>;
  using CoordinateCallback = std::function<bool(const Message<Coordinate<float>>&)>;
  using GPSCoordinateCallback = std::function<bool(const Message<GPSCoordinate>&)>;

  CallbackMessageHandler() = default;

  bool onMessage(const Message<float>& msg) override {
    if (valueCallback_) return valueCallback_(msg);
    return false;
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    if (coordinateCallback_) return coordinateCallback_(msg);
    return false;
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    if (gpsCoordinateCallback_) return gpsCoordinateCallback_(msg);
    return false;
  }

  void setValueCallback(ValueCallback cb) { valueCallback_ = cb; }
  void setCoordinateCallback(CoordinateCallback cb) { coordinateCallback_ = cb; }
  void setGPSCoordinateCallback(GPSCoordinateCallback cb) { gpsCoordinateCallback_ = cb; }

 private:
  ValueCallback valueCallback_;
  CoordinateCallback coordinateCallback_;
  GPSCoordinateCallback gpsCoordinateCallback_;
};

} // namespace tinyrobotics
