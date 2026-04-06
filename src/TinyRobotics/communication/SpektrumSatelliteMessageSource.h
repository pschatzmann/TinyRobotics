#pragma once
#include "MessageHandler.h"
#include "SpektrumSatellite.h"

namespace tinyrobotics {

/**
 * @class SpektrumSatelliteMessageSource
 * @ingroup communication
 * @brief Publishes control messages from a Spektrum satellite receiver to the
 * TinyRobotics message bus.
 *
 * This class bridges a Spektrum satellite receiver and the TinyRobotics message
 * bus system. It polls the receiver for new frames and, when available,
 * publishes standardized control messages:
 *   - Roll (degrees)
 *   - Pitch (degrees)
 *   - Yaw (degrees)
 *   - Throttle (percent)
 *
 * Each message is published with the appropriate unit and origin
 * (MessageOrigin::RemoteControl). The channel value range is automatically set
 * to 0-100 and mapped to -90..+90 degrees for angles.
 *
 * This class depends on https://github.com/pschatzmann/SpektrumSatellite
 * The SpektrumSatellite class must be set up proplerly and passed to the
 * constructor.
 *
 * Usage:
 * @code
 * SpektrumSatellite<uint16_t> satellite;
 * SpektrumSatelliteMessageSource<uint16_t> source(satellite);
 * // In your main loop:
 * source.run();
 * @endcode
 *
 * @tparam T Data type for channel values (default: uint16_t)
 */

template <class T = uint16_t>
class SpektrumSatelliteMessageSource : public MessageSource {
 public:
  SpektrumSatelliteMessageSource(SpektrumSatellite<T>& satellite)
      : satellite(satellite) {
    // scale the values from 0 to 1000
    satellite.setChannelValueRange(0, 100);
  };

  /// call in your loop to process incoming satellite messages and publish to
  /// handlers
  void run() {
    if (satellite.getFrame()) {
      float rollDegrees = toDegrees(satellite.getAileron());
      float pitchDegrees = toDegrees(satellite.getElevator());
      float yawDegrees = toDegrees(satellite.getRudder());
      float throttle = 0.1f * satellite.getThrottle();

      publish(Message(MessageContent::Throttle, Unit::Percent),
              MessageOrigin::RemoteControl);

      // 3D devices
      publish(Message(MessageContent::Roll,
                      Angle(rollDegrees, Unit::AngleDegree),
                      MessageOrigin::RemoteControl));

      publish(Message(MessageContent::Pitch,
                      Angle(pitchDegrees, Unit::AngleDegree),
                      MessageOrigin::RemoteControl));

      publish(Message(MessageContent::Yaw, Angle(yawDegrees, Unit::AngleDegree),
                      MessageOrigin::RemoteControl));

      // 2D
      publish(Message(MessageContent::SteeringAngle, rollDegrees,
                      Unit::AngleDegree),
              MessageOrigin::RemoteControl);
    }
  }

 protected:
  SpektrumSatellite<T>& satellite;

  float toDegrees(T value) {
    // Assuming the satellite values are in the range 0-1000, map to -90 to +90
    return map(value, 0.0f, 1000.0f, -90.0f, 90.0f);
  }
};

}  // namespace tinyrobotics