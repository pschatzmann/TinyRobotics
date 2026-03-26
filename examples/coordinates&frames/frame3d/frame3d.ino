#include <TinyRobotics.h>

FrameMgr3D<3> tf;


void setup() {
  FrameId world{FrameType::WORLD, 0};
  FrameId base{FrameType::BASE_LINK, 0 };
  FrameId cam{FrameType::CAMERA, 0};

  tf.add(world, -1);  // root
  tf.add(base, 0);    // base -> world
  tf.add(cam, 1);     // cam -> base
}

void loop() {
  int8_t cam = tf.find({FrameType::CAMERA, 0});
  int8_t world = tf.find({FrameType::WORLD, 0});

  Transform t = tf.get(cam, world);
}