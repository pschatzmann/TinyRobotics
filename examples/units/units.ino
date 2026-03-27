// Example: Demonstrate TinyRobotics units module
#include <TinyRobotics.h>

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("TinyRobotics Units Example");

  // Distance
  Distance d1(1.0, DistanceUnit::M);   // 1 meter
  Distance d2(100.0, DistanceUnit::CM); // 100 centimeters
  Distance d3(1000.0, DistanceUnit::MM); // 1000 millimeters
  Distance d4(3.28084, DistanceUnit::FEET); // ~1 meter in feet

  Serial.print("d1 in meters: "); Serial.println(d1.getDistance(DistanceUnit::M));
  Serial.print("d2 in meters: "); Serial.println(d2.getDistance(DistanceUnit::M));
  Serial.print("d3 in meters: "); Serial.println(d3.getDistance(DistanceUnit::M));
  Serial.print("d4 in meters: "); Serial.println(d4.getDistance(DistanceUnit::M));


  // Angle
  Angle a1(90.0, AngleUnit::DEG);
  Angle a2(3.14159f / 2, AngleUnit::RAD);
  Serial.print("a1 in degrees: "); Serial.println(a1.getAngle(AngleUnit::DEG));
  Serial.print("a1 in radians: "); Serial.println(a1.getAngle(AngleUnit::RAD));
  Serial.print("a2 in radians: "); Serial.println(a2.getAngle(AngleUnit::RAD));
  Serial.print("a2 in degrees: "); Serial.println(a2.getAngle(AngleUnit::DEG));

  // Speed
  Speed s1(2.5, SpeedUnit::MPS); // 2.5 meters per second
  Serial.print("Speed: "); Serial.print(s1.getSpeed(SpeedUnit::MPS)); Serial.println(" m/s");
  Serial.print("Speed: "); Serial.print(s1.getSpeed(SpeedUnit::KPH)); Serial.println(" km/h");

  // Time
  Time t1(60.0, TimeUnit::S);
  Time t2(60000.0, TimeUnit::MS);
  Serial.print("t1 in seconds: "); Serial.println(t1.getTime(TimeUnit::S));
  Serial.print("t2 in seconds: "); Serial.println(t2.getTime(TimeUnit::S));

}

void loop() {
  // Nothing to do
}
