#ifndef CONTROL_HPP
#define CONTROL_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace managers {
  class MotionPlayManager;
  class GaitManager;
  class VisionManager;
}  // namespace managers

namespace webots {
  class Motor;
  class PositionSensor;
  class LED;
  class Camera;
  class Accelerometer;
  class Gyro;
  class Keyboard;
  class Speaker;
};  // namespace webots

class Walk : public webots::Robot {
public:
  Walk();
  virtual ~Walk();
  void run();
  void checkIfFallen();

private:
  int mTimeStep;
  double mStartTime;
  bool mStopwatchRunning;

  void myStep();
  void wait(int ms);

  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::Accelerometer *mAccelerometer;
  webots::Gyro *mGyro;
  webots::Keyboard *mKeyboard;
  webots::Camera *mCamera;

  managers::VisionManager *mVisionManager;
  managers::MotionPlayManager *mMotionManager;
  managers::GaitManager *mGaitManager;
};

#endif
