#include "control.hpp"

#include <GaitManager.hpp>
#include <MotionPlayManager.hpp>
#include <VisionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp> 
#include "Walking.h"
#include "ColorFinder.h" 

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static ofstream gyroLogFile;
static ofstream trajectoryLogFile;

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/ 
};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();
  
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  // 1. Inisialisasi Kamera
  mCamera = getCamera("Camera");
  if (mCamera) {
    mCamera->enable(2 * mTimeStep);
  } else {
    std::cout << "Error: Perangkat kamera tidak ditemukan!" << std::endl;
  }
  int width = mCamera->getWidth();
  int height = mCamera->getHeight();

  mGyro = getGyro("Gyro");
  mGyro->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);

  mVisionManager = new VisionManager(
    width,      // camera width
    height,     // camera height
    0,          // target hue (tidak relevan untuk hitam)
    180,        // hue tolerance (toleransi besar karena tidak relevan)
    0,          // min saturation (saturasi rendah untuk hitam)
    30,         // min value (nilai kecerahan RENDAH untuk hitam)
    0.1,        // min percent of image
    30.0        // max percent of image
  );

  mMotionManager = new MotionPlayManager(this);
  mGaitManager = new GaitManager(this, "config.ini");


  mStartTime = 0.0;
  mStopwatchRunning = false;
}

Walk::~Walk() {
  delete mVisionManager;
  delete mMotionManager;
  delete mGaitManager;
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// function containing the main feedback loop
void Walk::run() {

  // First step to update sensors values
  myStep();

  // play the hello motion
  mMotionManager->playPage(9);  // init position
  wait(200);

  // main loop
  bool loopplay = false;
  const int SEARCH_DURATION = 100;

  while (true) {
    checkIfFallen();

    mMotors[19]->setPosition(-0.6);
    const unsigned char *image = mCamera->getImage();
    if (!image) {
      std::cout << "Gagal mengambil gambar" << std::endl;
      myStep();
      continue;
    }

    int width = mCamera->getWidth();
    int height = mCamera->getHeight();

    // Panggil fungsi baru getLinePosition
    double line_position = mVisionManager->getLinePosition(image, width, height);
    bool line_detected = (line_position != -9999);

    if (!line_detected) {
      mLineLostCounter++;
      std::cout << "Status: Garis Hilang! Mencari... (" << mLineLostCounter << ")" << std::endl;
      
      mGaitManager->setXAmplitude(0.0);

      if (mLineLostCounter <= SEARCH_DURATION) {
        mGaitManager->setAAmplitude(0.5); 
      } else if (mLineLostCounter <= SEARCH_DURATION * 3) {
        mGaitManager->setAAmplitude(-0.5);
      } else {
        mLineLostCounter = 0;
      }

    } else {
      mLineLostCounter = 0;
      
      double turn_threshold = width / 6.0;

      if (std::abs(line_position) < turn_threshold) {
        std::cout << "Status: Jalan Lurus (Posisi: " << line_position << ")" << std::endl;
        mGaitManager->setXAmplitude(1.0);
        // Kontrol proporsional sederhana untuk koreksi halus
        mGaitManager->setAAmplitude(-line_position * 0.005); 
      } else if (line_position < 0) {
        std::cout << "Status: Belok Kiri (Posisi: " << line_position << ")" << std::endl;
        mGaitManager->setXAmplitude(0.8);
        mGaitManager->setAAmplitude(0.4);
      } else {
        std::cout << "Status: Belok Kanan (Posisi: " << line_position << ")" << std::endl;
        mGaitManager->setXAmplitude(0.8);
        mGaitManager->setAAmplitude(-0.4);
      }
    }

    int key = mKeyboard->getKey();
    if (key == ' ') {
        if (loopplay) {
            mGaitManager->stop();
            loopplay = false;
        } else {
            mGaitManager->start();
            loopplay = true;
        }
        wait(200);
    }
    
    if (loopplay) {
      if (!mGaitManager->isWalking()) {
        mGaitManager->start();
      }
      mGaitManager->step(mTimeStep);
    }
    

    myStep();
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static int fleft = 0;
  static int fright = 0;
  static const double acc_tolerance = 30.0;
  static const double acc_step = 100;

  const double *acc = mAccelerometer->getValues();

  // Cek jatuh ke depan (sumbu Y)
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  // Cek jatuh ke belakang (sumbu Y)
  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  // Cek jatuh ke kiri (sumbu X)
  if (acc[0] < 400.0 + acc_tolerance)
    fleft++;
  else
    fleft = 0;

  // Cek jatuh ke kanan (sumbu X)
  if (acc[0] > 600.0 - acc_tolerance)
    fright++;
  else
    fright = 0;

  // Jatuh Kedepan
  if (fup > acc_step) {
    cout << "-------Jatuh Kedepan-------" << endl;
    mMotionManager->playPage(10);  // f_up
    mMotionManager->playPage(9);   // init position
    fup = 0; fdown = 0; fleft = 0; fright = 0; // Reset semua counter
  }
  // Jatuh Kebelakang
  else if (fdown > acc_step) {
    cout << "-------Jatuh Kebelakang-------" << endl;
    mMotionManager->playPage(11);  // b_up
    mMotionManager->playPage(9);   // init position
    fdown = 0; fup = 0; fleft = 0; fright = 0; // Reset semua counter
  }
  // Jatuh Kesamping Kiri
  else if (fleft > acc_step) {
    cout << "-------Jatuh Kesamping Kanan, mendorong dengan tangan kanan...-------" << endl;
    
    // Hentikan mode berjalan jika aktif
    mGaitManager->stop();
    
    // --- Urutan Gerakan Tangan Kanan ---
    // Tahap 1: Posisikan lengan untuk mendorong
    mMotors[2]->setPosition(-1.0); // ArmUpperR (bahu kanan ke samping)
    mMotors[0]->setPosition(-1.5); // ShoulderR (bahu kanan ke depan)
    mMotors[4]->setPosition(0.5);  // ArmLowerR (tekuk siku kanan)
    wait(500);

    // Tahap 2: Dorong!
    mMotors[0]->setPosition(1.5); // ArmUpperR (dorong dengan bahu kanan)
    wait(500);

    // Tahap 3: Kembali ke posisi siap
    mMotionManager->playPage(9); // Kembali ke posisi awal
    wait(200);

    // Reset semua counter agar loop berikutnya bisa mendeteksi kondisi baru (jatuh ke depan)
    fup = 0; fdown = 0; fleft = 0; fright = 0;
  }
  // Jatuh Kesamping Kanan
  else if (fright > acc_step) {
    cout << "-------Jatuh Kesamping Kiri, mendorong dengan tangan kiri...-------" << endl;
    
    // Hentikan mode berjalan jika aktif
    mGaitManager->stop();
    
    // --- Urutan Gerakan Tangan Kiri ---
    // Tahap 1: Posisikan lengan untuk mendorong
    mMotors[3]->setPosition(1.0); // ArmUpperL (bahu kiri ke samping)
    mMotors[1]->setPosition(-1.5); // ShoulderL (bahu kiri ke depan)
    mMotors[5]->setPosition(-0.5); // ArmLowerL (tekuk siku kiri)
    wait(500);

    // Tahap 2: Dorong!
    mMotors[3]->setPosition(-1.0); // ArmUpperL (dorong dengan bahu kiri)
    wait(500);

    // Tahap 3: Kembali ke posisi siap
    mMotionManager->playPage(9); // Kembali ke posisi awal
    wait(200);

    // Reset semua counter agar loop berikutnya bisa mendeteksi kondisi baru (jatuh ke depan)
    fup = 0; fdown = 0; fleft = 0; fright = 0;
  }
}
