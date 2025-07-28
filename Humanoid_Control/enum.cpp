#include "control.hpp"

#include <GaitManager.hpp>
#include <MotionPlayManager.hpp>
#include <VisionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Joystick.hpp> // Sertakan header Joystick
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "Walking.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();
  mLineLostCounter = 0;
  mControlMode = LINE_FOLLOWER; // Mode awal adalah line follower

  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  mCamera = getCamera("Camera");
  mCamera->enable(mTimeStep);
  int width = mCamera->getWidth();
  int height = mCamera->getHeight();

  mGyro = getGyro("Gyro");
  mGyro->enable(mTimeStep);

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);

  // Inisialisasi Joystick
  mJoystick = getJoystick();
  mJoystick->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }

  mVisionManager = new VisionManager(width, height, 0, 180, 0, 30, 0.1, 30.0);
  mMotionManager = new MotionPlayManager(this);
  mGaitManager = new GaitManager(this, "config.ini");
}

Walk::~Walk() {
  delete mVisionManager;
  delete mMotionManager;
  delete mGaitManager;
}

void Walk::myStep() {
  if (step(mTimeStep) == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// Fungsi baru untuk menangani logika line follower
void Walk::handleLineFollowing() {
  const unsigned char *image = mCamera->getImage();
  if (!image) return;

  int width = mCamera->getWidth();
  int height = mCamera->getHeight();

  double line_position = mVisionManager->getLinePosition(image, width, height);
  bool line_detected = (line_position != -9999);

  if (!line_detected) {
    mLineLostCounter++;
    cout << "Status: Garis Hilang! Mencari... (" << mLineLostCounter << ")" << endl;
    mGaitManager->setXAmplitude(0.0);
    if (mLineLostCounter <= 100) mGaitManager->setAAmplitude(0.5);
    else if (mLineLostCounter <= 300) mGaitManager->setAAmplitude(-0.5);
    else mLineLostCounter = 0;
  } else {
    mLineLostCounter = 0;
    double error = line_position;
    double turn_threshold = width / 6.0;

    if (std::abs(error) < turn_threshold) {
      cout << "Status: Jalan Lurus (Posisi: " << error << ")" << endl;
      mGaitManager->setXAmplitude(1.0);
      mGaitManager->setAAmplitude(-error * 0.005);
    } else if (error < 0) {
      cout << "Status: Belok Kiri (Posisi: " << error << ")" << endl;
      mGaitManager->setXAmplitude(0.8);
      mGaitManager->setAAmplitude(0.4);
    } else {
      cout << "Status: Belok Kanan (Posisi: " << error << ")" << endl;
      mGaitManager->setXAmplitude(0.8);
      mGaitManager->setAAmplitude(-0.4);
    }
  }
}

// Fungsi baru untuk menangani input keyboard
void Walk::handleKeyboard() {
  mGaitManager->setXAmplitude(0.0);
  mGaitManager->setAAmplitude(0.0);
  mGaitManager->setYAmplitude(0.0);
  
  int key = 0;
  while((key = mKeyboard->getKey()) >= 0) {
    switch (key) {
      case Keyboard::UP:    mGaitManager->setXAmplitude(1.0); break;
      case Keyboard::DOWN:  mGaitManager->setXAmplitude(-1.0); break;
      case Keyboard::RIGHT: mGaitManager->setAAmplitude(-0.5); break;
      case Keyboard::LEFT:  mGaitManager->setAAmplitude(0.5); break;
      case 'Q':             mGaitManager->setYAmplitude(0.5); break;
      case 'E':             mGaitManager->setYAmplitude(-0.5); break;
    }
  }
}

double map_joystick_axis(int value) {
  // Nilai maksimum absolut dari joystick. Anda bisa sesuaikan jika perlu.
  const double MAX_JOYSTICK_VALUE = 32768.0;
  
  // Lakukan pembagian untuk normalisasi
  double normalized_value = static_cast<double>(value) / MAX_JOYSTICK_VALUE;

  // Pastikan nilai tidak melebihi rentang -1.0 hingga 1.0
  if (normalized_value > 1.0) return 1.0;
  if (normalized_value < -1.0) return -1.0;
  
  return normalized_value;
}

double map_joystick_axis23(int value) {
  // Nilai maksimum absolut dari joystick. Anda bisa sesuaikan jika perlu.
  const double MAX_JOYSTICK_VALUE = 32768.0;
  
  // Lakukan pembagian untuk normalisasi
  double normalized_value = static_cast<double>(value) / MAX_JOYSTICK_VALUE;

  // Pastikan nilai tidak melebihi rentang -1.0 hingga 1.0
  if (normalized_value > 0.5) return 0.5;
  if (normalized_value < -0.5) return -0.5;
  
  return normalized_value;
}

void Walk::handleJoystick() {
  // Dapatkan nilai mentah dari joystick
  int raw_foward = mJoystick->getAxisValue(0); // Maju/Mundur (Stik Kiri Vertikal)
  int raw_side   = mJoystick->getAxisValue(1); // Geser Kiri/Kanan (Stik Kiri Horizontal)
  int raw_turn   = mJoystick->getAxisValue(2); // Putar Kiri/Kanan (Stik Kanan Horizontal)

  // Petakan nilai mentah ke rentang -1.0 hingga 1.0
  double foward = -map_joystick_axis(raw_foward); // Diberi negatif agar stik ke depan = maju
  double side   = map_joystick_axis23(raw_side);
  double turn   = -map_joystick_axis23(raw_turn);   // Diberi negatif agar stik ke kanan = putar kanan

  // Terapkan dead-zone untuk menghindari gerakan kecil saat stik diam
  if (std::abs(foward) < 0.1) foward = 0.0;
  if (std::abs(side) < 0.1) side = 0.0;
  if (std::abs(turn) < 0.1) turn = 0.0;
  
  // Atur gerakan robot menggunakan nilai yang sudah dipetakan
  mGaitManager->setXAmplitude(foward);
  mGaitManager->setYAmplitude(side);
  mGaitManager->setAAmplitude(turn);
}


void Walk::run() {
  myStep();
  mMotionManager->playPage(9);
  wait(200);

  bool loopplay = false;
  cout << "Mode Kontrol: Line Follower" << endl;

  while (true) {
    checkIfFallen();
    myStep(); // Pindahkan myStep() ke awal loop untuk update sensor

    // --- Logika Pergantian Mode ---
    int key = mKeyboard->getKey();
    if (key == 'M') { // 'M' untuk Mode
      if (mControlMode == LINE_FOLLOWER) {
        mControlMode = MANUAL_KEYBOARD;
        cout << "Mode Kontrol: Manual Keyboard" << endl;
      } else if (mControlMode == MANUAL_KEYBOARD) {
        mControlMode = MANUAL_JOYSTICK;
        cout << "Mode Kontrol: Manual Joystick" << endl;
      } else {
        mControlMode = LINE_FOLLOWER;
        cout << "Mode Kontrol: Line Follower" << endl;
      }
      // Beri jeda agar tidak langsung membaca input lain
      wait(200); 
    } else if (key == ' ') {
      if (loopplay) {
        mGaitManager->stop();
        loopplay = false;
      } else {
        mGaitManager->start();
        loopplay = true;
      }
      wait(200);
    }

    // --- Jalankan Logika Sesuai Mode ---
    if (loopplay) {
      if (!mGaitManager->isWalking()) mGaitManager->start();

      switch (mControlMode) {
        case LINE_FOLLOWER:
          mMotors[19]->setPosition(-0.6); // Arahkan kepala ke bawah
          handleLineFollowing();
          break;
        case MANUAL_KEYBOARD:
          mMotors[19]->setPosition(0.0); // Arahkan kepala lurus
          handleKeyboard();
          break;
        case MANUAL_JOYSTICK:
          mMotors[19]->setPosition(0.0); // Arahkan kepala lurus
          handleJoystick();
          break;
      }
      mGaitManager->step(mTimeStep);
    }
  }
}

// ... (sisa kode checkIfFallen tetap sama)
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
