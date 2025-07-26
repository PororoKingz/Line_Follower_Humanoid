#include "control.hpp"

#include <GaitManager.hpp>
#include <MotionPlayManager.hpp>
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

  mMotionManager = new MotionPlayManager(this);
  mGaitManager = new GaitManager(this, "config.ini");


  mStartTime = 0.0;
  mStopwatchRunning = false;
}

Walk::~Walk() {
if (trajectoryLogFile.is_open()) {
    trajectoryLogFile.close();
  }
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

double get_line_position(const unsigned char *image, int width, int height) {
    // ... (salin kode dari bagian 2 di sini)
    double line_pos_sum = 0;
    int line_pixel_count = 0;
    int start_y = height - 10;
    if (start_y < 0) start_y = 0;
    for (int y = start_y; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int intensity = image[y * width + x];
            if (intensity < 64) {
                line_pos_sum += (x - (double)width / 2.0);
                line_pixel_count++;
            }
        }
    }
    if (line_pixel_count == 0) {
        return 0.0;
    }
    return line_pos_sum / line_pixel_count;
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

  while (true) {
    checkIfFallen();

    mMotors[19]->setPosition(-0.6);

    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    mGaitManager->setYAmplitude(0.0);

    // Flag untuk mendeteksi apakah tombol gerakan ditekan pada step ini
    bool isMovingThisStep = false;

    // get keyboard key
    int key = 0;
    while ((key = mKeyboard->getKey()) >= 0) {
      switch (key) {
        case ' ':  // Space bar
          if (loopplay) {
            mGaitManager->stop();
            loopplay = false;
            wait(200);
          } else {
            mGaitManager->start();
            loopplay = true;
            wait(200);
          }
          break;
        case Keyboard::UP:
          mGaitManager->setXAmplitude(1.0);
          isMovingThisStep = true; // Tandai ada gerakan
          break;
        case Keyboard::DOWN:
          mGaitManager->setXAmplitude(-1.0);
          isMovingThisStep = true; // Tandai ada gerakan
          break;
        case Keyboard::RIGHT:
          mGaitManager->setAAmplitude(-0.5);
          isMovingThisStep = true; // Tandai ada gerakan
          break;
        case Keyboard::LEFT:
          mGaitManager->setAAmplitude(0.5);
          isMovingThisStep = true; // Tandai ada gerakan
          break;
        case 'Q':
          mGaitManager->setYAmplitude(0.5);
          isMovingThisStep = true; // Tandai ada gerakan
          break;
        case 'E':
          mGaitManager->setYAmplitude(-0.5);
          isMovingThisStep = true; // Tandai ada gerakan
          break;
      }
    }

    const unsigned char *image = mCamera->getImage();
    if (!image) {
      std::cout << "Gagal mengambil gambar" << std::endl;
      continue;
    }

  int width = mCamera->getWidth();
  int height = mCamera->getHeight();

    // 3. Proses gambar untuk mendapatkan posisi garis
    double line_position = get_line_position(image, width, height);

    // 4. Tentukan aksi berdasarkan posisi garis
    // Threshold untuk menentukan belokan, bisa disesuaikan
    double turn_threshold = width / 6.0;

    if (std::abs(line_position) < turn_threshold / 4) {
      // Garis ada di tengah, jalan lurus
      std::cout << "Status: Jalan Lurus" << std::endl;
      // panggil fungsi jalan lurus Anda
      // walk_forward(); 
    } else if (line_position < -turn_threshold) {
      // Garis terlalu ke kiri, belok kiri
      std::cout << "Status: Belok Kiri" << std::endl;
      // panggil fungsi belok kiri Anda
      // turn_left(); 
    } else if (line_position > turn_threshold) {
      // Garis terlalu ke kanan, belok kanan
      std::cout << "Status: Belok Kanan" << std::endl;
      // panggil fungsi belok kanan Anda
      // turn_right();
    } else {
        // Di antara lurus dan belok, bisa tetap lurus atau belok perlahan
        std::cout << "Status: Menyesuaikan..." << std::endl;
        // walk_forward();
    }
    
    // Jika tidak ada garis terdeteksi (line_position == 0 dari fungsi)
    // Anda bisa tambahkan logika untuk berhenti atau berputar
    if (line_position == 0.0) {
       // stop();
       std::cout << "Status: Garis Hilang!" << std::endl;
    }
  
    

    mGaitManager->step(mTimeStep);

    // step
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
