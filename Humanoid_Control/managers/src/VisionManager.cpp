// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "VisionManager.hpp"

using namespace Robot;
using namespace managers;
using namespace std;

VisionManager::VisionManager(int width, int height, int hue, int hueTolerance, int minSaturation,
                                                 int minValue, int minPercent, int maxPercent) {
  mFinder = new ColorFinder(hue, hueTolerance, minSaturation, minValue, minPercent, maxPercent);
  mBuffer = new FrameBuffer(width, height);
}

VisionManager::~VisionManager() {
  delete mFinder;
  delete mBuffer;
}

bool VisionManager::getBallCenter(double &x, double &y, const unsigned char *image) {
  Point2D pos;

  // Put the image in mBuffer
  mBuffer->m_BGRAFrame->m_ImageData = const_cast<unsigned char *>(image);
  // Convert the image from BGRA format to HSV format
  ImgProcess::BGRAtoHSV(mBuffer);
  // Extract position of the ball from HSV verson of the image
  pos = mFinder->GetPosition(mBuffer->m_HSVFrame);

  if (pos.X == -1 && pos.Y == -1) {
    x = 0.0;
    y = 0.0;
    return false;
  } else {
    x = pos.X;
    y = pos.Y;
    return true;
  }
}

// --- IMPLEMENTASI FUNGSI BARU ---
double VisionManager::getLinePosition(const unsigned char *image, int width, int height) {
    double line_pos_sum = 0;
    int line_pixel_count = 0;

    // Hanya proses 10 baris piksel terakhir di bagian bawah gambar
    int start_y = height - 10;
    if (start_y < 0) start_y = 0;

    for (int y = start_y; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Mengambil nilai intensitas dari channel biru (cukup untuk grayscale)
            int intensity = image[(y * width + x) * 4]; 
            
            // Anggap piksel dengan intensitas di bawah 64 adalah bagian dari garis hitam
            if (intensity < 64) {
                line_pos_sum += (x - (double)width / 2.0);
                line_pixel_count++;
            }
        }
    }

    if (line_pixel_count == 0) {
        // Mengembalikan nilai besar yang tidak mungkin (flag bahwa garis tidak ditemukan)
        return -9999; 
    }

    return line_pos_sum / line_pixel_count;
}

bool VisionManager::isDetected(int x, int y) {
  if (x > mFinder->m_result->m_Width || y > mFinder->m_result->m_Height)
    return false;

  int i = y * mFinder->m_result->m_Width + x;

  if (mFinder->m_result->m_ImageData[i] == 1)
    return true;
  else
    return false;
}
