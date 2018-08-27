/* Tao - A software package for sound synthesis with physical models
 * Copyright (C) 1993-1999 Mark Pearson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef OUTPUT_H
#define OUTPUT_H

#include <tao/device.h>
#include <fstream>
#include <sstream>
#include <vector>

#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

class TaoSynthEngine;

#define stereo 2
#define mono 1

class TaoDevice;

class DLLEXPORT TaoOutput : public TaoDevice {
  friend class TaoSynthEngine;

public:
  TaoOutput(std::shared_ptr<Tao> tao);
  ~TaoOutput();
  TaoOutput(std::shared_ptr<Tao> tao, const std::string filename, size_t channels);
  TaoOutput(std::shared_ptr<Tao> tao, const std::string outputName, const std::string filename, size_t channels);
  inline void ch(const float value, const size_t index) {
    if (index < samples.size())
      samples[index] = value;
    // else throw?
  }
  inline void chL(const float value) { ch(value, 0); }
  inline void chR(const float value) { ch(value, 1); }
  void update();
  void display();

private:
  static const int buffersize;
  int first_write;
  int index;
  std::vector<float> buffer;
  std::string fullfilename;
  std::ofstream *outputfile;
  std::vector<float> samples;
  float maxSample;
  std::string displayString;
  std::ostringstream *displayStream;
  static float displayPosition;
  float myDisplayPosition;
};

#endif
