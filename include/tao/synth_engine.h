/* TaoSynth - A software package for sound synthesis with physical models
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

#ifndef TAOSYNTHENGINE_H
#define TAOSYNTHENGINE_H

//#include <iostream> <iomanip>
//#include <string>
//#include <cmath>
//#include <ctime>

#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

namespace tao {
class Instrument;
class Device;

class DLLEXPORT SynthEngine {
  friend class GraphicsEngine;
  friend class Manager;

public:
  Instrument *instrumentList, *currentInstrument;
  Device *deviceList, *currentDevice;

  float time;
  long tick;
  long numSamples;
  float scoreDuration;
  float audioSampleRate;
  const int throwAway;
  float modelSampleRate;
  float Decay2VelocityMultiplierConst;
  float Hz2CellConst;

  SynthEngine(const float audio_rate);
  void pause();
  void unpause();
  int isActive();
  int done();
  void Tick();
  unsigned int getTime();
  void seedRandomNumGen();
  void setAudioRate(const float audioRate);
  void makeTheInstruments();
  void addInstrument(Instrument &instr);
  void removeInstrument(Instrument &instr);
  void addInstrument(Instrument *instr);
  void removeInstrument(Instrument *instr);
  void addDevice(Device &device);
  void removeDevice(Device &device);
  void addDevice(Device *device);
  void removeDevice(Device *device);
  void calculateInstrumentForces();
  void calculateInstrumentPositions();
  void updateDevices();

private:
  int active;
};
}
extern int randomi(int low, int high);
extern float randomf(float low, float high);

#endif
