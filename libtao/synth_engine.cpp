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

#include <tao/synth_engine.h>
#include <tao/device.h>
#include <tao/instrument.h>

#ifdef LINUX
extern int random();
#endif

#ifdef IRIX
extern long random();
#endif

using namespace tao;

SynthEngine::SynthEngine(const float audio_rate) : throwAway(2) {
  active = TRUE;
  instrumentList = NULL;
  deviceList = NULL;
  time = 0.0;
  tick = 0;
  setAudioRate(audio_rate);
}

void SynthEngine::pause() { active = FALSE; }

void SynthEngine::unpause() { active = TRUE; }

int SynthEngine::isActive() { return active; }

int SynthEngine::done() { return ((numSamples > 0) && (tick > numSamples)); }

void SynthEngine::Tick() {
  if (!active)
    return;

  tick++;
  time = (float)tick / (float)modelSampleRate;
}

unsigned int SynthEngine::getTime() { return 0u; }

void SynthEngine::seedRandomNumGen() { srand(getTime()); }

void SynthEngine::setAudioRate(const float audioRate) {
  audioSampleRate = audioRate;
  modelSampleRate = audioRate * throwAway;
  Decay2VelocityMultiplierConst = (.00012 * 44100.0) / audioRate;
  Hz2CellConst = 24000.0f * audioRate / 44100.0f;
}

void SynthEngine::makeTheInstruments() {
  Instrument *i = instrumentList;

  while (i) {
    i->createTheMaterial();
    i = i->next;
  }
}

void SynthEngine::addInstrument(Instrument &instr) {
  if (instrumentList == NULL)
    instrumentList = &instr;
  else {
    currentInstrument->next = &instr;
    instr.placeAbove(*currentInstrument);
  }
  currentInstrument = &instr;
}

void SynthEngine::removeInstrument(Instrument &instr) {
  if (instrumentList && instrumentList == &instr) {
    instrumentList = instrumentList->next;
    return;
  }

  for (Instrument *i = instrumentList; (i && i->next); i = i->next)
    if (i->next == &instr)
      i->next = i->next->next;
}

void SynthEngine::addInstrument(Instrument *instr) {
#ifdef SYNTHENGINE_DEBUG
  std::cout << "before addInstrument, current=" << currentInstrument
            << " new=" << instr << std::endl;
#endif

  if (instrumentList == NULL)
    instrumentList = instr;
  else {
    currentInstrument->next = instr;
    instr->placeAbove(*currentInstrument);
  }
  currentInstrument = instr;

#ifdef SYNTHENGINE_DEBUG
  std::cout << "after addInstrument, current=" << currentInstrument
            << " new=" << instr << std::endl;
#endif
}

void SynthEngine::removeInstrument(Instrument *instr) {
  if (instrumentList && instrumentList == instr) {
    instrumentList = instrumentList->next;
    return;
  }

  for (Instrument *i = instrumentList; (i && i->next); i = i->next)
    if (i->next == instr)
      i->next = i->next->next;
}

void SynthEngine::addDevice(Device &device) {
  if (deviceList == NULL)
    deviceList = &device;
  else
    currentDevice->next = &device;
  currentDevice = &device;
}

void SynthEngine::removeDevice(Device &device) {
  if (deviceList && deviceList == &device) {
    deviceList = deviceList->next;
    return;
  }

  for (Device *d = deviceList; (d && d->next); d = d->next)
    if (d->next == &device)
      d->next = d->next->next;
}

void SynthEngine::addDevice(Device *device) {
  if (deviceList == NULL)
    deviceList = device;
  else
    currentDevice->next = device;
  currentDevice = device;
}

void SynthEngine::removeDevice(Device *device) {
  if (deviceList && deviceList == device) {
    deviceList = deviceList->next;
    return;
  }

  for (Device *d = deviceList; (d && d->next); d = d->next)
    if (d->next == device)
      d->next = d->next->next;
}

//////////////////////////////////////////////////////////////////////////////
// Member function names:
//	calculateInstrumentForces(),
//	updateInstrumentPositions(),
//
// Functionality:
//	Cause all instruments to be updated by scanning the linked list
//	and calling the appropriate member functions for each instrument.
//
// Local variable:
//	i: current instrument.
//////////////////////////////////////////////////////////////////////////////

void SynthEngine::calculateInstrumentForces() {
  if (!active)
    return;

  for (Instrument *i = instrumentList; i; i = i->next)
    i->calculateForces(0, i->ymax);
}

void SynthEngine::calculateInstrumentPositions() {
  if (!active)
    return;

  for (Instrument *i = instrumentList; i; i = i->next)
    i->calculatePositions(0, i->ymax);
}

void SynthEngine::updateDevices() {
  if (!active)
    return;

  for (Device *d = deviceList; d; d = d->next) {
    d->update();
  }
}

//////////////////////////////////////////////////////////////////////////////
// Global function name:
//	randomi(int low, int high)
//
// Functionality:
//	Returns a random integer between low and high inclusive.
//////////////////////////////////////////////////////////////////////////////

int randomi(int low, int high) { return (low + (rand() % (high - low + 1))); }

//////////////////////////////////////////////////////////////////////////////
// Global function name:
//	randomf(float low, float high)
//
// Functionality:
//	Returns a random floating point number between low and high inclusive.
//////////////////////////////////////////////////////////////////////////////

float randomf(float low, float high) {
  return (float)randomi((int)(low * 100000), (int)(high * 100000)) / 100000.0f;
}
