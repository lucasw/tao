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

#include <tao/tao.h>
#include <iostream>
#include <stdio.h>

// float &Time = tao.synthesisEngine.time;
// long &Tick = tao.synthesisEngine.tick;

Tao::Tao(const float audio_rate) :
    audioRate(audio_rate),
    synthesisEngine(audio_rate) {
  scoreFunctionPtr = NULL;
  durationFunctionPtr = NULL;

  setScoreDuration(10.0);
}

void Tao::seedRandomNumGen() { synthesisEngine.seedRandomNumGen(); }

void Tao::scoreDurationFunc(float (*functionPtr)(void)) {
  durationFunctionPtr = functionPtr;
}

void Tao::scoreFunc(void (*functionPtr)(void)) {
  scoreFunctionPtr = functionPtr;
}

void Tao::setAudioSampleRate(const float sr) { synthesisEngine.setAudioRate(sr); }

void Tao::setScoreDuration(const float duration) {
  synthesisEngine.scoreDuration = duration;
  synthesisEngine.numSamples =
      (long)(duration * synthesisEngine.modelSampleRate);
}

void Tao::executeScore() {
  if (scoreFunctionPtr)
    (*scoreFunctionPtr)();
}

void Tao::masterTick() {
  if (synthesisEngine.done())
    exit(0);

  synthesisEngine.calculateInstrumentForces();

  if (graphics_engine_ && graphics_engine_->active &&
      (synthesisEngine.tick % graphics_engine_->refreshRate == 0)) {
    graphics_engine_->clearBackBuffer();
    graphics_engine_->pushModelViewMatrix();
    graphics_engine_->rotateAndTranslate();
  }

  if (synthesisEngine.isActive())
    executeScore();

  synthesisEngine.updateDevices();
  synthesisEngine.calculateInstrumentPositions();
  synthesisEngine.Tick();

  if (graphics_engine_ && graphics_engine_->active &&
      (synthesisEngine.tick % graphics_engine_->refreshRate == 0)) {
    graphics_engine_->display();
    graphics_engine_->popModelViewMatrix();
    graphics_engine_->swapBuffers();
    graphics_engine_->flushGraphics();
  }
}

void Tao::run() {

  // it's up to the caller to create the graphics engine 
  if (graphics_engine_)
  {
    graphics_engine_->activate();
    synthesisEngine.pause();
    graphics_engine_->init();
    graphics_engine_->calculateOriginForRotations();
  }

  std::cout << "Sample rate=" << synthesisEngine.audioSampleRate << " Hz"
            << std::endl;
  std::cout << "Score duration=" << synthesisEngine.scoreDuration << " seconds"
            << std::endl;

  seedRandomNumGen();

  while (1) {
    masterTick();
    if (graphics_engine_ && graphics_engine_->active)
      graphics_engine_->display();
  }
}
