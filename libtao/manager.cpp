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

#include <tao/manager.h>
#include <iostream>
#include <stdio.h>

using namespace tao;

Manager::Manager(const float audio_rate) :
    audioRate(audio_rate),
    synthesisEngine(audio_rate) {
  setScoreDuration(0.0);
}

void Manager::seedRandomNumGen() { synthesisEngine.seedRandomNumGen(); }

void Manager::setAudioSampleRate(const float sr) { synthesisEngine.setAudioRate(sr); }

void Manager::setScoreDuration(const float duration) {
  synthesisEngine.scoreDuration = duration;
  synthesisEngine.numSamples =
      (long)(duration * synthesisEngine.modelSampleRate);
}

void Manager::preUpdate() {
  if (synthesisEngine.done())
    exit(0);

  synthesisEngine.calculateInstrumentForces();

  // TODO(lucasw) if all the graphics engine code can be removed
  // from the other files then this pre-update code can go away
  if (graphics_engine_ && graphics_engine_->active &&
      (synthesisEngine.tick % graphics_engine_->refreshRate == 0)) {
    graphics_engine_->clearBackBuffer();
    graphics_engine_->pushModelViewMatrix();
    graphics_engine_->rotateAndTranslate();
  }
}

void Manager::postUpdate() {
  synthesisEngine.updateDevices();
  synthesisEngine.calculateInstrumentPositions();
  synthesisEngine.Tick();

  if (graphics_engine_ && graphics_engine_->active &&
      (synthesisEngine.tick % graphics_engine_->refreshRate == 0)) {
    graphics_engine_->display();
    graphics_engine_->popModelViewMatrix();

    if (glfwWindowShouldClose(graphics_engine_->window_.get())) {
      graphics_engine_->deactivate();
    }
  }
}

void Manager::init() {

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
}
