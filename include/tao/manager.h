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

#ifndef TAO_H
#define TAO_H

extern "C" {
#include <stdlib.h>
#include <unistd.h>
}

#include <tao/graphics_engine.h>
#include <tao/synth_engine.h>

namespace tao {
class Manager { // : std::enable_shared_from_this<Manager> {
  friend class Output; // necessary because a Output object must
                          // be able to find out the audio sample rate
public:
  Manager(const float audio_rate = 44100.0f);
  void seedRandomNumGen();
  void setAudioSampleRate(const float sr);
  void setScoreDuration(const float duration);
  void executeScore();
  void init();
  void preUpdate();
  void postUpdate();

  SynthEngine synthesisEngine;
  std::shared_ptr<GraphicsEngine> graphics_engine_;

private:
  // TODO(lucasw) should audio rate be able to change dynamically?
  float audioRate;
  float scoreDuration;

  // using std::enable_shared_from_this<Manager>::shared_from_this;

};
}
#endif
