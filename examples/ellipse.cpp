//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "ellipse.tao".
//
// It contains automatically generated definitions for the following functions
// which are required by the Tao library in order to produce a complete
// executable:
//
//   int    taoAudioRate()	- returns the audio sampling rate in Hz.
//   float  taoScoreDuration()	- returns the duration of the score
//				  in seconds.
//   void   taoInit()		- this function is called just before execution
//				  of the score and contains user specified code
//				  for initialising variable values, devices,
//				  instruments and connections.
//   void   taoScore()		- this function is called once on every tick of
//				  the synthesis engine and contains all the code
//				  representing the user's time-domain inter-
//				  actions with the instruments and devices.
//
// The `main()' function defined at the end of this generated file registers
// the functions described above with the top level object `tao' (an instance
// of class `Tao'), and then invokes the member function `tao.main()'. This
// function enters the main synthesis engine loop which calculates the number
// of ticks specified by the score duration, and updates the graphics window
// (if graphics mode is on). It only exits if the graphics window is closed, if
// the ESC key is pressed whilst the graphics window has the mouse focus, if
// CTRL-C is pressed in the shell window from which Tao was invoked, or the
// `performance' reaches its natural conclusion.
//////////////////////////////////////////////////////////////////////////////////

#include "taodefs.h"
#include <cmath>
#include <iostream>

Tao tao;

// Audio rate: <sample_rate> ;

int taoAudioRate() { return 44100; }

// Declarations

TaoEllipse ellipse("ellipse", TaoPitch(200.000f, TaoPitch::frq),
                   TaoPitch(400.000f, TaoPitch::frq), 20.0000f);

// Init: <statements> ...

void taoInit() {
  ellipse.lockPerimeter();
}

// Score <duration> : <statements> ...

float taoScoreDuration() { return 9.00000f; }

TaoOutput outputs[5] = {TaoOutput("outputs_0", "ellipse_outputs_0", 1),
                        TaoOutput("outputs_1", "ellipse_outputs_1", 1),
                        TaoOutput("outputs_2", "ellipse_outputs_2", 1),
                        TaoOutput("outputs_3", "ellipse_outputs_3", 1),
                        TaoOutput("outputs_4", "ellipse_outputs_4", 1)};


int count = 0;
void taoScore() {
  tao.initStartAndEnd();

  if (Tick <= (long)((tao.newEnd = 0.000500000) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = 0.00000) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    ellipse(0.150000f, 0.500000f)
        .applyForce(((Time - tao.start) / (tao.end - tao.start) *
                         (0.00000f - 30.0000f) +
                     30.0000f));
    tao.popStartAndEnd();
  }

  float y = 0.2;
  for (size_t i = 0; i < 5; ++i) {
    outputs[i].chL(ellipse(0.3, y));
    outputs[i].chR(ellipse(0.7, y));
    y += 0.1;
  }

  tao.popStartAndEnd();

  if (count++ % taoAudioRate() == 0)
  {
    std::cout << count / taoAudioRate() << std::endl;
  }
}

main(int argc, char *argv[]) {
  tao.initStartAndEnd();
  tao.audioRateFunc(taoAudioRate);
  tao.initFunc(taoInit);
  tao.scoreDurationFunc(taoScoreDuration);
  tao.scoreFunc(taoScore);
  tao.main(argc, argv);
}

// End of C++ program generated from script "ellipse.tao"
