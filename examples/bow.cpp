//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "bow.tao".
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

#include <tao/taodefs.h>
#include <cmath>
#include <iostream>

Tao tao;

// Audio rate: <sample_rate> ;

int taoAudioRate() { return 44100; }

// Declarations

TaoString tau_string("tau_string", TaoPitch(200.000f, TaoPitch::frq), 30.0000f);

TaoBow bow("bow");

TaoOutput outputs[5] = {TaoOutput("outputs_0", "bow_outputs_0", 1),
                        TaoOutput("outputs_1", "bow_outputs_1", 1),
                        TaoOutput("outputs_2", "bow_outputs_2", 1),
                        TaoOutput("outputs_3", "bow_outputs_3", 1),
                        TaoOutput("outputs_4", "bow_outputs_4", 1)};

// Init: <statements> ...

void taoInit() {
  tau_string.lockEnds();
}

// Score <duration> : <statements> ...

float taoScoreDuration() { return 30.0000f; }

void taoScore() {
  tao.initStartAndEnd();

  if (Tick <= (long)((tao.newEnd = tao.start * 1.00000 + 0.100000f) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = tao.start) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    tau_string(0.100000f).applyForce(1.00000f);
    tao.popStartAndEnd();
  }

  outputs[0].ch1(tau_string(0.100000f));
  outputs[1].ch1(tau_string(0.300000f));
  outputs[2].ch1(tau_string(0.500000f));
  outputs[3].ch1(tau_string(0.700000f));
  outputs[4].ch1(tau_string(0.900000f));

  tao.popStartAndEnd();
}

main(int argc, char *argv[]) {
  tao.initStartAndEnd();
  tao.audioRateFunc(taoAudioRate);
  tao.initFunc(taoInit);
  tao.scoreDurationFunc(taoScoreDuration);
  tao.scoreFunc(taoScore);
  tao.main(argc, argv);
}

// End of C++ program generated from script "bow.tao"
