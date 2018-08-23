//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "test.tao".
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

TaoCircle c("c", TaoPitch(800.000f, TaoPitch::frq), 20.0000f);

TaoString tau_strings[4] = {
    TaoString("tau_strings[0]", TaoPitch(800.000f, TaoPitch::frq), 20.0000f),
    TaoString("tau_strings[1]", TaoPitch(810.000f, TaoPitch::frq), 20.0000f),
    TaoString("tau_strings[2]", TaoPitch(820.000f, TaoPitch::frq), 20.0000f),
    TaoString("tau_strings[3]", TaoPitch(830.000f, TaoPitch::frq), 20.0000f)};

TaoRectangle r("r", TaoPitch(800.000f, TaoPitch::frq),
               TaoPitch(900.000f, TaoPitch::frq), 20.0000f);

TaoTriangle t("t", TaoPitch(800.000f, TaoPitch::frq),
              TaoPitch(900.000f, TaoPitch::frq), 20.0000f);

TaoConnector conn1("conn1"), conn2("conn2"), conn3("conn3"), conn4("conn4");
int s;

// Init: <statements> ...

void taoInit() {
  for (s = 0; s <= 3; s++) {
    tau_strings[s].lockEnds();
  }

  c.lockPerimeter();
  r.lockCorners();
  t.lockLeft().lockRight();
  conn1(tau_strings[0](0.100000f), tau_strings[1](0.100000f));
  conn2(tau_strings[1](0.900000f), tau_strings[2](0.900000f));
  conn3(tau_strings[2](0.100000f), tau_strings[3](0.100000f));
  conn4(r(0.600000f, 0.200000f), 0.00000f);
  r.placeRightOf(c, 20);
  t.placeAbove(r);
}

// Score <duration> : <statements> ...

float taoScoreDuration() { return 20.0000f; }

void taoScore() {
  tao.initStartAndEnd();

  if (Tick <= (long)((tao.newEnd = tao.start * 1.00000 + 0.000100000f) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = tao.start) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    tau_strings[0](0.100000f).applyForce(1.00000f);
    tau_strings[1](0.100000f).applyForce(1.00000f);
    tau_strings[2](0.100000f).applyForce(1.00000f);
    tau_strings[3](0.100000f).applyForce(1.00000f);
    c(0.100000f, 0.500000f).applyForce(10.0000f);
    r(0.700000f, 0.800000f).applyForce(10.0000f);
    t(0.800000f, 0.600000f).applyForce(10.0000f);
    tao.popStartAndEnd();
  }

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

// End of C++ program generated from script "test.tao"
