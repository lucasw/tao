//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "pitches.tao".
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

TaoString array1[] = {
    TaoString("array1[0]", TaoPitch(200.000f, TaoPitch::frq), 20.0000f),
    TaoString("array1[1]", TaoPitch(220.000f, TaoPitch::frq), 20.0000f),
    TaoString("array1[2]", TaoPitch(240.000f, TaoPitch::frq), 20.0000f),
    TaoString("array1[3]", TaoPitch(260.000f, TaoPitch::frq), 20.0000f)};

TaoString array2[] = {
    TaoString("array2[0]", TaoPitch(8.00000f, TaoPitch::pch), 20.0000f),
    TaoString("array2[1]", TaoPitch(8.04000f, TaoPitch::pch), 20.0000f),
    TaoString("array2[2]", TaoPitch(8.06000f, TaoPitch::pch), 20.0000f),
    TaoString("array2[3]", TaoPitch(8.08000f, TaoPitch::pch), 20.0000f)};

TaoString array3[] = {
    TaoString("array3[0]", TaoPitch(8.00000f, TaoPitch::oct), 20.0000f),
    TaoString("array3[1]", TaoPitch(8.20000f, TaoPitch::oct), 20.0000f),
    TaoString("array3[2]", TaoPitch(8.40000f, TaoPitch::oct), 20.0000f),
    TaoString("array3[3]", TaoPitch(8.60000f, TaoPitch::oct), 20.0000f)};
int n;

// Init: <statements> ...

void taoInit() {
  for (n = 0; n <= 3; n++) {
    array1[n].lockEnds();
    array2[n].lockEnds();
    array3[n].lockEnds();
  }
}

// Score <duration> : <statements> ...

float taoScoreDuration() { return 5.00000f; }

void taoScore() {
  tao.initStartAndEnd();

  tao.graphicsEngine.label(array1[0], 1.00000f, 1.00000f, -1.00000f,
                           "User-defined label!", 0.00000f, 1.00000f, 0.00000f);

  if (Tick <= (long)((tao.newEnd = tao.start * 1.00000 + 0.000100000f) *
                     tao.synthesisEngine.modelSampleRate) &&
      Tick >= (long)((tao.newStart = tao.start) *
                     tao.synthesisEngine.modelSampleRate)) {
    tao.pushStartAndEnd1();
    for (n = 0; n <= 3; n++) {
      array1[n](0.100000f).applyForce(1.00000f);
      array2[n](0.100000f).applyForce(1.00000f);
      array3[n](0.100000f).applyForce(1.00000f);
    }

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

// End of C++ program generated from script "pitches.tao"
