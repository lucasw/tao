//////////////////////////////////////////////////////////////////////////////////
// This is the translated version of script "tau_string.tao".
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
// of class `Tao'), and then invokes the member function `mtao->main()'. This
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

static std::shared_ptr<Tao> mtao;

int taoAudioRate() { return 44100; }

static std::shared_ptr<TaoString> tau_string;
static std::shared_ptr<TaoOutput> output;

void taoInit() {
  tau_string->lockEnds();
}

float taoScoreDuration() { return 5.0f; }

void taoScore() {
  mtao->initStartAndEnd();

  if (mtao->synthesisEngine.tick <= (long)((mtao->newEnd = 0.001) *
                     mtao->synthesisEngine.modelSampleRate) &&
      mtao->synthesisEngine.tick >= (long)((mtao->newStart = 0.0) *
                     mtao->synthesisEngine.modelSampleRate)) {
    mtao->pushStartAndEnd1();
    (*tau_string)(0.1f).applyForce(
        ((mtao->synthesisEngine.time - mtao->start) / (mtao->end - mtao->start) * (0.0f - 1.0f) +
         1.0f));
    mtao->popStartAndEnd();
  }

  // TODO(lucasw) if output only has one channel, what does L and R do?
  output->chL((*tau_string)(0.2));
  output->chR((*tau_string)(0.8));

  mtao->popStartAndEnd();
}

main(int argc, char *argv[]) {
  mtao.reset(new Tao);
  const float decay = 20.0;
  tau_string.reset(new TaoString(mtao, "tau_string",
      TaoPitch(200.0f, TaoPitch::frq), decay));

  output.reset(new TaoOutput(mtao, "output", "strand_output", 1));

  mtao->initStartAndEnd();
  mtao->audioRateFunc(taoAudioRate);
  mtao->initFunc(taoInit);
  mtao->scoreDurationFunc(taoScoreDuration);
  mtao->scoreFunc(taoScore);
  mtao->main(argc, argv);
}
