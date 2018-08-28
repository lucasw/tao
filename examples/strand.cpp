//////////////////////////////////////////////////////////////////////////////////
// Demonstrate a single string/strand
//////////////////////////////////////////////////////////////////////////////////

#include <tao/taodefs.h>
#include <cmath>
#include <iostream>

static std::shared_ptr<Tao> mtao;

static std::shared_ptr<TaoString> tau_string;
static std::shared_ptr<TaoOutput> output;

static float pos = 0.1;
static float mag = 1.0;
void taoScore() {
  bool apply_force = true;

  pos += 0.000001;
  if (apply_force) {
    const int nsamples = 16000;
    float force = mag * (1.0 - float(mtao->synthesisEngine.tick % nsamples) / float(nsamples));
    (*tau_string)(pos).applyForce(force);
    mag *= 1.000001;
  }

  // TODO(lwalter) output should be configured and then passed to tao which will
  // process it itself?  Or is there a desire for the user to move around where the sample
  // is extracted from?
  // TODO(lucasw) if output only has one channel, then chR goes nowhere
  output->chL((*tau_string)(0.2));
  output->chR((*tau_string)(0.5));
}

main(int argc, char *argv[]) {
  const float audio_rate = 44100.0f;
  mtao.reset(new Tao(audio_rate));
  const float decay = 20.0;
  tau_string.reset(new TaoString(mtao, "tau_string",
      TaoPitch(150.0f, TaoPitch::frq), decay));

  // need two channels if going to use stereo L and R
  const size_t num_channels = 2;
  output.reset(new TaoOutput(mtao, "output", "strand_output", num_channels));

  for (int i = 0; i < argc; ++i) {
    std::cout << argv[i] << std::endl;
  }
  if ((argc > 1) && (std::string(argv[1]) == "-g"))
  {
    mtao->graphics_engine_.reset(new TaoGraphicsEngine(mtao));
  }

  mtao->setScoreDuration(5.0);
  tau_string->lockEnds();

  mtao->scoreFunc(taoScore);
  mtao->run();
}
