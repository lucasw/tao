//////////////////////////////////////////////////////////////////////////////////
// Demonstrate a single string/strand
//////////////////////////////////////////////////////////////////////////////////

#include <tao/taodefs.h>
#include <cmath>
#include <iostream>

class StrandExample
{
public:
  StrandExample(const bool use_graphics) :
      pos(0.1),
      mag(1.0) {

    const float audio_rate = 44100.0f;
    manager_.reset(new tao::Manager(audio_rate));
    const float decay = 20.0;
    strand.reset(new tao::String(manager_, "strand",
        tao::Pitch(150.0f, tao::Pitch::frq), decay));

    // need two channels if going to use stereo L and R
    const size_t num_channels = 2;
    output.reset(new tao::Output(manager_, "output", "strand_output", num_channels));

    if (use_graphics)
      manager_->graphics_engine_.reset(new tao::GraphicsEngine(manager_));

    manager_->setScoreDuration(1.0);
    strand->lockEnds();
    manager_->init();
  }

  void spin() {
    while (true) {
      manager_->preUpdate();
      score();
      manager_->postUpdate();
    }
  }

  void score() {
    if (!manager_->synthesisEngine.isActive())
      return;

    const int nsamples = 44100;
    int samples_second = manager_->synthesisEngine.tick % nsamples;

    if (samples_second == 0)
      std::cout << "time: " << manager_->synthesisEngine.time << "\n";

    bool apply_force = true;

    // pos += 0.000001;
    if (apply_force) {
      // float force = mag * (1.0 - float(samples_second) / float(nsamples));
      // This should create a graphics point
      (*strand)(pos).applyForce(mag);
      mag = 0.0;
      // mag *= 1.000001;
    }

    // TODO(lwalter) output should be configured and then passed to tao which will
    // process it itself?  Or is there a desire for the user to move around where the sample
    // is extracted from?
    // TODO(lucasw) if output only has one channel, then chR goes nowhere
    // TODO(lucaw) These should create graphics points
    output->chL((*strand)(0.2));
    output->chR((*strand)(0.5));
  }

private:
  std::shared_ptr<tao::Manager> manager_;
  std::shared_ptr<tao::String> strand;
  std::shared_ptr<tao::Output> output;

  float pos;
  float mag;
};

main(int argc, char *argv[]) {
  bool use_graphics = false;
  for (int i = 0; i < argc; ++i) {
    std::cout << argv[i] << std::endl;
  }
  if ((argc > 1) && (std::string(argv[1]) == "-g"))
  {
    use_graphics = true;
  }

  StrandExample strand_example(use_graphics);
  strand_example.spin();
}
