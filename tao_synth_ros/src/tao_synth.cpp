//////////////////////////////////////////////////////////////////////////////////
// Demonstrate a single string/strand
//////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tao/taodefs.h>


class TaoSynthRos
{
public:
  TaoSynthRos(const bool use_graphics) :
      pos_(0.1),
      force_(0.0)
  {
    const float audio_rate = 44100.0f;
    tao_.reset(new Tao(audio_rate));
    const float decay = 20.0;
    strand_.reset(new TaoString(tao_, "strand",
        TaoPitch(150.0f, TaoPitch::frq), decay));

    // need two channels if going to use stereo L and R
    const size_t num_channels = 2;
    output_.reset(new TaoOutput(tao_, "output", "strand_output", num_channels));

    if (use_graphics)
      tao_->graphics_engine_.reset(new TaoGraphicsEngine(tao_));

    strand_->lockEnds();
    tao_->init();

    force_sub_ = nh_.subscribe("force", 10,
        &TaoSynthRos::forceCallback, this);
    position_sub_ = nh_.subscribe("position", 10,
        &TaoSynthRos::positionCallback, this);

    spin();
  }

  void forceCallback(const std_msgs::Float32ConstPtr& msg)
  {
    force_ = msg->data;
  }

  void positionCallback(const std_msgs::Float32ConstPtr& msg)
  {
    pos_ = msg->data;
  }

  void spinOnce()
  {
    tao_->preUpdate();
    score();
    tao_->postUpdate();
    ros::spinOnce();
  }

  void spin()
  {
    while (ros::ok())
    {
      spinOnce();
    }
  }

  void score()
  {
    if (!tao_->synthesisEngine.isActive())
      return;

    const int nsamples = 44100;
    int samples_second = tao_->synthesisEngine.tick % nsamples;

    if (samples_second == 0)
      std::cout << "time: " << tao_->synthesisEngine.time << "\n";

    if (force_ != 0.0)
    {
      // This should create a graphics point
      (*strand_)(pos_).applyForce(force_);
    }
    // TODO(lucasw) or decay it
    force_ = 0.0;

    // TODO(lwalter) output should be configured and then passed to tao which will
    // process it itself?  Or is there a desire for the user to move around where the sample
    // is extracted from?
    // TODO(lucasw) if output only has one channel, then chR goes nowhere
    // TODO(lucaw) These should create graphics points
    output_->chL((*strand_)(0.2));
    output_->chR((*strand_)(0.5));
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber force_sub_;
  ros::Subscriber position_sub_;

  std::shared_ptr<Tao> tao_;
  std::shared_ptr<TaoString> strand_;
  std::shared_ptr<TaoOutput> output_;

  float force_;
  float pos_;
};

main(int argc, char *argv[]) {
  ros::init(argc, argv, "tao_synth");

  bool use_graphics = true;
  for (int i = 0; i < argc; ++i)
  {
    std::cout << argv[i] << std::endl;
  }
  if ((argc > 1) && (std::string(argv[1]) == "-g"))
  {
    use_graphics = true;
  }

  TaoSynthRos tao_synth_ros(use_graphics);
  ros::spin();
  // strand_example.spin();
}
