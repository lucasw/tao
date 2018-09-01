//////////////////////////////////////////////////////////////////////////////////
// Demonstrate a single string/strand
//////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <std_msgs/Float32.h>
#include <tao/taodefs.h>


class TaoSynthRos
{
public:
  TaoSynthRos() :
      pos_(0.1),
      force_(0.0),
      samples_per_msg_(882),
      write_output_(false),
      audio_rate_(44100.0f),
      update_period_(0.01)
  {
    ros::param::get("~audio_rate", audio_rate_);
    tao_.reset(new Tao(audio_rate_));
    const float decay = 20.0;
    strand_.reset(new TaoString(tao_, "strand",
        TaoPitch(150.0f, TaoPitch::frq), decay));

    // need two channels if going to use stereo L and R
    const size_t num_channels = 2;
    ros::param::get("~write_output", write_output_);
    if (write_output_)
      output_.reset(new TaoOutput(tao_, "output", "strand_output", num_channels));

    // TODO(lucasw) using graphics messes with the timing of synth updates,
    // need to fix that.
    bool use_graphics = false;
    ros::param::get("~use_graphics", use_graphics);
    if (use_graphics)
      tao_->graphics_engine_.reset(new TaoGraphicsEngine(tao_));

    strand_->lockEnds();
    tao_->init();

    ros::param::get("~samples_per_msg", samples_per_msg_);

    force_sub_ = nh_.subscribe("force", 10,
        &TaoSynthRos::forceCallback, this);
    position_sub_ = nh_.subscribe("position", 10,
        &TaoSynthRos::positionCallback, this);
    audio_pub_ = nh_.advertise<sensor_msgs::ChannelFloat32>("samples", 30);

    ros::param::get("~update_period", update_period_);
    timer_ = nh_.createTimer(ros::Duration(update_period_), &TaoSynthRos::update, this);
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
  }

  void update(const ros::TimerEvent& event)
  {
    ros::Time t0 = ros::Time::now();
    // TODO(lucasw) instead of update_period use the event time
    for (size_t i = 0; i < audio_rate_ * update_period_; ++i)
    {
      spinOnce();
    }
    ros::Time t1 = ros::Time::now();
    if ((t1 - t0).toSec() > update_period_)
    {
      // TODO(lucasw) need to do fewer updates - but could
      // audio common handle a changing sample rate?
    }
    ros::spinOnce();
  }

  void score()
  {
    if (!tao_->synthesisEngine.isActive())
      return;

    int samples_second = tao_->synthesisEngine.tick % static_cast<int>(audio_rate_);

    // if (samples_second == 0)
    //   std::cout << "time: " << tao_->synthesisEngine.time << "\n";

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
    float sample_l = (*strand_)(0.13);
    float sample_r = (*strand_)(0.58);

    audio_msg_.values.push_back(sample_l);
    if (audio_msg_.values.size() == samples_per_msg_)
    {
      audio_pub_.publish(audio_msg_);
      audio_msg_.values.resize(0);
    }

    if (write_output_)
    {
      output_->chL(sample_l);
      output_->chR(sample_r);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber force_sub_;
  ros::Subscriber position_sub_;
  ros::Publisher audio_pub_;
  sensor_msgs::ChannelFloat32 audio_msg_;
  int samples_per_msg_;
  float update_period_;
  ros::Timer timer_;

  std::shared_ptr<Tao> tao_;
  std::shared_ptr<TaoString> strand_;
  std::shared_ptr<TaoOutput> output_;
  bool write_output_;

  float audio_rate_;
  float force_;
  float pos_;
};

main(int argc, char *argv[]) {
  ros::init(argc, argv, "tao_synth");
  TaoSynthRos tao_synth_ros;
  ros::spin();
  // strand_example.spin();
}
