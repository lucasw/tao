//////////////////////////////////////////////////////////////////////////////////
// Demonstrate a single string/strand
//////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <queue>
#include <sensor_msgs/ChannelFloat32.h>
#include <std_msgs/Float32.h>
#include <tao/taodefs.h>
#include <tao_synth_ros/AddAssembly.h>
#include <tao_synth_ros/Force.h>
#include <tao_synth_ros/Instrument.h>
#include <tao_synth_ros/Output.h>


class TaoSynthRos
{
public:
  TaoSynthRos() :
      pos_(0.1),
      force_(0.0),
      samples_per_msg_(882),
      // write_output_(false),
      max_time_(0.0),
      audio_rate_(44100.0f),
      update_period_(0.01),
      pt_scale_(0.02)
  {
    ros::param::get("~audio_rate", audio_rate_);
    manager_.reset(new tao::Manager(audio_rate_));

    ros::param::get("~force", force_);
    ros::param::get("~max_time", max_time_);

    // TODO(lucasw) using graphics messes with the timing of synth updates,
    // need to fix that.
    bool use_graphics = false;
    ros::param::get("~use_graphics", use_graphics);
    if (use_graphics)
      manager_->graphics_engine_.reset(new tao::GraphicsEngine(manager_));

    manager_->init();

    ros::param::get("~samples_per_msg", samples_per_msg_);

    force_sub_ = nh_.subscribe("force", 10,
        &TaoSynthRos::forceCallback, this);
    audio_pub_ = nh_.advertise<sensor_msgs::ChannelFloat32>("samples", 30);

    ros::param::get("~use_marker_array", use_marker_array_);
    if (use_marker_array_)
    {
      marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 3);
      marker_timer_ = nh_.createTimer(ros::Duration(1.0 / 10.0f), &TaoSynthRos::updateMarker, this);
    }

    add_assembly_service_ = nh_.advertiseService("add_assembly",
        &TaoSynthRos::addAssembly, this);

    ros::param::get("~update_period", update_period_);
    timer_ = nh_.createTimer(ros::Duration(update_period_), &TaoSynthRos::update, this);
  }

  void forceCallback(const tao_synth_ros::ForceConstPtr& msg)
  {
    force_queue_.push(msg);
  }

  void spinOnce()
  {
    manager_->preUpdate();
    score();
    manager_->postUpdate();
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
      // or need to dial down the fidelity of the synth engine?
    }
    ros::spinOnce();
  }

  void score()
  {
    if (!manager_->synthesisEngine.isActive())
      return;

    if ((max_time_ > 0.0) && (manager_->synthesisEngine.time > max_time_))
      ros::shutdown();

    // TODO(lucasw) this may not be the best model- maybe a force
    // should exist for a certain amount of time, minimum 1 update step.
    // with the current system it is hard to apply a constant force-
    // it will get doubled up some instants and maybe gaps in others.
    while (force_queue_.size() > 0)
    {
      tao_synth_ros::ForceConstPtr force = force_queue_.front();
      if (use_marker_array_)
        force_queue_viz_.push(force);

      if (instruments_.count(force->name) > 0)
      {
        (*instruments_[force->name])(force->x, force->y).applyForce(force->force);
      }
      else
      {
        // maybe the instrument was recently removed and there is a queue
        // of forces to clear out, so maybe shouldn't error spam.
        ROS_ERROR_STREAM(force->name << " not in instruments, can't apply force");
      }
      force_queue_.pop();
    }
    // int samples_second = manager_->synthesisEngine.tick % static_cast<int>(audio_rate_);
    // if (samples_second == 0)
    //   std::cout << "time: " << manager_->synthesisEngine.time << "\n";

    // TODO(lucasw) iterate through queue of forces to apply and apply them
    #if 0
    if (force_ != 0.0)
    {
      // This should create a graphics point
      (*strand_)(pos_).applyForce(force_);
    }
    force_ *= 0.98;
    if (force_ < 1e-6)
      force_ = 0.0;

    // TODO(lucasw) if output only has one channel, then chR goes nowhere
    // TODO(lucaw) These should create graphics points
    float sample_0 = (*strand_)(0.13);
    float sample_1 = (*strand_)(0.58);


    if (write_output_)
    {
      output_->ch(sample_0, 0);
      // output_->chL(sample_0);
      // output_->chR(sample_1);
    }
    #endif

    // TODO(lucasw) for now have a simple even mix of all outputs,
    // later optionally publish them separately.
    float sum = 0;
    for (const auto& output : outputs_)
    {
      if (instruments_.count(output.second.instrument_name) == 0)
      {
        ROS_ERROR_STREAM("no instrument " << output.second.instrument_name
            << " for output " << output.second.name);
        continue;
      }
      const float sample = (*instruments_[output.second.instrument_name])(
          output.second.x, output.second.y);
      sum += sample;
    }

    audio_msg_.values.push_back(sum);
    if (audio_msg_.values.size() == samples_per_msg_)
    {
      audio_pub_.publish(audio_msg_);
      audio_msg_.values.resize(0);
    }

  }

  bool addAssembly(tao_synth_ros::AddAssembly::Request& req,
                     tao_synth_ros::AddAssembly::Response& res)
  {
    res.success = true;
    // TODO(lucasw) need to disconnect all Connections to any
    // object that gets deleted - maybe the destructor could do that?
    // Connections aren't even supported here yet, but otherwise
    // the shared_ptr won't get deleted and the object will hang around
    // until whatever it is connected to gets deleted, it also won't
    // get displayed.  (But will the synth engine get updated?)

    for (const auto instrument : req.instruments)
    {
      std::string message;
      if (!addInstrument(instrument, message))
        res.success = false;
      res.message += ", " + message;
    }
    for (const auto output : req.outputs)
    {
      std::string message;
      if (!addOutput(output, message))
        res.success = false;
      res.message += ", " + message;
    }
    return true;
  }

  bool addInstrument(const tao_synth_ros::Instrument& instrument, std::string& message)
  {
    if (instruments_.count(instrument.name) > 0)
    {
      manager_->synthesisEngine.removeInstrument(instruments_[instrument.name].get());
      instruments_[instrument.name] = nullptr;
      message += "removed existing instance of " + instrument.name;
      ROS_INFO_STREAM(message);
      //
    }
    if (instrument.action == tao_synth_ros::Instrument::ADD)
    {
      if (instrument.type == tao_synth_ros::Instrument::STRAND)
      {
        // TODO(lucasw) if the synth_engine instrument list is upgraded
        // then that can be used instead of a redundant map here.
        instruments_[instrument.name].reset(new tao::String(manager_, instrument.name,
            tao::Pitch(instrument.pitch_x, tao::Pitch::frq), instrument.decay));
        instruments_[instrument.name]->placeAt(instrument.position.x,
                                               instrument.position.y);
        instruments_[instrument.name]->lockEnds();
        ROS_INFO_STREAM("new Strand " << instrument.name << " "
            << instrument.pitch_x << " " << instrument.decay);
      }
      else
      {
        std::stringstream ss;
        ss << "can't add type " << instrument.type << " " << instrument.name;
        ROS_ERROR_STREAM(ss.str());
        message += ss.str();
        return false;
      }
    }
    return true;
  }

  bool addOutput(const tao_synth_ros::Output& output, std::string& message)
  {
    // need two channels if going to use stereo L and R
    // const size_t num_channels = 1;
    // ros::param::get("~write_output", write_output_);
    // if (write_output_)
    //   output_.reset(new tao::Output(manager_, "output", "strand_output", num_channels));
    if (outputs_.count(output.name) > 0)
    {
      message += "removed existing output " + output.name;
    }
    outputs_[output.name] = output;
  }

  void updateMarker(const ros::TimerEvent& event)
  {
    visualization_msgs::MarkerArray marker_array;
    for (const auto& instrument : instruments_)
    {
      displayInstrument(instrument.second, marker_array);
    }

    while (force_queue_viz_.size() > 0)
    {
      displayForce(force_queue_viz_.front(), marker_array);
      force_queue_viz_.pop();
    }
    marker_array_pub_.publish(marker_array);
  }

  bool displayForce(tao_synth_ros::ForceConstPtr& force,
      visualization_msgs::MarkerArray& marker_array)
  {
    if (instruments_.count(force->name) == 0)
      return false;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.pose.orientation.w = 1.0;
    marker.frame_locked = true;

    marker.ns = "tao_synth/force/" + force->name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.06;
    marker.scale.y = 0.08;
    marker.scale.z = 0.0;

    marker.color.r = 0.8;
    marker.color.g = 0.3;
    marker.color.b = 0.3;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(1.0);

    const float x_scale = instruments_[force->name]->getXMax() * pt_scale_;
    float x = instruments_[force->name]->getWorldX() + force->x * x_scale;
    float y = instruments_[force->name]->getWorldY() + force->y * x_scale;
    // TODO(lucasw) z could be the current z position of the string, leave 0 for now

    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    marker.points.push_back(pt);
    pt.z = force->force;
    marker.points.push_back(pt);

    marker_array.markers.push_back(marker);
  }

  void displayInstrument(std::shared_ptr<tao::Instrument> instr,
      visualization_msgs::MarkerArray& marker_array)
  {
    if (!instr)
      return;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.pose.orientation.w = 1.0;
    marker.frame_locked = true;

    marker.ns = "tao_synth/" + instr->getName();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    // TODO(lucasw) id needs to be unique if wanting multiple overlapping waveforms
    marker.lifetime = ros::Duration(0.5);

    float cellPosition;

    // TODO(lucasw) make these configurable
    size_t jstep = 4;
    size_t istep = 4;
    float globalMagnification = 0.4;

    float magnification = globalMagnification * instr->getMagnification();

    marker.pose.position.x = instr->getWorldX();
    marker.pose.position.y = instr->getWorldY();

    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.8;
    marker.color.a = 0.5;

    // draw horizontal lines through rows of cells
    // ROS_INFO_STREAM(instr->getYMax());
    for (size_t j = 0; j < instr->rows.size(); j += jstep) {
      // ROS_INFO_STREAM(j << " " << instr->rows[j].xmax);
      for (size_t i = 0; i < instr->rows[j].cells.size(); i+= istep) {
        // TODO(lucasw) add per point color later
        // if (c->velocityMultiplier < instr->defaultVelocityMultiplier)
        //   glColor3f(0.2, 0.2, 0.2);
        // else
        //  glColor3f(0.0, 0.0, 0.0);
        geometry_msgs::Point pt;
        pt.x = instr->rows[j].offset + i;
        pt.x *= pt_scale_;
        pt.y = j;
        pt.y *= pt_scale_;
        // the amplitude of the cell
        pt.z = (instr->rows[j].cells[i]) * magnification;
        marker.points.push_back(pt);
      }
    }

    marker_array.markers.push_back(marker);
#if 0
    if (instr->ymax > 0) // if instrument is 2D, draw line round perimeter
    {                   // if perimeter is locked make line thicker
      if (instr->perimeterLocked)
        glLineWidth(2.0);
      else
        glLineWidth(1.0);

      glBegin(GL_LINE_STRIP);

      j = 0;

      for (i = 0, c = instr->rows[0].cells; i <= instr->rows[0].xmax; i++, c++)
      // across bottom
      {
        cellPosition = c->position;
        x = instr->worldx + instr->rows[j].offset + i;
        z = cellPosition * magnification;
        y = j + instr->worldy;

        glVertex3f(x, y, z);
      }

      for (j = 0; j <= instr->ymax; j++) // up right
      {
        c = &instr->rows[j].cells[instr->rows[j].xmax];
        cellPosition = c->position;
        x = instr->worldx + instr->rows[j].offset + instr->rows[j].xmax;
        z = cellPosition * magnification;
        y = j + instr->worldy;

        glVertex3f(x, y, z);
      }

      j = instr->ymax;

      for (i = instr->rows[instr->ymax].xmax; i >= 0; i--) // across top
      {
        c = &instr->rows[instr->ymax].cells[i];
        cellPosition = c->position;
        x = instr->worldx + instr->rows[j].offset + i;
        z = cellPosition * magnification;
        y = j + instr->worldy;

        glVertex3f(x, y, z);
      }

      for (j = instr->ymax; j >= 0; j--) // down left
      {
        c = &instr->rows[j].cells[0];
        cellPosition = c->position;
        x = instr->worldx + instr->rows[j].offset;
        z = cellPosition * magnification;
        y = j + instr->worldy;

        glVertex3f(x, y, z);
      }

      glEnd();
    }

    glPointSize(3.0);
    glBegin(GL_POINTS);

    for (j = 0; j <= instr->ymax; j++) // scan cells again to mark any
    {                                 // locked or glued ones

      for (i = 0, c = instr->rows[j].cells; i <= instr->rows[j].xmax; i++, c++) {
        cellPosition = c->position;
        if (c->mode & TAO_CELL_LOCK_MODE) {
          if ((i == 0 || i == instr->rows[j].xmax || j == 0 || j == instr->ymax) &&
              instr->perimeterLocked) // if we're at the instrument's
          {                          // perimeter and it is locked then
            continue;                // don't mark individual locked
          }                          // points as the locked perimeter
                                     // has already been displayed as a
                                     // thicker line.
          glColor3f(0.0f, 0.0f, 0.0f);
          x = instr->worldx + instr->rows[j].offset + i;
          z = cellPosition * magnification;
          y = j + instr->worldy;
          glVertex3f(x, y, z);
        }
      }
    }

    glEnd();

    j = instr->ymax / 2;
    c = &instr->rows[j].cells[instr->xmax];
    cellPosition = c->position;
    x = (GLfloat)(instr->worldx + instr->xmax + 3.0);
    z = (GLfloat)(cellPosition * magnification);
    y = (GLfloat)(j + instr->worldy);

    // std::cout << "x=" << x << " y=" << y << " z=" << z
    //     << " name=" << instr->name << std::endl;

    //displayCharString(x, y, z, instr->name, 0.0, 0.0, 0.0);
    #endif
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber force_sub_;
  std::queue<tao_synth_ros::ForceConstPtr> force_queue_;
  std::queue<tao_synth_ros::ForceConstPtr> force_queue_viz_;

  ros::ServiceServer add_assembly_service_;

  ros::Publisher marker_array_pub_;
  bool use_marker_array_ = false;
  ros::Timer marker_timer_;

  ros::Publisher audio_pub_;
  sensor_msgs::ChannelFloat32 audio_msg_;
  int samples_per_msg_;
  float update_period_;
  ros::Timer timer_;
  float max_time_;

  std::shared_ptr<tao::Manager> manager_;
  std::map<std::string, std::shared_ptr<tao::Instrument> > instruments_;
  std::map<std::string, tao_synth_ros::Output> outputs_;
  // bool write_output_;

  float audio_rate_;
  float force_;
  float pos_;
  float pt_scale_;
};

main(int argc, char *argv[]) {
  ros::init(argc, argv, "tao_synth_ros");
  TaoSynthRos tao_synth_ros;
  ros::spin();
  // strand_example.spin();
}
