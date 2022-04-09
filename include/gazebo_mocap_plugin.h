/**
 * @brief MOCAP Plugin
 *
 * This plugin simulates MOCAP data
 *
 * @author Abhishek Goudar <deloney21@gmail.com>
 */

#ifndef _GAZEBO_MOCAP_PLUGIN_HH_
#define _GAZEBO_MOCAP_PLUGIN_HH_

#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Odometry.pb.h>

namespace gazebo
{
class GAZEBO_VISIBLE MocapPlugin : public ModelPlugin
{
public:
  MocapPlugin();
  virtual ~MocapPlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo&);
  void getSdfParams(sdf::ElementPtr sdf);

private:
  std::string _namespace;
  physics::ModelPtr _model;
  physics::WorldPtr _world;
  event::ConnectionPtr _updateConnection;

  nav_msgs::msgs::Odometry odom_msg;

  transport::NodePtr _nh;
  transport::PublisherPtr _pub_odom;

  common::Time _last_pub_time;
  common::Time _last_time;

  ignition::math::Pose3d _pose_model_start;

  double _pub_rate;
  // vision position estimate noise parameters
  double _corellation_time;
  double _random_walk;
  double _noise_density;

  ignition::math::Vector3d _bias;

  std::default_random_engine _rand;
  std::normal_distribution<float> _randn;
  static constexpr double kDefaultPubRate 		= 30.0;	 // [Hz]
  static constexpr double kDefaultCorrelationTime	= 60.0;	 // [s]
  static constexpr double kDefaultRandomWalk		= 1.0;	 // [(m/s) / sqrt(hz)]
  static constexpr double kDefaultNoiseDensity		= 0.0005;// [(m) / sqrt(hz)]

};     // class GAZEBO_VISIBLE MocapPlugin
}      // namespace gazebo
#endif // _GAZEBO_MOCAP_PLUGIN_HH_
