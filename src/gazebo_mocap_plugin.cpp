 /**
  * @brief MOCAP Plugin
  *
  * This plugin simulates MOCAP data
  *
  * @author Abhishek Goudar <deloney21@gmail.com>
  */

#include <gazebo_mocap_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(MocapPlugin)

MocapPlugin::MocapPlugin() : ModelPlugin()
{
}

MocapPlugin::~MocapPlugin()
{
  _updateConnection->~Connection();
}

void MocapPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  _namespace.clear();
  if (sdf->HasElement("robotNamespace")) {
    _namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_vision_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("pubRate")) {
    _pub_rate = sdf->GetElement("pubRate")->Get<int>();
  } else {
    _pub_rate = kDefaultPubRate;
    gzwarn << "[gazebo_vision_plugin] Using default publication rate of " << _pub_rate << " Hz\n";
  }

  // if (sdf->HasElement("corellationTime")) {
  //   _corellation_time = sdf->GetElement("corellationTime")->Get<float>();
  // } else {
  //   _corellation_time = kDefaultCorrelationTime;
  //   gzwarn << "[gazebo_vision_plugin] Using default correlation time of " << _corellation_time << " s\n";
  // }

  // if (sdf->HasElement("randomWalk")) {
  //   _random_walk = sdf->GetElement("randomWalk")->Get<float>();
  // } else {
  //   _random_walk = kDefaultRandomWalk;
  //   gzwarn << "[gazebo_vision_plugin] Using default random walk of " << _random_walk << " (m/s) / sqrt(hz)\n";
  // }

  // if (sdf->HasElement("noiseDensity")) {
  //   _noise_density = sdf->GetElement("noiseDensity")->Get<float>();
  // } else {
  //   _noise_density = kDefaultNoiseDensity;
  //   gzwarn << "[gazebo_vision_plugin] Using default noise density of " << _noise_density << " (m) / sqrt(hz)\n";
  // }
}

void MocapPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  // Store model
  _model = model;

  _world = _model->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 9
  _last_time = _world->SimTime();
  _last_pub_time = _world->SimTime();
  // remember start pose -> VIO should always start with zero
  _pose_model_start = _model->WorldPose();
#else
  _last_time = _world->GetSimTime();
  _last_pub_time = _world->GetSimTime();
  // remember start pose -> VIO should always start with zero
  _pose_model_start = ignitionFromGazeboMath(_model->GetWorldPose());
#endif

  _nh = transport::NodePtr(new transport::Node());
  _nh->Init(_namespace);

  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MocapPlugin::OnUpdate, this, _1));

  _pub_odom = _nh->Advertise<nav_msgs::msgs::Odometry>("~/" + _model->GetName() + "/mocap_odom", 10);
}

void MocapPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = _world->SimTime();
#else
  common::Time current_time = _world->GetSimTime();
#endif
  double dt = (current_time - _last_pub_time).Double();

  if (dt > 1.0 / _pub_rate) {

    // get pose of the model that the plugin is attached to
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose_model_world = _model->WorldPose();
    ignition::math::Vector3d velocity_model_world = _model->WorldLinearVel();
    ignition::math::Vector3d angular_velocity_model = _model->RelativeAngularVel();
#else
    ignition::math::Pose3d pose_model_world = ignitionFromGazeboMath(_model->GetWorldPose());
    ignition::math::Vector3d velocity_model_world = ignitionFromGazeboMath(_model->GetWorldLinearVel());
    ignition::math::Vector3d angular_velocity_model = ignitionFromGazeboMath(_model->GetRelativeAngularVel());
#endif
    ignition::math::Pose3d pose_model; // pose in local frame (relative to where it started)
    pose_model.Pos().X() = pose_model_world.Pos().X();
    pose_model.Pos().Y() = pose_model_world.Pos().Y();
    pose_model.Pos().Z() = pose_model_world.Pos().Z();
    pose_model.Rot().Euler(pose_model_world.Rot().Roll(),
                           pose_model_world.Rot().Pitch(),
                           pose_model_world.Rot().Yaw());

    // update noise parameters
    // ignition::math::Vector3d noise_pos;
    // ignition::math::Vector3d noise_linvel;
    // ignition::math::Vector3d noise_angvel;

    // // position noise model
    // noise_pos.X() = _noise_density * sqrt(dt) * _randn(_rand);
    // noise_pos.Y() = _noise_density * sqrt(dt) * _randn(_rand);
    // noise_pos.Z() = _noise_density * sqrt(dt) * _randn(_rand);

    // // velocity noise model
    // noise_linvel.X() = _noise_density * sqrt(dt) * _randn(_rand);
    // noise_linvel.Y() = _noise_density * sqrt(dt) * _randn(_rand);
    // noise_linvel.Z() = _noise_density * sqrt(dt) * _randn(_rand);

    // // angular rates noise model
    // double tau_g = _corellation_time;
    // double sigma_g_d = 1 / sqrt(dt) * _noise_density;
    // double sigma_b_g = _random_walk;
    // double sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (exp(-2.0 * dt / tau_g) - 1.0));
    // double phi_g_d = exp(-1.0 / tau_g * dt);

    // noise_angvel.X() = phi_g_d * noise_angvel.X() + sigma_b_g_d * sqrt(dt) * _randn(_rand);
    // noise_angvel.Y() = phi_g_d * noise_angvel.Y() + sigma_b_g_d * sqrt(dt) * _randn(_rand);
    // noise_angvel.Z() = phi_g_d * noise_angvel.Z() + sigma_b_g_d * sqrt(dt) * _randn(_rand);

    // // random walk generation
    // ignition::math::Vector3d random_walk;
    // random_walk.X() = _random_walk * sqrt(dt) * _randn(_rand);
    // random_walk.Y() = _random_walk * sqrt(dt) * _randn(_rand);
    // random_walk.Z() = _random_walk * sqrt(dt) * _randn(_rand);

    // // bias integration
    // _bias.X() += random_walk.X() * dt - _bias.X() / _corellation_time;
    // _bias.Y() += random_walk.Y() * dt - _bias.Y() / _corellation_time;
    // _bias.Z() += random_walk.Z() * dt - _bias.Z() / _corellation_time;

    // Fill odom msg
    odom_msg.set_time_usec(current_time.Double() * 1e6);

    gazebo::msgs::Vector3d* position = new gazebo::msgs::Vector3d();
    position->set_x(pose_model.Pos().X();// + noise_pos.X() + _bias.X());
    position->set_y(pose_model.Pos().Y();// + noise_pos.Y() + _bias.Y());
    position->set_z(pose_model.Pos().Z();// + noise_pos.Z() + _bias.Z());
    odom_msg.set_allocated_position(position);

    ignition::math::Quaterniond pose_model_quaternion = pose_model.Rot();
    gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
    orientation->set_x(pose_model_quaternion.X());
    orientation->set_y(pose_model_quaternion.Y());
    orientation->set_z(pose_model_quaternion.Z());
    orientation->set_w(pose_model_quaternion.W());
    odom_msg.set_allocated_orientation(orientation);

    gazebo::msgs::Vector3d* linear_velocity = new gazebo::msgs::Vector3d();
    linear_velocity->set_x(velocity_model_world.X();// + noise_linvel.X());
    linear_velocity->set_y(velocity_model_world.Y();// + noise_linvel.Y());
    linear_velocity->set_z(velocity_model_world.Z();// + noise_linvel.Z());
    odom_msg.set_allocated_linear_velocity(linear_velocity);

    gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
    angular_velocity->set_x(angular_velocity_model.X();// + noise_angvel.X());
    angular_velocity->set_y(angular_velocity_model.Y();// + noise_angvel.Y());
    angular_velocity->set_z(angular_velocity_model.Z();// + noise_angvel.Z());
    odom_msg.set_allocated_angular_velocity(angular_velocity);

    // for (int i = 0; i < 36; i++){
    //   switch (i){
    //     // principal diagonal = the variance of the random variables
    //     // = noise_density²
    //     case 0: case 7: case 14: case 21: case 28: case 35:
    //       odom_msg.add_pose_covariance(_noise_density * _noise_density);
    //       odom_msg.add_velocity_covariance(_noise_density * _noise_density);
    //       break;
    //     default:
    //       odom_msg.add_pose_covariance(0.0);
    //       odom_msg.add_velocity_covariance(0.0);
    //   }
    // }
    for (int i = 0; i < 36; i++){
      switch (i){
        // principal diagonal = the variance of the random variables
        // = noise_density²
        case 0: case 7: case 14: case 21: case 28: case 35:
          odom_msg.add_pose_covariance(0.000000001);
          odom_msg.add_velocity_covariance(0.000000001);
          break;
        default:
          odom_msg.add_pose_covariance(0.0);
          odom_msg.add_velocity_covariance(0.0);
      }
    }

    _last_pub_time = current_time;

    // publish odom msg
    _pub_odom->Publish(odom_msg);
  }
}
} // namespace gazebo
