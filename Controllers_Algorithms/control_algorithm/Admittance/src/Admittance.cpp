#include "Admittance/Admittance.h"
#include "Behavior/KneeBucklingBehavior.h"
#include "Behavior/SeatSlidingBehavior.h"
#include "Behavior/OutOfLineStandBehavior.h"
#include "Behavior/OutOfLineStSBackBehavior.h"
#include "Behavior/OutOfLineStSSideBehavior.h"
#include "Behavior/StSBehavior.h"

#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

Admittance::Admittance(ros::NodeHandle &n,
    double frequency,
    std::string topic_arm_state,
    std::string topic_arm_command,
    std::string topic_wrench_state,
    std::vector<double> M,
    std::vector<double> D,
    std::vector<double> K,
    std::vector<double> desired_pose,
    double arm_max_vel,
    double arm_max_acc,
    double arm_max_ang_vel,
    double arm_max_ang_acc,
    std::vector<double> workspace_limits,
    std::string base_link,
    std::string end_link)://
  nh_(n), loop_rate_(frequency),
  M_(M.data()), D_(D.data()),K_(K.data()),desired_pose_(desired_pose.data()),
  arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc), arm_max_ang_vel_(arm_max_ang_vel), arm_max_ang_acc_(arm_max_ang_acc), 
  workspace_limits_(workspace_limits),
  base_link_(base_link), end_link_(end_link){

  //* Subscribers
  sub_arm_state_           = nh_.subscribe(topic_arm_state, 5, 
      &Admittance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_state_        = nh_.subscribe(topic_wrench_state, 5,
      &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  //* Publishers
  pub_arm_cmd_             = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);
  vac_pub_                 = nh_.advertise<geometry_msgs::Point>("/var_damping", 5);
  pub_wrench_              = nh_.advertise<geometry_msgs::WrenchStamped>("/wrench_input", 5);

  // initializing the class variables
  arm_position_.setZero();
  arm_twist_.setZero();
  wrench_external_.setZero();
  desired_pose_position_ << desired_pose_.topRows(3);
  desired_pose_orientation_.coeffs() << desired_pose_.bottomRows(4)/desired_pose_.bottomRows(4).norm();



  while (nh_.ok() && !arm_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }

  // Init integrator
  arm_desired_twist_adm_.setZero();
  last_published_twist_.setZero();

  ft_arm_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;
  force_x_pre = 0;
  force_y_pre = 0;
  force_z_pre = 0;
  torque_x_pre = 0;
  torque_y_pre = 0;
  torque_z_pre = 0;

  // Load rotation admittance parameters (yaw: base z-axis, pitch: EE x-axis)
  if (!nh_.getParam("m_yaw", m_yaw_)) { m_yaw_ = 1.0; ROS_WARN("m_yaw not set, defaulting to 1.0"); }
  if (!nh_.getParam("d_yaw", d_yaw_)) { d_yaw_ = 1.0; ROS_WARN("d_yaw not set, defaulting to 1.0"); }
  if (!nh_.getParam("k_yaw", k_yaw_)) { k_yaw_ = 0.0; ROS_WARN("k_yaw not set, defaulting to 0.0"); }
  if (!nh_.getParam("m_pitch", m_pitch_)) { m_pitch_ = 1.0; ROS_WARN("m_pitch not set, defaulting to 1.0"); }
  if (!nh_.getParam("d_pitch", d_pitch_)) { d_pitch_ = 10.0; ROS_WARN("d_pitch not set, defaulting to 10.0"); }
  if (!nh_.getParam("workspcae_floor_limits", workspace_floor_limits_)) { ROS_WARN("workspcae_floor_limits not set, floor limit disabled"); }
  if (!nh_.getParam("k_pitch", k_pitch_)) { k_pitch_ = 0.0; ROS_WARN("k_pitch not set, defaulting to 0.0"); }
  ee_x_in_base_ = Eigen::Vector3d::UnitX();

  wait_for_transformations();

  setup_diagnostics();

  // load behaviors
  load_behaviors_from_param();

  // key interface starts
  key_stop_ = false;
  key_thread_ = std::thread(&Admittance::keyboardLoop, this);
}

//!-                   INITIALIZATION                    -!//

void Admittance::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  // Makes sure all TFs exists before enabling all transformations in the callbacks
  // while (!get_rotation_matrix(rot_matrix, listener, "world", base_link_)) {sleep(1);}
  base_world_ready_ = true;
  // while (!get_rotation_matrix(rot_matrix, listener, base_link_, "world")) {sleep(1);}
  world_arm_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) {sleep(1);}
  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}

//!-                    CONTROL LOOP                     -!//

void Admittance::run() {

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {

    const ros::Time cycle_start = ros::Time::now();
    if (!last_cycle_time_.isZero()) {
      measured_dt_ = (cycle_start - last_cycle_time_).toSec();
    }
    last_cycle_time_ = cycle_start;

    compute_admittance();

    send_commands_to_robot();

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

//!-                Admittance Dynamics                  -!//

void Admittance::compute_admittance() {

  // While the robot is stopped it does not follow velocity commands, but the
  // admittance keeps integrating the operator wrench. Left alone the command
  // winds up to several hundred mm/s, and the arm lurches the moment the
  // protective stop is released. Hold every integrator state at zero instead.
  if (safety_stop_active()) {
    if (!safety_stop_held_) {
      ROS_WARN("Safety stop: holding the admittance integrator at zero.");
      safety_stop_held_ = true;
    }
    arm_desired_twist_adm_.setZero();
    last_published_twist_.setZero();
    v_yaw_ = 0.0;
    v_pitch_ = 0.0;
    contact_scale_filtered_ = 0.0;
  } else if (safety_stop_held_) {
    ROS_INFO("Safety mode back to NORMAL: releasing the integrator from zero.");
    safety_stop_held_ = false;
  }

  error.topRows(3) = arm_position_ - desired_pose_position_;
  if(desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0)
  {
    arm_orientation_.coeffs() << -arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err (arm_orientation_ * desired_pose_orientation_.inverse());
  if(quat_rot_err.coeffs().norm() > 1e-3)
  {
    quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();

  // Decompose rotation error into yaw (base z) and pitch (EE x) components
  ee_x_in_base_ = arm_orientation_.toRotationMatrix().col(0);
  double e_yaw = error.tail(3).dot(Eigen::Vector3d::UnitZ());
  double e_pitch = error.tail(3).dot(ee_x_in_base_);
  diag_.e_yaw = e_yaw;
  diag_.e_pitch = e_pitch;

  // Behavor renewing and effects collection
  double dt = loop_rate_.expectedCycleTime().toSec(); // time interval
  double tnow = ros::Time::now().toSec();             // current time

  Matrix6d rotation_ft_base;
  get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);

  Vector6d ext_from_behaviors = Vector6d::Zero();
  bool any_behavior_active = false;
  if (wrench_external_.norm() > -10.0) {
    active_behavior_name_.clear();
    for (auto& b : behaviors_) {
      b->update(tnow, dt);
      ext_from_behaviors += rotation_ft_base * b->externalWrench();
      if (b->isActive()) {
        any_behavior_active = true;
        if (active_behavior_name_.empty()) active_behavior_name_ = b->name();
        else active_behavior_name_ += "+" + b->name();
      }
    }
  }
  wrench_behavior_base_ = ext_from_behaviors;
  // Reset velocity when all behaviors just finished
  if (was_any_behavior_active_ && !any_behavior_active) {
    arm_desired_twist_adm_.setZero();
    v_yaw_ = 0.0;
    v_pitch_ = 0.0;
  }
  was_any_behavior_active_ = any_behavior_active;
  // Translation error w.r.t. desired equilibrium
  Vector6d coupling_wrench_arm;

  // Commit the following if CAC (damping unchange)
  // X direction
  if (last_acceleration_x_ > 0)
  {
    var_D_x = fabs(92.4 - 193 * fabs(last_acceleration_x_));
  }
  else{
    var_D_x = fabs(92.4 - 193 * fabs(last_acceleration_x_));
  }
  //
  if (var_D_x > 50){var_D_x = 50;}
  if (var_D_x < 10){var_D_x = 10;}
  D_(0,0) = fabs(var_D_x);

  // Y direction
   if (last_acceleration_y_ > 0)
  {
    var_D_y = fabs(25.2 - 23.6 * fabs(last_acceleration_y_));
  }
  else{
    var_D_y = fabs(25.2 + 47 * fabs(last_acceleration_y_));
  }
  //
  if (var_D_y > 50){var_D_y = 50;}
  if (var_D_y < 10){var_D_y = 10;}
  D_(1,1) = fabs(var_D_y);

  // Z direction
  if (last_acceleration_z_ > 0)
  {
    var_D_z = fabs(310 - 787 * fabs(last_acceleration_z_));
  }
  else{
    var_D_z = fabs(310 + 75 * fabs(last_acceleration_z_));
  }
  //
  if (var_D_z > 400){var_D_z = 400;}
  if (var_D_z < 10){var_D_z = 10;}
  D_(2,2) = fabs(var_D_z);


  geometry_msgs::Point vac_msg;
  vac_msg.x = var_D_x;
  vac_msg.y = var_D_y;
  vac_msg.z = var_D_z;     
  vac_pub_.publish(vac_msg);

  // Vertical Force Compensation
  const double center_x = 0.0263;
  const double center_y = -0.974;
  const double center_z = 0.160;
  const double gain_z = 400; // [N/m^2] adjust this gain to scale the compensation effect
  auto min_z = [](double a, double b) { return (a < b ? a : b); };
  auto max_z = [](double a, double b) { return (a > b ? a : b); };
  double vertical_force_compensation = gain_z * ((arm_position_(0) - center_x)*(arm_position_(0) - center_x)
                                               + (arm_position_(1) - center_y)*(arm_position_(1) - center_y)
                                               + max_z(min_z(0.0,arm_position_(2) - center_z),workspace_limits_[4])*max_z(min_z(0.0,arm_position_(2) - center_z),workspace_limits_[4]));
  diag_.vertical_compensation = vertical_force_compensation;

  wrench_external_(2) -= vertical_force_compensation;

  // --- Translation 3D admittance ---
  coupling_wrench_arm.head(3) = D_.topLeftCorner(3,3) * arm_desired_twist_adm_.head(3)
                              + K_.topLeftCorner(3,3) * error.head(3);
  arm_desired_accelaration.head(3) = M_.topLeftCorner(3,3).inverse()
      * (-coupling_wrench_arm.head(3) + wrench_external_.head(3) - ext_from_behaviors.head(3));

  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();
  acc_norm_ = std::max(acc_norm_, a_acc_norm);

  if (a_acc_norm > arm_max_acc_) {
    acc_clamped_ = true;
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // Integrate translation
  ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_adm_.head(3) += arm_desired_accelaration.head(3) * duration.toSec();
  last_acceleration_x_ = arm_desired_twist_adm_(0);
  last_acceleration_y_ = arm_desired_twist_adm_(1);
  last_acceleration_z_ = arm_desired_twist_adm_(2);

  // --- Yaw 1D admittance (rotation around base z-axis) ---
  double tau_ext_yaw = wrench_external_.tail(3).dot(Eigen::Vector3d::UnitZ())
                     - ext_from_behaviors.tail(3).dot(Eigen::Vector3d::UnitZ());
  double a_yaw = (1.0 / m_yaw_) * (-d_yaw_ * v_yaw_ - k_yaw_ * e_yaw + tau_ext_yaw);
  v_yaw_ += a_yaw * duration.toSec();

  // --- Pitch 1D admittance (rotation around EE x-axis) ---
  double tau_ext_pitch = wrench_external_.tail(3).dot(ee_x_in_base_)
                       - ext_from_behaviors.tail(3).dot(ee_x_in_base_);
  double a_pitch = (1.0 / m_pitch_) * (-d_pitch_ * v_pitch_ - k_pitch_ * e_pitch + tau_ext_pitch);
  v_pitch_ += a_pitch * duration.toSec();

  // Compose angular velocity in base frame
  arm_desired_twist_adm_.tail(3) = v_yaw_ * Eigen::Vector3d::UnitZ()
                                 + v_pitch_ * ee_x_in_base_;

  // Contact gate: low-pass filtered scale to avoid chattering
  const double force_low  = 0.0;   // [N]
  const double force_high = 3.0;   // [N]
  const double tau = 0.15;          // [s] filter time constant
  double force_norm = wrench_external_.head(3).norm();
  double target_scale;
  if (force_norm >= force_high) {
    target_scale = 1.0;
  } else if (force_norm <= force_low) {
    target_scale = 0.0;
  } else {
    target_scale = (force_norm - force_low) / (force_high - force_low);
  }
  double alpha = dt / (tau + dt);
  contact_scale_filtered_ += alpha * (target_scale - contact_scale_filtered_);
  arm_desired_twist_adm_ *= contact_scale_filtered_;
  // arm_desired_twist_adm_.head(3) *= (contact_scale_filtered_ + 1.0) / 2.0;

  // Workspace limits enforcement
  const double x = arm_position_(0);
  const double y = arm_position_(1);
  const double z = arm_position_(2);

  const double x_min = workspace_limits_[0], x_max = workspace_limits_[1];
  const double y_min = workspace_limits_[2], y_max = workspace_limits_[3];
  double z_min = workspace_limits_[4];
  const double z_max = workspace_limits_[5];
  const double margin = workspace_limits_[8];

  // If x <= x_edge and y <= y_edge, extend z lower limit to z_floor
  if (workspace_floor_limits_.size() >= 3) {
    const double x_edge = workspace_floor_limits_[0];
    const double y_edge = workspace_floor_limits_[1];
    const double z_floor = workspace_floor_limits_[2];
    if (x <= x_edge && y <= y_edge) {
      z_min = z_floor;
    }
  }

  auto enforce_axis = [&](int axis_idx, double pos, double lo, double hi) {
    double vel = arm_desired_twist_adm_(axis_idx);
    if (pos <= lo + margin && vel < 0) {
      workspace_clamped_ = true;
      if (pos <= lo) {
        arm_desired_twist_adm_(axis_idx) = 0;                       // Hard Floor
      } else {
        double scale = std::max(0.0, (pos - lo) / margin);          // Soft Floor (0..1)
        arm_desired_twist_adm_(axis_idx) *= scale;
      }
    }
    if (pos >= hi - margin && vel > 0) {
      workspace_clamped_ = true;
      if (pos >= hi) {
        arm_desired_twist_adm_(axis_idx) = 0;                       // Hard Floor
      } else {
        double scale = std::max(0.0, (hi - pos) / margin);          // Soft Floor (0..1)
        arm_desired_twist_adm_(axis_idx) *= scale;
      }
    }
  };

  enforce_axis(0, x, x_min, x_max);
  enforce_axis(1, y, y_min, y_max);
  enforce_axis(2, z, z_min, z_max);

  // Yaw workspace limits (relative to desired orientation)
  const double yaw_min = workspace_limits_[6], yaw_max = workspace_limits_[7];
  const double yaw_margin = 0.05; // [rad]
  if (e_yaw <= yaw_min + yaw_margin && v_yaw_ < 0) {
    if (e_yaw <= yaw_min) {
      v_yaw_ = 0;
    } else {
      v_yaw_ *= std::max(0.0, (e_yaw - yaw_min) / yaw_margin);
    }
  }
  if (e_yaw >= yaw_max - yaw_margin && v_yaw_ > 0) {
    if (e_yaw >= yaw_max) {
      v_yaw_ = 0;
    } else {
      v_yaw_ *= std::max(0.0, (yaw_max - e_yaw) / yaw_margin);
    }
  }
  // Recompose angular velocity after yaw limiting
  arm_desired_twist_adm_.tail(3) = v_yaw_ * Eigen::Vector3d::UnitZ()
                                 + v_pitch_ * ee_x_in_base_;

  // Pitch workspace limits (relative to desired orientation)
  if (workspace_limits_.size() > 10) {
    const double pitch_min = workspace_limits_[9], pitch_max = workspace_limits_[10];
    const double pitch_margin = 0.05; // [rad]
    if (e_pitch <= pitch_min + pitch_margin && v_pitch_ < 0) {
      if (e_pitch <= pitch_min) {
        v_pitch_ = 0;
      } else {
        v_pitch_ *= std::max(0.0, (e_pitch - pitch_min) / pitch_margin);
      }
    }
    if (e_pitch >= pitch_max - pitch_margin && v_pitch_ > 0) {
      if (e_pitch >= pitch_max) {
        v_pitch_ = 0;
      } else {
        v_pitch_ *= std::max(0.0, (pitch_max - e_pitch) / pitch_margin);
      }
    }
    // Recompose angular velocity after pitch limiting
    arm_desired_twist_adm_.tail(3) = v_yaw_ * Eigen::Vector3d::UnitZ()
                                   + v_pitch_ * ee_x_in_base_;
  }
}

//!-                     CALLBACKS                       -!//

void Admittance::state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  arm_position_ <<  msg->pose.position.x,
                    msg->pose.position.y, 
                    msg->pose.position.z;

  arm_orientation_.coeffs() <<  msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z,
                                msg->pose.orientation.w;

  arm_twist_ << msg->twist.linear.x, 
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
}

void Admittance::state_wrench_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {
    wrench_ft_frame <<  msg->wrench.force.x,
                        msg->wrench.force.y,
                        msg->wrench.force.z,
                        msg->wrench.torque.x,
                        msg->wrench.torque.y,
                        msg->wrench.torque.z;

    float force_thres_lower_limit_ = 6;
    float force_thres_upper_limit_ = 200;
    float T_X_ = 0;
    float T_Y_ = 0;
    float T_Z_ = 0; // manully compensation

    if(fabs(wrench_ft_frame(0)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(0)) > force_thres_upper_limit_){wrench_ft_frame(0) = 0;}
    else{
      // if(wrench_ft_frame(0) > 0){wrench_ft_frame(0) -= T_X_;}
      // else{wrench_ft_frame(0) += T_X_;}
      wrench_ft_frame(0) -= T_X_;
      // wrench_ft_frame(0) = (1 - 0.2)*force_x_pre + 0.2*wrench_ft_frame(0);
      // force_x_pre = wrench_ft_frame(0);
    }
    if(fabs(wrench_ft_frame(1)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(1)) > force_thres_upper_limit_){wrench_ft_frame(1) = 0;}
    else{
      // if(wrench_ft_frame(1) > 0){wrench_ft_frame(1) -= T_Y_;}
      // else{wrench_ft_frame(1) += T_Y_;}
      wrench_ft_frame(1) -= T_Y_;
      // wrench_ft_frame(1) = (1 - 0.2)*force_y_pre + 0.2*wrench_ft_frame(1);
      // force_y_pre = wrench_ft_frame(1);
    }
    if(fabs(wrench_ft_frame(2)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(2)) > force_thres_upper_limit_){wrench_ft_frame(2) = 0;}
    else{
      // if(wrench_ft_frame(2) > 0){wrench_ft_frame(2) -= T_Z_;}
      // else{wrench_ft_frame(2) += T_Z_;}
      wrench_ft_frame(2) += T_Z_;
      // wrench_ft_frame(2) = (1 - 0.2)*force_z_pre + 0.2*wrench_ft_frame(2);
      // force_z_pre = wrench_ft_frame(2);
    }
    // Torque
    if(fabs(wrench_ft_frame(3)) < 1 || fabs(wrench_ft_frame(3)) > force_thres_upper_limit_){wrench_ft_frame(3) = 0;}
    else{
      // wrench_ft_frame(3) = (1 - 0.1)*torque_x_pre + 0.1*wrench_ft_frame(3);
      // torque_x_pre = wrench_ft_frame(3);
    }
    if(fabs(wrench_ft_frame(4)) < 1 || fabs(wrench_ft_frame(4)) > force_thres_upper_limit_){wrench_ft_frame(4) = 0;}
    else{
      // wrench_ft_frame(4) = (1 - 0.2)*torque_y_pre + 0.2*wrench_ft_frame(4);
      // torque_y_pre = wrench_ft_frame(4);
    }
    if(fabs(wrench_ft_frame(5)) < 1 || fabs(wrench_ft_frame(5)) > force_thres_upper_limit_){wrench_ft_frame(5) = 0;}
    else{
      // wrench_ft_frame(5) = (1 - 0.2)*torque_z_pre + 0.2*wrench_ft_frame(5);
      // torque_z_pre = wrench_ft_frame(5);
    }

    get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
    wrench_external_ <<  rotation_ft_base * wrench_ft_frame;
    wrench_user_base_ = wrench_external_;

    geometry_msgs::WrenchStamped wrench_input;
    wrench_input.wrench.force.x  = wrench_ft_frame(0);
    wrench_input.wrench.force.y  = wrench_ft_frame(1);
    wrench_input.wrench.force.z  = wrench_ft_frame(2);
    wrench_input.wrench.torque.x = wrench_ft_frame(3);
    wrench_input.wrench.torque.y = wrench_ft_frame(4);
    wrench_input.wrench.torque.z = wrench_ft_frame(5);
    
    pub_wrench_.publish(wrench_input);

  }
}

//!-               COMMANDING THE ROBOT                  -!//

void Admittance::send_commands_to_robot() {
  // Never hand a non-finite twist to the IK solver: it would turn into NaN
  // joint velocities and there is no telling what the arm does with those.
  // Recover by dropping the whole integrator state back to standstill.
  if (!arm_desired_twist_adm_.allFinite() || !std::isfinite(v_yaw_) ||
      !std::isfinite(v_pitch_)) {
    ROS_ERROR_THROTTLE(1.0, "Admittance produced a non-finite command; "
                            "resetting the integrator to zero.");
    arm_desired_twist_adm_.setZero();
    last_published_twist_.setZero();
    v_yaw_ = 0.0;
    v_pitch_ = 0.0;
    last_acceleration_x_ = 0.0;
    last_acceleration_y_ = 0.0;
    last_acceleration_z_ = 0.0;
    track_err_lin_filtered_ = 0.0;
    track_err_ang_filtered_ = 0.0;
  }

  if (safety_stop_active()) {
    // Publish zero rather than nothing: the cartesian controller latches the
    // last twist it received, so silence would leave the old command in place.
    arm_desired_twist_adm_.setZero();
    // The compliance logic is skipped while stopped, so clear its state instead
    // of leaving the pre-stop values frozen in the diagnostics.
    track_err_lin_filtered_ = 0.0;
    track_err_ang_filtered_ = 0.0;
    track_gain_lin_ = 0.0;
    track_gain_ang_ = 0.0;
  } else {
    apply_tracking_compliance();
  }

  double lin_norm = (arm_desired_twist_adm_.segment(0, 3)).norm();
  // (Normalized Scaling) Velosity limitation 
  if (lin_norm > arm_max_vel_) {
    vel_clamped_ = true;
    ROS_WARN_STREAM_THROTTLE(1, "Admittance fast linear velocity! norm: " << lin_norm);
    arm_desired_twist_adm_.segment(0, 3) *= (arm_max_vel_ / lin_norm);
  }
  // (Normalized Scaling) Angular limitation
  double ang_norm = (arm_desired_twist_adm_.segment(3, 3)).norm();
  if (ang_norm > arm_max_ang_vel_) {
    ang_vel_clamped_ = true;
    ROS_WARN_STREAM_THROTTLE(1, "Admittance fast angular velocity! norm: " << ang_norm);
    arm_desired_twist_adm_.segment(3, 3) *= (arm_max_ang_vel_ / ang_norm);
  }

  // Slew limitation (make dv, domega less than dt*max_acc)
  double dt = loop_rate_.expectedCycleTime().toSec();
  Vector3d v_prev = last_published_twist_.segment(0,3);
  Vector3d v_new  = arm_desired_twist_adm_.segment(0,3);
  Vector3d dv = v_new - v_prev;
  double max_dv = std::max(1e-6, arm_max_acc_ * dt);
  if (dv.norm() > max_dv) {
    slew_clamped_ = true;
    dv *= (max_dv / dv.norm());
    v_new = v_prev + dv;
  }
  arm_desired_twist_adm_.segment(0,3) = v_new;

  Vector3d w_prev = last_published_twist_.segment(3,3);
  Vector3d w_new  = arm_desired_twist_adm_.segment(3,3);
  Vector3d dw = w_new - w_prev;
  double max_dw = std::max(1e-6, arm_max_ang_acc_ * dt);
  if (dw.norm() > max_dw) {
    slew_clamped_ = true;
    dw *= (max_dw / dw.norm());
    w_new = w_prev + dw;
  }
  arm_desired_twist_adm_.segment(3,3) = w_new;

  // Update rotation integrator states from the limited angular velocity
  v_yaw_ = arm_desired_twist_adm_.tail(3).dot(Eigen::Vector3d::UnitZ());
  v_pitch_ = arm_desired_twist_adm_.tail(3).dot(ee_x_in_base_);

  geometry_msgs::Twist arm_twist_cmd;
  arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_adm_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_adm_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_adm_(5);

  pub_arm_cmd_.publish(arm_twist_cmd);
  last_published_twist_ = arm_desired_twist_adm_;

  if (safety_stop_active()) {
    // Start the slew limiter from standstill once the stop is released.
    last_published_twist_.setZero();
    v_yaw_ = 0.0;
    v_pitch_ = 0.0;
  }

  publish_diagnostics();
}

//!-                    UTILIZATION                      -!//

/**
 * To check whether TF exists
*/

bool Admittance::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                            ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }
  return true;
}

void Admittance::load_behaviors_from_param() {
  XmlRpc::XmlRpcValue arr;
  if (!nh_.getParam("delay_sec", delay_sec)) { ROS_ERROR("Couldn't retrieve the delaytime for behaviors."); return; }
  if (!nh_.getParam("behaviors", arr) || arr.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    auto kb = std::make_shared<KneeBucklingBehavior>("knee_default");
    behaviors_.push_back(kb);
    ROS_INFO("No behaviors param found. Added default KneeBucklingBehavior.");
    return;
  }
  for (int i = 0; i < arr.size(); ++i) {
    if (arr[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
    std::string type = static_cast<std::string>(arr[i]["type"]);
    std::string name = static_cast<std::string>(arr[i]["name"]);
    if (type == "KneeBuckling") {
      auto b = std::make_shared<KneeBucklingBehavior>(name);
      if (arr[i].hasMember("force_y"))  b->force_y  = static_cast<double>(arr[i]["force_y"]);
      if (arr[i].hasMember("force_z"))  b->force_z  = static_cast<double>(arr[i]["force_z"]);
      if (arr[i].hasMember("duration")) b->duration = static_cast<double>(arr[i]["duration"]);
      behaviors_.push_back(b);
    } else if (type == "SeatSliding") {
      auto b = std::make_shared<SeatSlidingBehavior>(name);
      if (arr[i].hasMember("force_y"))  b->force_y  = static_cast<double>(arr[i]["force_y"]);
      if (arr[i].hasMember("force_z"))  b->force_z  = static_cast<double>(arr[i]["force_z"]);
      if (arr[i].hasMember("torque_x")) b->torque_x = static_cast<double>(arr[i]["torque_x"]);
      if (arr[i].hasMember("duration")) b->duration = static_cast<double>(arr[i]["duration"]);
      behaviors_.push_back(b);
    } else if (type == "OutOfLineStand") {
      auto b = std::make_shared<OutOfLineStandBehavior>(name);
      if (arr[i].hasMember("force_x"))  b->force_x  = static_cast<double>(arr[i]["force_x"]);
      if (arr[i].hasMember("duration")) b->duration = static_cast<double>(arr[i]["duration"]);
      behaviors_.push_back(b);
    } else if (type == "OutOfLineStSBack") {
      auto b = std::make_shared<OutOfLineStSBackBehavior>(name);
      if (arr[i].hasMember("force_y"))  b->force_y  = static_cast<double>(arr[i]["force_y"]);
      if (arr[i].hasMember("force_z"))  b->force_z  = static_cast<double>(arr[i]["force_z"]);
      if (arr[i].hasMember("torque_x")) b->torque_x = static_cast<double>(arr[i]["torque_x"]);
      if (arr[i].hasMember("duration")) b->duration = static_cast<double>(arr[i]["duration"]);
      behaviors_.push_back(b);
    } else if (type == "OutOfLineStSSide") {
      auto b = std::make_shared<OutOfLineStSSideBehavior>(name);
      if (arr[i].hasMember("force_x"))  b->force_x  = static_cast<double>(arr[i]["force_x"]);
      if (arr[i].hasMember("force_y"))  b->force_y  = static_cast<double>(arr[i]["force_y"]);
      if (arr[i].hasMember("force_z"))  b->force_z  = static_cast<double>(arr[i]["force_z"]);
      if (arr[i].hasMember("torque_y")) b->torque_y = static_cast<double>(arr[i]["torque_y"]);
      if (arr[i].hasMember("duration")) b->duration = static_cast<double>(arr[i]["duration"]);
      behaviors_.push_back(b);
    } else if (type == "StS") {
      auto b = std::make_shared<StSBehavior>(name);
      if (arr[i].hasMember("force_y"))  b->force_y  = static_cast<double>(arr[i]["force_y"]);
      if (arr[i].hasMember("force_z"))  b->force_z  = static_cast<double>(arr[i]["force_z"]);
      if (arr[i].hasMember("duration")) b->duration = static_cast<double>(arr[i]["duration"]);
      behaviors_.push_back(b);
    } else {
      ROS_WARN("Unknown behavior type: %s", type.c_str());
    }
  }
}

void Admittance::triggerBehavior(const std::string& name) {
  for (auto& b : behaviors_) {
    if (b->name() == name) {
      ros::Timer t = nh_.createTimer(ros::Duration(delay_sec),[this, b](const ros::TimerEvent&){
        b->trigger();
        ROS_INFO("Triggered behavior: %s", b->name().c_str());
      }, true);
      behavior_delayed_timers_.push_back(t);
      return;
    }
  }
  ROS_WARN("Behavior not found: %s", name.c_str());
}

void Admittance::resetAllBehaviors() {
  for (auto& b : behaviors_) b->reset();
  ROS_INFO("All behaviors reset.");
}

void Admittance::keyboardLoop() {
  // change to raw mode
  struct termios raw;
  if (tcgetattr(STDIN_FILENO, &orig_term_) == 0) {
    raw = orig_term_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ROS_INFO("Keyboard: press 'k' (knee), 's' (slide), 'o' (OOL_stand), 'b' (OOL_StS_back), 'l' (OOL_StS_side), 't' (StS), 'r' (reset), 'd' (dump diagnostic log).");

  while (ros::ok() && !key_stop_) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    struct timeval tv {0, 100000}; // 100ms
    int rv = select(STDIN_FILENO+1, &set, nullptr, nullptr, &tv);
    if (rv > 0 && FD_ISSET(STDIN_FILENO, &set)) {
      char c;
      ssize_t n = read(STDIN_FILENO, &c, 1);
      if (n == 1) {
        if (c == 'k') {
          triggerBehavior("knee1");
        } else if (c == 's') {
          triggerBehavior("slide1");
        } else if (c == 'o') {
          triggerBehavior("ool_stand1");
        } else if (c == 'b') {
          triggerBehavior("ool_sts_back1");
        } else if (c == 'l') {
          triggerBehavior("ool_sts_side1");
        } else if (c == 't') {
          triggerBehavior("sts1");
        } else if (c == 'r') {
          resetAllBehaviors();
        } else if (c == 'd') {
          if (diag_recorder_) diag_recorder_->requestManualDump();
        }
      }
    }
  }
  // restoration
  tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
}

//!-                    DIAGNOSTICS                      -!//

void Admittance::apply_tracking_compliance() {
  if (!tracking_enabled_) {
    track_gain_lin_ = 0.0;
    track_gain_ang_ = 0.0;
    return;
  }

  const double dt = loop_rate_.expectedCycleTime().toSec();

  // last_published_twist_ still holds the previous cycle's command, so this is
  // the error between what we asked for and what the arm actually did.
  const double e_lin = (last_published_twist_.head(3) - arm_twist_.head(3)).norm();
  const double e_ang = (last_published_twist_.tail(3) - arm_twist_.tail(3)).norm();

  const double alpha = dt / (track_filter_tau_ + dt);
  track_err_lin_filtered_ += alpha * (e_lin - track_err_lin_filtered_);
  track_err_ang_filtered_ += alpha * (e_ang - track_err_ang_filtered_);

  auto ramp = [](double e, double lo, double hi) {
    if (hi <= lo) return 0.0;
    return std::min(1.0, std::max(0.0, (e - lo) / (hi - lo)));
  };
  track_gain_lin_ = ramp(track_err_lin_filtered_, track_err_lin_low_, track_err_lin_high_);
  track_gain_ang_ = ramp(track_err_ang_filtered_, track_err_ang_low_, track_err_ang_high_);

  if (track_gain_lin_ <= 0.0 && track_gain_ang_ <= 0.0) return;

  // First order pull, so the back off rate is an explicit time constant rather
  // than a per-cycle factor that compounds at the loop rate.
  const double k = dt / std::max(1e-3, track_pull_tau_);

  arm_desired_twist_adm_.head(3) +=
      track_gain_lin_ * k * (arm_twist_.head(3) - arm_desired_twist_adm_.head(3));

  // The rotation admittance only lives in the (base z, EE x) subspace, so pull
  // those two states rather than the raw angular vector.
  const double w_yaw_meas = arm_twist_.tail(3).dot(Eigen::Vector3d::UnitZ());
  const double w_pitch_meas = arm_twist_.tail(3).dot(ee_x_in_base_);
  v_yaw_ += track_gain_ang_ * k * (w_yaw_meas - v_yaw_);
  v_pitch_ += track_gain_ang_ * k * (w_pitch_meas - v_pitch_);
  arm_desired_twist_adm_.tail(3) = v_yaw_ * Eigen::Vector3d::UnitZ()
                                 + v_pitch_ * ee_x_in_base_;

  ROS_WARN_STREAM_THROTTLE(0.5, "Tracking compliance: the arm is behind its command"
      << " (err_lin " << track_err_lin_filtered_ << " m/s, err_ang "
      << track_err_ang_filtered_ << " rad/s), easing off with gain "
      << track_gain_lin_ << "/" << track_gain_ang_);
}

bool Admittance::safety_stop_active() const {
  if (!diag_recorder_) return false;
  const uint8_t mode = diag_recorder_->safetyMode();
  // 0 means the driver has not reported a mode yet, e.g. in simulation.
  return mode != 0 && mode != ur_dashboard_msgs::SafetyMode::NORMAL;
}

void Admittance::setup_diagnostics() {
  // publish_rate <= 0 means one diagnostic sample per control cycle.
  double publish_rate = 0.0;
  double console_rate = 2.0;
  nh_.param("diag/publish_rate", publish_rate, publish_rate);
  nh_.param("diag/console_rate", console_rate, console_rate);
  diag_publish_period_ = (publish_rate > 0.0) ? 1.0 / publish_rate : 0.0;
  console_period_      = (console_rate > 0.0) ? 1.0 / console_rate : 0.0;

  nh_.param("tracking/enabled", tracking_enabled_, tracking_enabled_);
  nh_.param("tracking/filter_tau", track_filter_tau_, track_filter_tau_);
  nh_.param("tracking/pull_tau", track_pull_tau_, track_pull_tau_);
  nh_.param("tracking/err_lin_low", track_err_lin_low_, track_err_lin_low_);
  nh_.param("tracking/err_lin_high", track_err_lin_high_, track_err_lin_high_);
  nh_.param("tracking/err_ang_low", track_err_ang_low_, track_err_ang_low_);
  nh_.param("tracking/err_ang_high", track_err_ang_high_, track_err_ang_high_);
  ROS_INFO(tracking_enabled_
      ? "Tracking compliance enabled: easing off between %.3f and %.3f m/s of velocity error."
      : "Tracking compliance DISABLED (%.3f, %.3f unused).",
      track_err_lin_low_, track_err_lin_high_);

  pub_diag_ = nh_.advertise<admittance_msgs::AdmittanceDiag>("/admittance_diag", 10);
  sub_joint_state_ = nh_.subscribe("/joint_states", 1,
      &Admittance::state_joint_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  diag_recorder_.reset(new DiagRecorder(nh_));

  // Build the kinematic chain so the Jacobian, and with it the distance to a
  // singularity, can be evaluated alongside the admittance state.
  std::string param_name, urdf_xml;
  if (!nh_.searchParam("robot_description", param_name) ||
      !nh_.getParam(param_name, urdf_xml) || urdf_xml.empty()) {
    ROS_WARN("robot_description not found: manipulability will not be reported.");
    return;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) {
    ROS_WARN("Could not parse robot_description: manipulability will not be reported.");
    return;
  }
  if (!tree.getChain(base_link_, end_link_, kdl_chain_)) {
    ROS_WARN_STREAM("No chain from " << base_link_ << " to " << end_link_
                    << ": manipulability will not be reported.");
    return;
  }

  for (const auto& segment : kdl_chain_.segments) {
    if (segment.getJoint().getType() != KDL::Joint::None) {
      chain_joint_names_.push_back(segment.getJoint().getName());
    }
  }
  joint_position_.assign(chain_joint_names_.size(), 0.0);
  joint_velocity_.assign(chain_joint_names_.size(), 0.0);
  joint_effort_.assign(chain_joint_names_.size(), 0.0);

  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  ROS_INFO_STREAM("Diagnostics: chain " << base_link_ << " -> " << end_link_
                  << " with " << chain_joint_names_.size() << " joints.");
}

void Admittance::state_joint_callback(const sensor_msgs::JointStateConstPtr msg) {
  if (chain_joint_names_.empty()) return;

  // /joint_states is not guaranteed to be ordered like the chain, so match by name.
  for (std::size_t i = 0; i < chain_joint_names_.size(); ++i) {
    for (std::size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] != chain_joint_names_[i]) continue;
      if (j < msg->position.size()) joint_position_[i] = msg->position[j];
      if (j < msg->velocity.size()) joint_velocity_[i] = msg->velocity[j];
      if (j < msg->effort.size())   joint_effort_[i]   = msg->effort[j];
      break;
    }
  }
  joint_state_ready_ = true;
}

void Admittance::update_manipulability() {
  if (!jac_solver_ || !joint_state_ready_) return;

  const unsigned int n = kdl_chain_.getNrOfJoints();
  if (joint_position_.size() != n) return;

  KDL::JntArray q(n);
  for (unsigned int i = 0; i < n; ++i) q(i) = joint_position_[i];

  KDL::Jacobian jac(n);
  if (jac_solver_->JntToJac(q, jac) < 0) return;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac.data);
  const Eigen::VectorXd sv = svd.singularValues();
  if (sv.size() == 0) return;
  sigma_min_ = sv(sv.size() - 1);
  manipulability_ = sv.prod();
}

void Admittance::publish_diagnostics() {
  if (!diag_recorder_) return;

  const ros::Time now = ros::Time::now();
  if (diag_publish_period_ > 0.0 && !last_diag_time_.isZero() &&
      (now - last_diag_time_).toSec() < diag_publish_period_) {
    return;
  }
  last_diag_time_ = now;

  update_manipulability();

  diag_.header.stamp = now;
  diag_.header.frame_id = base_link_;

  diag_.wrench_user.force.x  = wrench_user_base_(0);
  diag_.wrench_user.force.y  = wrench_user_base_(1);
  diag_.wrench_user.force.z  = wrench_user_base_(2);
  diag_.wrench_user.torque.x = wrench_user_base_(3);
  diag_.wrench_user.torque.y = wrench_user_base_(4);
  diag_.wrench_user.torque.z = wrench_user_base_(5);

  diag_.wrench_effective.force.x  = wrench_external_(0);
  diag_.wrench_effective.force.y  = wrench_external_(1);
  diag_.wrench_effective.force.z  = wrench_external_(2);
  diag_.wrench_effective.torque.x = wrench_external_(3);
  diag_.wrench_effective.torque.y = wrench_external_(4);
  diag_.wrench_effective.torque.z = wrench_external_(5);

  // Sign flipped so this reads as the wrench acting on the effector.
  diag_.wrench_behavior.force.x  = -wrench_behavior_base_(0);
  diag_.wrench_behavior.force.y  = -wrench_behavior_base_(1);
  diag_.wrench_behavior.force.z  = -wrench_behavior_base_(2);
  diag_.wrench_behavior.torque.x = -wrench_behavior_base_(3);
  diag_.wrench_behavior.torque.y = -wrench_behavior_base_(4);
  diag_.wrench_behavior.torque.z = -wrench_behavior_base_(5);

  diag_.twist_cmd.linear.x  = last_published_twist_(0);
  diag_.twist_cmd.linear.y  = last_published_twist_(1);
  diag_.twist_cmd.linear.z  = last_published_twist_(2);
  diag_.twist_cmd.angular.x = last_published_twist_(3);
  diag_.twist_cmd.angular.y = last_published_twist_(4);
  diag_.twist_cmd.angular.z = last_published_twist_(5);

  diag_.twist_meas.linear.x  = arm_twist_(0);
  diag_.twist_meas.linear.y  = arm_twist_(1);
  diag_.twist_meas.linear.z  = arm_twist_(2);
  diag_.twist_meas.angular.x = arm_twist_(3);
  diag_.twist_meas.angular.y = arm_twist_(4);
  diag_.twist_meas.angular.z = arm_twist_(5);

  // The quantity the UR protective stop ultimately reacts to.
  const double err_lin = (last_published_twist_.head(3) - arm_twist_.head(3)).norm();
  const double err_ang = (last_published_twist_.tail(3) - arm_twist_.tail(3)).norm();
  diag_.tracking_error_lin = err_lin;
  diag_.tracking_error_ang = err_ang;

  diag_.position.x = arm_position_(0);
  diag_.position.y = arm_position_(1);
  diag_.position.z = arm_position_(2);

  diag_.damping_diag.x = D_(0,0);
  diag_.damping_diag.y = D_(1,1);
  diag_.damping_diag.z = D_(2,2);

  diag_.tracking_error_lin_filtered = track_err_lin_filtered_;
  diag_.tracking_error_ang_filtered = track_err_ang_filtered_;
  diag_.tracking_gain_lin = track_gain_lin_;
  diag_.tracking_gain_ang = track_gain_ang_;

  diag_.contact_scale = contact_scale_filtered_;
  diag_.acc_norm = acc_norm_;

  diag_.acc_clamped = acc_clamped_;
  diag_.vel_clamped = vel_clamped_;
  diag_.ang_vel_clamped = ang_vel_clamped_;
  diag_.slew_clamped = slew_clamped_;
  diag_.workspace_clamped = workspace_clamped_;

  diag_.joint_position = joint_position_;
  diag_.joint_velocity = joint_velocity_;
  diag_.joint_effort = joint_effort_;
  diag_.sigma_min = sigma_min_;
  diag_.manipulability = manipulability_;

  diag_.loop_dt = measured_dt_;
  diag_.safety_mode = diag_recorder_->safetyMode();
  diag_.active_behavior = active_behavior_name_;

  pub_diag_.publish(diag_);
  diag_recorder_->push(diag_);

  acc_clamped_ = false;
  vel_clamped_ = false;
  ang_vel_clamped_ = false;
  slew_clamped_ = false;
  workspace_clamped_ = false;
  acc_norm_ = 0.0;

  if (console_period_ <= 0.0) return;
  if (!last_console_time_.isZero() && (now - last_console_time_).toSec() < console_period_) {
    return;
  }
  last_console_time_ = now;

  std::string flags;
  if (acc_clamped_)       flags += "[ACC]";
  if (vel_clamped_)       flags += "[VEL]";
  if (ang_vel_clamped_)   flags += "[AVEL]";
  if (slew_clamped_)      flags += "[SLEW]";
  if (workspace_clamped_) flags += "[WS]";
  if (flags.empty())      flags = "-";

  double effort_max = 0.0;
  for (double e : joint_effort_) effort_max = std::max(effort_max, std::fabs(e));

  char line[512];
  std::snprintf(line, sizeof(line),
      "f_u=(%6.1f %6.1f %6.1f)N f_b=(%6.1f %6.1f %6.1f)N | "
      "|v_cmd|=%.3f |v_meas|=%.3f err=%.3f | D=(%3.0f %3.0f %3.0f) gate=%.2f | "
      "sig=%.3f eff=%.1f dt=%.4f | trk=%.2f/%.2f | beh=%s %s",
      diag_.wrench_user.force.x, diag_.wrench_user.force.y, diag_.wrench_user.force.z,
      diag_.wrench_behavior.force.x, diag_.wrench_behavior.force.y, diag_.wrench_behavior.force.z,
      last_published_twist_.head(3).norm(), arm_twist_.head(3).norm(), err_lin,
      D_(0,0), D_(1,1), D_(2,2), contact_scale_filtered_,
      sigma_min_, effort_max, measured_dt_,
      track_gain_lin_, track_gain_ang_,
      active_behavior_name_.empty() ? "-" : active_behavior_name_.c_str(),
      flags.c_str());
  ROS_INFO_STREAM(line);
}
