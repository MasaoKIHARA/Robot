#ifndef ADMITTANCE_H
#define ADMITTANCE_H

#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include "sensor_msgs/JointState.h"

#include <memory>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cmath>
#include <vector>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include "Behavior/Behavior.h"
#include "Admittance/DiagRecorder.h"
#include "admittance_msgs/AdmittanceDiag.h"

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>


using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
#define PI 3.1415926

class Admittance
{
protected:
  // ROS VARIABLES:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  // ADMITTANCE PARAMETERS:
  Matrix6d M_, D_, K_;

  // Subscribers:
  ros::Subscriber sub_arm_state_;
  ros::Subscriber sub_wrench_state_;
  ros::Subscriber sub_filtered_force_;
  // Publishers:
  ros::Publisher pub_arm_cmd_;
  ros::Publisher vac_pub_;
  ros::Publisher pub_wrench_;
  // Variables:
  Vector3d      arm_position_;
  Quaterniond   arm_orientation_;
  Vector6d      arm_twist_;
  Vector6d      wrench_external_;
  Vector6d      arm_desired_twist_adm_;
  Vector6d      arm_desired_accelaration;
  Vector6d      last_published_twist_;

  Vector7d      desired_pose_;
  Vector3d      desired_pose_position_;
  Quaterniond   desired_pose_orientation_;

  Vector6d      error;

  // TF:
  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool ft_arm_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  double arm_max_vel_;
  double arm_max_acc_;
  double arm_max_ang_vel_;
  double arm_max_ang_acc_;
  std::vector<double> workspace_limits_;
  std::vector<double> workspace_floor_limits_;

  double force_x_pre, force_y_pre, force_z_pre;
  double torque_x_pre, torque_y_pre, torque_z_pre;
  double D_z0, A_z0, B_z0;

  double var_D_x;
  double var_D_y;
  double var_D_z;

  double last_acceleration_x_;
  double last_acceleration_y_;
  double last_acceleration_z_;

  // Rotation admittance (1D per axis)
  double m_yaw_, d_yaw_, k_yaw_;       // yaw (base z-axis) parameters
  double m_pitch_, d_pitch_, k_pitch_;  // pitch (EE x-axis) parameters
  double v_yaw_ = 0.0;                 // yaw angular velocity state
  double v_pitch_ = 0.0;               // pitch angular velocity state
  Eigen::Vector3d ee_x_in_base_;       // cached EE x-axis in base frame

  // behavior framework
  std::vector<std::shared_ptr<Behavior>> behaviors_;
  bool was_any_behavior_active_ = false;

  // Contact gate (low-pass filtered)
  double contact_scale_filtered_ = 0.0;

  // --- Diagnostics ---
  ros::Publisher pub_diag_;
  ros::Subscriber sub_joint_state_;
  std::unique_ptr<DiagRecorder> diag_recorder_;
  admittance_msgs::AdmittanceDiag diag_;

  double diag_publish_period_ = 0.01;  // [s] 0 disables the diagnostic topic
  double console_period_ = 0.5;        // [s] 0 disables the console summary
  ros::Time last_diag_time_;
  ros::Time last_console_time_;
  ros::Time last_cycle_time_;
  double measured_dt_ = 0.0;           // actual control loop period [s]

  // Joint state mirrored from the driver, ordered along the kinematic chain
  std::vector<std::string> chain_joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  bool joint_state_ready_ = false;

  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  double sigma_min_ = 0.0;
  double manipulability_ = 0.0;

  // True while the robot reports a safety mode other than NORMAL. The
  // admittance integrator is held at zero for as long as this lasts.
  bool safety_stop_held_ = false;

  // Which limiter fired during the current cycle
  bool acc_clamped_ = false;
  bool vel_clamped_ = false;
  bool ang_vel_clamped_ = false;
  bool slew_clamped_ = false;
  bool workspace_clamped_ = false;
  double acc_norm_ = 0.0;

  // Operator wrench as it arrives from the sensor, before compute_admittance
  // folds the synthetic vertical compensation into wrench_external_
  Vector6d wrench_user_base_ = Vector6d::Zero();
  // Behavior wrench of the current cycle, expressed in the base frame
  Vector6d wrench_behavior_base_ = Vector6d::Zero();
  std::string active_behavior_name_;

  // key interface
  std::thread key_thread_;
  std::atomic_bool key_stop_{false};
  struct termios orig_term_{};

public:
  Admittance(ros::NodeHandle &n, double frequency,
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
                      std::string end_link
                       );
  void run();
  void triggerBehavior(const std::string& name);
  void resetAllBehaviors();
private:
  // Control
  void compute_admittance();
  // Callbacks
  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
  void state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void send_commands_to_robot();
  void wait_for_transformations();
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);

  // internal
  double delay_sec;
  void load_behaviors_from_param();
  void keyboardLoop();

  // True when the robot is not obeying velocity commands (protective stop,
  // safeguard stop, emergency stop, violation, fault). False when the safety
  // mode is NORMAL or has not been reported at all, e.g. in simulation.
  bool safety_stop_active() const;

  // Diagnostics
  void setup_diagnostics();
  void state_joint_callback(const sensor_msgs::JointStateConstPtr msg);
  void update_manipulability();
  void publish_diagnostics();

  std::vector<ros::Timer> behavior_delayed_timers_;

private:
  std::string   base_link_;
  std::string   end_link_;
};

#endif // ADMITTANCE_H