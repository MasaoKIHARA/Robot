#include <Admittance/Admittance.h>

Admittance::Admittance(ros::NodeHandle &n,
    double frequency,
    std::string topic_arm_state,
    std::string topic_arm_command,
    std::string topic_wrench_state,
    std::vector<double> M,
    std::vector<double> D,
    std::vector<double> K,
    std::vector<double> B,
    std::vector<double> C,
    std::vector<double> desired_pose,
    std::string base_link,
    std::string end_link,
    double arm_max_vel,
    double arm_max_acc,
    double min_Z_height,
    double max_Z_height)://
  nh_(n), loop_rate_(frequency),
  M_(M.data()), D_(D.data()),K_(K.data()),desired_pose_(desired_pose.data()),
  arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
  base_link_(base_link), end_link_(end_link),
  min_Z_height_(min_Z_height), max_Z_height_(max_Z_height), z_limit_warned_(false){

  //* Subscribers
  sub_arm_state_           = nh_.subscribe(topic_arm_state, 5, 
      &Admittance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_state_        = nh_.subscribe(topic_wrench_state, 5,
      &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  latest_waist_angle_ = 0.0f;										 
  sub_waist_angle_ = nh_.subscribe("/waist_angle", 1, &Admittance::waist_angle_callback, this);	 
  ROS_INFO("Subscribing to /waist_angle");

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


  ft_arm_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;
  force_x_pre = 0;
  force_y_pre = 0;
  force_z_pre = 0;
  torque_x_pre = 0;
  torque_y_pre = 0;
  torque_z_pre = 0;
  wait_for_transformations();

  // Convert std::vector to Eigen::VectorXd
  B_ = Eigen::Map<const Eigen::VectorXd>(B.data(), B.size());
  C_ = Eigen::Map<const Eigen::VectorXd>(C.data(), C.size());
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

    compute_admittance();

    send_commands_to_robot();

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

//!-                Admittance Dynamics                  -!//

void Admittance::waist_angle_callback(const std_msgs::Float32ConstPtr& msg){
  latest_waist_angle_ = msg->data;
  ROS_DEBUG("Received waist angle: %.2f", latest_waist_angle_);
}

void Admittance::compute_admittance() {

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


 // Determine the desired_accelaration
 
  double theta = latest_waist_angle_;	
  Eigen::VectorXd F_pat_ = B_ * theta + C_;

  coupling_wrench_arm=  D_ * (arm_desired_twist_adm_) + K_*error;
  arm_desired_accelaration = M_.inverse() * ( - coupling_wrench_arm  + wrench_external_ + F_pat_);

  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();

  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }
  // Integrate for velocity based interface
  
  ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_adm_ += arm_desired_accelaration * duration.toSec();
  last_acceleration_x_ = arm_desired_twist_adm_(0);
  last_acceleration_y_ = arm_desired_twist_adm_(1);
  last_acceleration_z_ = arm_desired_twist_adm_(2);
  
  if (arm_position_(2) <= min_Z_height_ && wrench_external_(2) + F_pat_(2) < 0){
    arm_desired_twist_adm_.setZero();
    if (!z_limit_warned_) {
       ROS_WARN("[Admittance] Effector Z (%.3f) <= min_z_height (%.3f): All arm commands set to 0", arm_position_(2), min_Z_height_);
       z_limit_warned_ = true;
    } 
  }
  else{
    if (z_limit_warned_) {
       ROS_INFO("[Admittance] Effector Z height (%.3f) recovered above min_z_height (%.3f): Control resumes", arm_position_(2), min_Z_height_);
       z_limit_warned_ = false;
    } 
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
                        0,0,
                        // msg->wrench.torque.x,
                        // msg->wrench.torque.y,
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
  double norm_vel_des = (arm_desired_twist_adm_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

    arm_desired_twist_adm_.segment(0, 3) *= (arm_max_vel_);

  }
  
  geometry_msgs::Twist arm_twist_cmd;
  arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_adm_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_adm_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_adm_(5);

  pub_arm_cmd_.publish(arm_twist_cmd);
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

