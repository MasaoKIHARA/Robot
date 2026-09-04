#include "Admittance/DiagRecorder.h"

#include <cstdlib>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

namespace {

std::string expand_home(const std::string& path) {
  if (path.empty() || path[0] != '~') return path;
  const char* home = std::getenv("HOME");
  if (!home) return path;
  return std::string(home) + path.substr(1);
}

std::string safety_mode_name(uint8_t mode) {
  switch (mode) {
    case ur_dashboard_msgs::SafetyMode::NORMAL:                return "NORMAL";
    case ur_dashboard_msgs::SafetyMode::REDUCED:               return "REDUCED";
    case ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP:       return "PROTECTIVE_STOP";
    case ur_dashboard_msgs::SafetyMode::RECOVERY:              return "RECOVERY";
    case ur_dashboard_msgs::SafetyMode::SAFEGUARD_STOP:        return "SAFEGUARD_STOP";
    case ur_dashboard_msgs::SafetyMode::SYSTEM_EMERGENCY_STOP: return "SYSTEM_EMERGENCY_STOP";
    case ur_dashboard_msgs::SafetyMode::ROBOT_EMERGENCY_STOP:  return "ROBOT_EMERGENCY_STOP";
    case ur_dashboard_msgs::SafetyMode::VIOLATION:             return "VIOLATION";
    case ur_dashboard_msgs::SafetyMode::FAULT:                 return "FAULT";
    default:                                                   return "UNKNOWN";
  }
}

} // namespace

DiagRecorder::DiagRecorder(ros::NodeHandle& nh) {
  nh.param("diag/buffer_sec", buffer_sec_, 5.0);
  nh.param("diag/post_trigger_sec", post_trigger_sec_, 2.0);
  std::string dir;
  nh.param<std::string>("diag/dump_dir", dir, "~/admittance_logs");
  dump_dir_ = expand_home(dir);
  mkdir(dump_dir_.c_str(), 0775);  // fails harmlessly if it already exists

  sub_safety_mode_ = nh.subscribe("/ur_hardware_interface/safety_mode", 1,
                                  &DiagRecorder::safety_mode_callback, this);

  ROS_INFO_STREAM("DiagRecorder: keeping " << buffer_sec_ << " s of history, dumping to "
                  << dump_dir_ << " on protective stop.");
}

void DiagRecorder::safety_mode_callback(const ur_dashboard_msgs::SafetyModeConstPtr& msg) {
  const uint8_t previous = safety_mode_;
  safety_mode_ = msg->mode;
  if (previous == safety_mode_) return;

  ROS_WARN_STREAM("Safety mode: " << safety_mode_name(previous) << " -> "
                  << safety_mode_name(safety_mode_));

  // Anything that is not NORMAL means the robot stopped obeying us.
  if (safety_mode_ != ur_dashboard_msgs::SafetyMode::NORMAL &&
      safety_mode_ != 0) {
    arm_dump(safety_mode_name(safety_mode_));
  }
}

void DiagRecorder::arm_dump(const std::string& reason) {
  if (dump_armed_) return;  // a dump is already being collected
  dump_armed_ = true;
  dump_reason_ = reason;
  dump_deadline_ = ros::Time::now() + ros::Duration(post_trigger_sec_);
  ROS_WARN_STREAM("DiagRecorder: armed by " << reason << ", collecting "
                  << post_trigger_sec_ << " s more before writing the log.");
}

void DiagRecorder::push(const admittance_msgs::AdmittanceDiag& sample) {
  buffer_.push_back(sample);

  if (manual_request_.exchange(false)) {
    arm_dump("MANUAL");
  }

  if (dump_armed_) {
    // Keep everything until the deadline, then flush the whole window.
    if (ros::Time::now() >= dump_deadline_) {
      write_csv();
      buffer_.clear();
      dump_armed_ = false;
    }
    return;
  }

  // Normal operation: drop samples older than the history window.
  const ros::Time cutoff = sample.header.stamp - ros::Duration(buffer_sec_);
  while (!buffer_.empty() && buffer_.front().header.stamp < cutoff) {
    buffer_.pop_front();
  }
}

void DiagRecorder::write_csv() {
  if (buffer_.empty()) {
    ROS_WARN("DiagRecorder: nothing buffered, no log written.");
    return;
  }

  std::time_t now = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));

  std::ostringstream path;
  path << dump_dir_ << "/" << stamp << "_" << dump_reason_ << ".csv";

  std::ofstream out(path.str());
  if (!out) {
    ROS_ERROR_STREAM("DiagRecorder: cannot write " << path.str());
    return;
  }

  const std::size_t n_joints = buffer_.back().joint_position.size();

  out << "t,"
      << "fu_x,fu_y,fu_z,tu_x,tu_y,tu_z,"
      << "fb_x,fb_y,fb_z,tb_x,tb_y,tb_z,"
      << "fe_x,fe_y,fe_z,te_x,te_y,te_z,vert_comp,"
      << "vcmd_x,vcmd_y,vcmd_z,wcmd_x,wcmd_y,wcmd_z,"
      << "vmeas_x,vmeas_y,vmeas_z,wmeas_x,wmeas_y,wmeas_z,"
      << "err_lin,err_ang,err_lin_f,err_ang_f,trk_g_lin,trk_g_ang,"
      << "tau0,tau1,tau2,tau3,tau4,tau5,tau_f,tau_arm,vert_comp_raw,"
      << "pos_x,pos_y,pos_z,"
      << "D_x,D_y,D_z,contact_scale,acc_norm,e_yaw,e_pitch,"
      << "acc_clamped,vel_clamped,ang_vel_clamped,slew_clamped,workspace_clamped,";
  for (std::size_t j = 0; j < n_joints; ++j) out << "q" << j << ",";
  for (std::size_t j = 0; j < n_joints; ++j) out << "qd" << j << ",";
  for (std::size_t j = 0; j < n_joints; ++j) out << "eff" << j << ",";
  out << "sigma_min,manipulability,loop_dt,safety_mode,active_behavior\n";

  const double t0 = buffer_.front().header.stamp.toSec();
  out << std::fixed << std::setprecision(6);

  for (const auto& d : buffer_) {
    out << (d.header.stamp.toSec() - t0) << ","
        << d.wrench_user.force.x << "," << d.wrench_user.force.y << "," << d.wrench_user.force.z << ","
        << d.wrench_user.torque.x << "," << d.wrench_user.torque.y << "," << d.wrench_user.torque.z << ","
        << d.wrench_behavior.force.x << "," << d.wrench_behavior.force.y << "," << d.wrench_behavior.force.z << ","
        << d.wrench_behavior.torque.x << "," << d.wrench_behavior.torque.y << "," << d.wrench_behavior.torque.z << ","
        << d.wrench_effective.force.x << "," << d.wrench_effective.force.y << "," << d.wrench_effective.force.z << ","
        << d.wrench_effective.torque.x << "," << d.wrench_effective.torque.y << "," << d.wrench_effective.torque.z << ","
        << d.vertical_compensation << ","
        << d.twist_cmd.linear.x << "," << d.twist_cmd.linear.y << "," << d.twist_cmd.linear.z << ","
        << d.twist_cmd.angular.x << "," << d.twist_cmd.angular.y << "," << d.twist_cmd.angular.z << ","
        << d.twist_meas.linear.x << "," << d.twist_meas.linear.y << "," << d.twist_meas.linear.z << ","
        << d.twist_meas.angular.x << "," << d.twist_meas.angular.y << "," << d.twist_meas.angular.z << ","
        << d.tracking_error_lin << "," << d.tracking_error_ang << ","
        << d.tracking_error_lin_filtered << "," << d.tracking_error_ang_filtered << ","
        << d.tracking_gain_lin << "," << d.tracking_gain_ang << ","
        << (d.tau_ext.size() > 0 ? d.tau_ext[0] : 0.0) << ","
        << (d.tau_ext.size() > 1 ? d.tau_ext[1] : 0.0) << ","
        << (d.tau_ext.size() > 2 ? d.tau_ext[2] : 0.0) << ","
        << (d.tau_ext.size() > 3 ? d.tau_ext[3] : 0.0) << ","
        << (d.tau_ext.size() > 4 ? d.tau_ext[4] : 0.0) << ","
        << (d.tau_ext.size() > 5 ? d.tau_ext[5] : 0.0) << ","
        << d.tau_joint_filtered << "," << d.tau_moment_arm << ","
        << d.vertical_compensation_raw << ","
        << d.position.x << "," << d.position.y << "," << d.position.z << ","
        << d.damping_diag.x << "," << d.damping_diag.y << "," << d.damping_diag.z << ","
        << d.contact_scale << "," << d.acc_norm << "," << d.e_yaw << "," << d.e_pitch << ","
        << int(d.acc_clamped) << "," << int(d.vel_clamped) << "," << int(d.ang_vel_clamped) << ","
        << int(d.slew_clamped) << "," << int(d.workspace_clamped) << ",";

    for (std::size_t j = 0; j < n_joints; ++j)
      out << (j < d.joint_position.size() ? d.joint_position[j] : 0.0) << ",";
    for (std::size_t j = 0; j < n_joints; ++j)
      out << (j < d.joint_velocity.size() ? d.joint_velocity[j] : 0.0) << ",";
    for (std::size_t j = 0; j < n_joints; ++j)
      out << (j < d.joint_effort.size() ? d.joint_effort[j] : 0.0) << ",";

    out << d.sigma_min << "," << d.manipulability << "," << d.loop_dt << ","
        << int(d.safety_mode) << "," << d.active_behavior << "\n";
  }

  out.close();
  ROS_WARN_STREAM("DiagRecorder: wrote " << buffer_.size() << " samples ("
                  << (buffer_.back().header.stamp - buffer_.front().header.stamp).toSec()
                  << " s) to " << path.str());
}
