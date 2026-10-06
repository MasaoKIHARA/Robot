#ifndef ADMITTANCE_DIAG_RECORDER_H
#define ADMITTANCE_DIAG_RECORDER_H

#include "ros/ros.h"
#include "admittance_msgs/AdmittanceDiag.h"
#include "ur_dashboard_msgs/SafetyMode.h"

#include <atomic>
#include <deque>
#include <string>

/**
 * Keeps a rolling window of AdmittanceDiag samples and writes it out as CSV
 * whenever the robot enters a protective stop, so the seconds leading up to the
 * stop can be inspected afterwards.
 *
 * The dump keeps recording for post_trigger_sec after the trigger, so the file
 * covers both sides of the event.
 */
class DiagRecorder
{
public:
  explicit DiagRecorder(ros::NodeHandle& nh);

  /// Store one sample. Called once per diagnostic publish from the control loop.
  void push(const admittance_msgs::AdmittanceDiag& sample);

  /// Latest safety mode reported by the driver (ur_dashboard_msgs/SafetyMode).
  uint8_t safetyMode() const { return safety_mode_; }

  /// Request a dump by hand, e.g. from the keyboard thread. Thread safe.
  void requestManualDump() { manual_request_ = true; }

private:
  void safety_mode_callback(const ur_dashboard_msgs::SafetyModeConstPtr& msg);
  void arm_dump(const std::string& reason);
  void write_csv();

  ros::Subscriber sub_safety_mode_;

  std::deque<admittance_msgs::AdmittanceDiag> buffer_;

  double buffer_sec_;        // how much history to keep before a trigger [s]
  double post_trigger_sec_;  // how long to keep recording after a trigger [s]
  std::string dump_dir_;

  uint8_t safety_mode_ = 0;  // 0 means "not reported yet"
  std::atomic_bool manual_request_{false};

  bool dump_armed_ = false;
  ros::Time dump_deadline_;
  std::string dump_reason_;
};

#endif // ADMITTANCE_DIAG_RECORDER_H
