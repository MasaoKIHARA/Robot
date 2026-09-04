# Diagnostic snapshot of one admittance control cycle.
# Everything is expressed in the robot base frame unless stated otherwise.

Header header

# --- Wrenches [N, Nm] -------------------------------------------------------
# Operator wrench after the dead-zone / saturation stage of state_wrench_callback
geometry_msgs/Wrench wrench_user
# Wrench the active maladaptive behaviors actually apply to the effector.
# This is the negated sum of Behavior::externalWrench(), i.e. the sign it has
# when it enters the admittance dynamics, so a knee buckling shows up here as a
# downward (negative z) force rather than the positive value in BehaviorParams.
geometry_msgs/Wrench wrench_behavior
# wrench_external_ as it actually enters the admittance dynamics this cycle,
# i.e. the operator wrench after the vertical compensation has been folded in.
# Comparing this against wrench_user shows how much of the driving force is
# synthetic rather than applied by the operator.
geometry_msgs/Wrench wrench_effective

# --- Twists [m/s, rad/s] ----------------------------------------------------
# Command actually published to the cartesian velocity controller
geometry_msgs/Twist twist_cmd
# End effector twist measured by the controller (/ee_state)
geometry_msgs/Twist twist_meas
# |v_cmd - v_meas|. This is the quantity the UR protective stop reacts to.
float64 tracking_error_lin
float64 tracking_error_ang

# --- Admittance internals ---------------------------------------------------
geometry_msgs/Point position          # end effector position
geometry_msgs/Vector3 damping_diag    # D_(0,0), D_(1,1), D_(2,2) [Ns/m]
float64 contact_scale                 # low pass filtered contact gate, 0..1
float64 acc_norm                      # peak |a| before the acceleration clamp,
                                      # since the previous sample [m/s^2]
float64 vertical_compensation         # synthetic downward force added by the
                                      # distance-based compensation [N]
float64 e_yaw                         # yaw error w.r.t. the desired pose [rad]
float64 e_pitch                       # pitch error w.r.t. the desired pose [rad]

# --- Which limiter fired since the previous sample --------------------------
bool acc_clamped
bool vel_clamped
bool ang_vel_clamped
bool slew_clamped
bool workspace_clamped

# --- Robot side -------------------------------------------------------------
# Ordered along the kinematic chain (shoulder_pan .. wrist_3)
float64[] joint_position              # [rad]
float64[] joint_velocity              # [rad/s]
float64[] joint_effort                # driver dependent unit
float64 sigma_min                     # smallest singular value of the Jacobian
float64 manipulability                # sqrt(det(J J^T))

# --- Health -----------------------------------------------------------------
float64 loop_dt                       # measured cycle time [s]
uint8 safety_mode                     # ur_dashboard_msgs/SafetyMode, 0 = unknown
string active_behavior                # name of the active behavior, "" if none
