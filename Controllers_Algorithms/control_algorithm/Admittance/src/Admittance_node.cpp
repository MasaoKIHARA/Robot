#include "ros/ros.h"
#include "Admittance/Admittance.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_node");

    ros::NodeHandle nh;
    double frequency = 500.0;

    // Parameters
    std::string topic_arm_state;
    std::string topic_arm_command;
    std::string topic_wrench_state;
    std::string base_link;
    std::string end_link;

    std::vector<double> M;
    std::vector<double> D;
    std::vector<double> K;
    std::vector<double> B;
    std::vector<double> C;
    std::vector<double> desired_pose;
    
    double arm_max_vel;
    double arm_max_acc;
    double arm_max_ang_vel;
    double arm_max_ang_acc;

    double min_Z_height;
    double max_Z_height;


    // LOADING PARAMETERS FROM THE ROS SERVER 

    // Topic names
    if (!nh.getParam("topic_arm_state", topic_arm_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the arm."); return -1; }
    if (!nh.getParam("topic_arm_command", topic_arm_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the arm."); return -1; }
    if (!nh.getParam("topic_wrench_state", topic_wrench_state)) { ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor."); return -1; }
    // ADMITTANCE PARAMETERS
    if (!nh.getParam("mass_arm", M)) { ROS_ERROR("Couldn't retrieve the desired mass of the arm."); return -1; }
    if (!nh.getParam("damping_arm", D)) { ROS_ERROR("Couldn't retrieve the desired damping of the coupling."); return -1; }
    if (!nh.getParam("stiffness_coupling", K)) { ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling."); return -1; }
    if (!nh.getParam("coefficients_theta", B)){ ROS_ERROR("Couldn't retrieve the desired coefficients for theta."); return -1; }
    if (!nh.getParam("offset_theta", C)){ ROS_ERROR("Couldn't retrieve the desired offset for theta."); return -1; }
    if (!nh.getParam("desired_pose", desired_pose)) { ROS_ERROR("Couldn't retrieve the desired pose of the spring."); return -1; }
    if (!nh.getParam("arm_max_vel", arm_max_vel)) { ROS_ERROR("Couldn't retrieve the max velocity for the arm."); return -1;}
    if (!nh.getParam("arm_max_acc", arm_max_acc)) { ROS_ERROR("Couldn't retrieve the max acceleration for the arm."); return -1;}
    if (!nh.getParam("arm_max_ang_vel", arm_max_ang_vel)) { ROS_ERROR("Couldn't retrieve the max angular velocity for the arm."); return -1;}
    if (!nh.getParam("arm_max_ang_acc", arm_max_ang_acc)) { ROS_ERROR("Couldn't retrieve the max angular acceleration for the arm."); return -1;}
    if (!nh.getParam("min_Z_height", min_Z_height)) { ROS_ERROR("Couldn't retrieve the min height of the effector."); return -1;}
    if (!nh.getParam("max_Z_height", max_Z_height)) { ROS_ERROR("Couldn't retrieve the max height of the effector."); return -1;}
    if (!nh.getParam("base_link", base_link)) { ROS_ERROR("Couldn't retrieve the base_link."); return -1; }
    if (!nh.getParam("end_link", end_link)) { ROS_ERROR("Couldn't retrieve the end_link."); return -1; } 

    // Constructing the controller
    Admittance admittance(
        nh,
        frequency,
        topic_arm_state,
        topic_arm_command,
        topic_wrench_state,
        M, D, K, B, C, desired_pose,
        arm_max_vel,
        arm_max_acc,
        arm_max_ang_vel,
        arm_max_ang_acc,
        min_Z_height,
        max_Z_height,
        base_link,
        end_link
    );

    // Running the controller
    admittance.run();

    return 0;
}