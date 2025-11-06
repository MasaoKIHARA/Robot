#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <deque>

class AverageFilter
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber wrench_sub_;
  ros::Publisher filtered_wrench_pub_;

  std::deque<geometry_msgs::WrenchStamped> wrench_buffer_;
  const int buffer_size_ = 10; // Number of samples to average

public:
  AverageFilter()
  {
    wrench_sub_ = nh_.subscribe("/wrench", 10, &AverageFilter::wrenchCallback, this);
    filtered_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/avg_filtered_wrench", 10);
  }

  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg)
  {
    wrench_buffer_.push_front(*wrench_msg);
    if (wrench_buffer_.size() > buffer_size_)
    {
      wrench_buffer_.pop_back();
    }

    geometry_msgs::WrenchStamped filtered_wrench_msg;
    filtered_wrench_msg.header = wrench_msg->header;

    // Initialize sums
    double sum_fx = 0.0;
    double sum_fy = 0.0;
    double sum_fz = 0.0;
    double sum_tx = 0.0;
    double sum_ty = 0.0;
    double sum_tz = 0.0;

    // Compute the sum of force and torque components
    for (const auto& wrench : wrench_buffer_)
    {
      sum_fx += wrench.wrench.force.x;
      sum_fy += wrench.wrench.force.y;
      sum_fz += wrench.wrench.force.z;
      sum_tx += wrench.wrench.torque.x;
      sum_ty += wrench.wrench.torque.y;
      sum_tz += wrench.wrench.torque.z;
    }

    // Compute the average force and torque components
    filtered_wrench_msg.wrench.force.x = sum_fx / wrench_buffer_.size();
    filtered_wrench_msg.wrench.force.y = sum_fy / wrench_buffer_.size();
    filtered_wrench_msg.wrench.force.z = sum_fz / wrench_buffer_.size();
    filtered_wrench_msg.wrench.torque.x = sum_tx / wrench_buffer_.size();
    filtered_wrench_msg.wrench.torque.y = sum_ty / wrench_buffer_.size();
    filtered_wrench_msg.wrench.torque.z = sum_tz / wrench_buffer_.size();

    filtered_wrench_pub_.publish(filtered_wrench_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "average_filter_node");
  AverageFilter average_filter;
  ros::spin();
  return 0;
}