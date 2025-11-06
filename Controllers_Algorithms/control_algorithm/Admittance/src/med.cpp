#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <deque>

class MedianFilter
{
public:
  MedianFilter()
  {
    // Subscribe to the "/wrench" topic
    wrench_sub_ = nh_.subscribe("/wrench", 10, &MedianFilter::wrenchCallback, this);

    // Advertise the "/median_filtered_wrench" topic
    median_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/median_filtered_wrench", 10);
  }

  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    // Add the new wrench data to the deque
    wrench_data_.push_back(*msg);

    // Maintain the deque size to the past 10 samples
    if (wrench_data_.size() > 10)
    {
      wrench_data_.pop_front();
    }

    // Process the median filtered data
    geometry_msgs::WrenchStamped median_filtered_wrench = calculateMedianFilteredWrench();

    // Publish the median filtered wrench data
    median_wrench_pub_.publish(median_filtered_wrench);
  }

  geometry_msgs::WrenchStamped calculateMedianFilteredWrench()
  {
    // Separate force and torque components
    std::vector<double> force_x, force_y, force_z;
    std::vector<double> torque_x, torque_y, torque_z;

    for (const auto& wrench : wrench_data_)
    {
      force_x.push_back(wrench.wrench.force.x);
      force_y.push_back(wrench.wrench.force.y);
      force_z.push_back(wrench.wrench.force.z);

      torque_x.push_back(wrench.wrench.torque.x);
      torque_y.push_back(wrench.wrench.torque.y);
      torque_z.push_back(wrench.wrench.torque.z);
    }

    // Sort the values in ascending order
    std::sort(force_x.begin(), force_x.end());
    std::sort(force_y.begin(), force_y.end());
    std::sort(force_z.begin(), force_z.end());

    std::sort(torque_x.begin(), torque_x.end());
    std::sort(torque_y.begin(), torque_y.end());
    std::sort(torque_z.begin(), torque_z.end());

    // Calculate the median value for each component
    double median_force_x = force_x[force_x.size() / 2];
    double median_force_y = force_y[force_y.size() / 2];
    double median_force_z = force_z[force_z.size() / 2];

    double median_torque_x = torque_x[torque_x.size() / 2];
    double median_torque_y = torque_y[torque_y.size() / 2];
    double median_torque_z = torque_z[torque_z.size() / 2];

    // Create a new WrenchStamped message with the median filtered values
    geometry_msgs::WrenchStamped median_filtered_wrench;
    median_filtered_wrench.header = wrench_data_.back().header;
    median_filtered_wrench.wrench.force.x = median_force_x;
    median_filtered_wrench.wrench.force.y = median_force_y;
    median_filtered_wrench.wrench.force.z = median_force_z;
    median_filtered_wrench.wrench.torque.x = median_torque_x;
    median_filtered_wrench.wrench.torque.y = median_torque_y;
    median_filtered_wrench.wrench.torque.z = median_torque_z;

    return median_filtered_wrench;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber wrench_sub_;
  ros::Publisher median_wrench_pub_;
  std::deque<geometry_msgs::WrenchStamped> wrench_data_;
};

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "median_filter_node");

  // Create an instance of the MedianFilter class
  MedianFilter median_filter;

  // Spin the node and process callbacks
  ros::spin();

  return 0;
}