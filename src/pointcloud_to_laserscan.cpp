#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class PointCloud2ToLaserScanNode : public rclcpp::Node
{
public:
  PointCloud2ToLaserScanNode()
  : Node("pointcloud2_to_laserscan")
  {
    this->declare_parameter<double>("min_z", -1.0);
    this->declare_parameter<double>("max_z", 1.0);
    
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();

    pointcloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10, std::bind(&PointCloud2ToLaserScanNode::pointCloud2Callback, this, std::placeholders::_1));

    laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("output_laserscan", 10);
  }

private:
  void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Create LaserScan message
    sensor_msgs::msg::LaserScan laser_scan;
    laser_scan.header = msg->header;
    //
    // Configure LaserScan message (you should adjust these values according to your needs)
    laser_scan.angle_min = -M_PI;
    laser_scan.angle_max = M_PI;
    laser_scan.angle_increment = M_PI / 180.0;
    laser_scan.time_increment = 0.0;
    laser_scan.scan_time = 0.1;
    laser_scan.range_min = 0.1;
    laser_scan.range_max = 100.0;
    int num_readings = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment;
    laser_scan.ranges.assign(num_readings, laser_scan.range_max + 1.0);

    // Iterate over PointCloud2 data
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (*iter_z >= min_z_ && *iter_z <= max_z_)
      {
        double range = std::sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y));
        double angle = std::atan2(*iter_y, *iter_x);

        // Find the corresponding index in the LaserScan message
        int index = (angle - laser_scan.angle_min) / laser_scan.angle_increment;

        // Update the range value if the current point is closer than the previous one
        if (index >= 0 && index < num_readings && range < laser_scan.ranges[index])
        {
          laser_scan.ranges[index] = range;
        }
      }
    }

    // Publish the LaserScan message
    laserscan_pub_->publish(laser_scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
  double min_z_;
  double max_z_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2ToLaserScanNode>());
  rclcpp::shutdown();
  return 0;
}
