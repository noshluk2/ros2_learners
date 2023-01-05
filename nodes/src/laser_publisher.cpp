#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
class LaserPublisher : public rclcpp::Node
{
  public:
    LaserPublisher()
    : Node("minimal_publisher")
    {
      laser_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
      laser_timer = this->create_wall_timer(500ms, std::bind(&LaserPublisher::laser_callback, this));
    }
  private:

    void laser_callback()
    {
      std::vector<float> range(100, 10.0f);

      auto scan_msg = sensor_msgs::msg::LaserScan();
      scan_msg.header.stamp = this->now();
      scan_msg.header.frame_id = "Laser_frame";
      scan_msg.angle_min = 0;
      scan_msg.angle_max = 3.14;
      scan_msg.angle_increment = 0.1;
      scan_msg.range_min = 0.0;
      scan_msg.range_max = 10.0;
      scan_msg.ranges = range;



      // RCLCPP_WARN(this->get_logger(), "Int Pub : '%d'", scan_msg.data);

      laser_publisher->publish(scan_msg);
    }


    rclcpp::TimerBase::SharedPtr laser_timer;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserPublisher>());
  rclcpp::shutdown();
  return 0;
}