#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;
class OccupancyGrid_Publisher : public rclcpp::Node
{
  public:
    OccupancyGrid_Publisher()
    : Node("occupancy_grid_publisher")
    {
      og_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_occupancy_grid", 10);
      og_timer = this->create_wall_timer(500ms, std::bind(&OccupancyGrid_Publisher::og_callback, this));
    }
  private:

    void og_callback()
    {

      auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
      std::vector<signed char> og_array(35);
      for(int i=0;i<35;i++){
        og_array[i] = i % 3 == 0 ? 100 : 0 ;
      }

      occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
      occupancy_grid_msg.header.frame_id = "map_frame";

      occupancy_grid_msg.info.resolution = 1;

      occupancy_grid_msg.info.width = 5;
      occupancy_grid_msg.info.height = 7;

      occupancy_grid_msg.info.origin.position.x = 0.0;
      occupancy_grid_msg.info.origin.position.y = 0.0;
      occupancy_grid_msg.info.origin.position.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.x = 0.0;
      occupancy_grid_msg.info.origin.orientation.y = 0.0;
      occupancy_grid_msg.info.origin.orientation.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.w = 1.0;
      occupancy_grid_msg.data = og_array;

      og_pub->publish(occupancy_grid_msg);
    }


    rclcpp::TimerBase::SharedPtr og_timer;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
  rclcpp::shutdown();
  return 0;
}