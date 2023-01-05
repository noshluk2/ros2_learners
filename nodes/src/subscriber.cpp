#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_1 = this->create_subscription<std_msgs::msg::String>("topic_1", 10, std::bind(&MinimalSubscriber::topic_callback_1, this, _1));

      subscription_2 = this->create_subscription<std_msgs::msg::Int16>("topic_2", 10, std::bind(&MinimalSubscriber::topic_callback_2, this, _1));
    }

  private:
    void topic_callback_1(const std_msgs::msg::String & msg) const
    {
      RCLCPP_WARN(this->get_logger(), "String Sub: '%s'", msg.data.c_str());
    }

    void topic_callback_2(const std_msgs::msg::Int16 & msg) const
    {
      RCLCPP_FATAL(this->get_logger(), "Int sub '%d'", msg.data);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}