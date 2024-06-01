#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class TransformPublisher : public rclcpp::Node {
public:
    TransformPublisher()
    : Node("transform_publisher"), dynamic_broadcaster_(this), static_broadcaster_(this)
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TransformPublisher::timer_callback, this));
        publish_static_transforms();
    }

private:
    void publish_static_transforms() {
        // Static transform from base to camera
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "base";
        static_transform.child_frame_id = "camera";
        static_transform.transform.translation.x = 0.2;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        static_broadcaster_.sendTransform(static_transform);
    }

    void timer_callback() {
        geometry_msgs::msg::TransformStamped dynamic_transform;
        tf2::Quaternion q;

        // Dynamic transform from base to wheel (changing over time)
        dynamic_transform.header.stamp = this->get_clock()->now();
        dynamic_transform.header.frame_id = "base";
        dynamic_transform.child_frame_id = "wheel";
        dynamic_transform.transform.translation.x = 0.3;
        dynamic_transform.transform.translation.y = 0.0;
        dynamic_transform.transform.translation.z = 0.0;

        // Simulating dynamic rotation
        static double angle = 0.0;
        angle += 0.1;  // Increment angle for rotation
        q.setRPY(0, 0, angle);
        dynamic_transform.transform.rotation.x = q.x();
        dynamic_transform.transform.rotation.y = q.y();
        dynamic_transform.transform.rotation.z = q.z();
        dynamic_transform.transform.rotation.w = q.w();

        dynamic_broadcaster_.sendTransform(dynamic_transform);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster dynamic_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
