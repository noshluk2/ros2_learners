#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PclOmplNode : public rclcpp::Node {
public:
  PclOmplNode() : Node("pcl_ompl_node") {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/points",  10,
        std::bind(&PclOmplNode::cloud_cb, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "PCL+OMPL node ready.");
  }

private:
  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // --- PCL: convert & downsample ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *in);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(0.10f, 0.10f, 0.10f); // 10cm voxels
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*ds);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Downsampled cloud: %zu -> %zu points",
                         in->size(), ds->size());

    // --- OMPL: trivial 2-D plan (all states valid) ---
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2); bounds.setLow(-1.0); bounds.setHigh(6.0);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ob::State*) { return true; }); // ignore obstacles (minimal demo)

    ob::ScopedState<> start(space); start[0] = 0.0; start[1] = 0.0;
    ob::ScopedState<> goal(space);  goal[0]  = 5.0; goal[1]  = 5.0;
    ss.setStartAndGoalStates(start, goal);

    if (ss.solve(0.05)) {
      RCLCPP_INFO(get_logger(), "OMPL plan found. length=%.3f",
                  ss.getSolutionPath().length());
    } else {
      RCLCPP_WARN(get_logger(), "OMPL no plan (timeout).");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclOmplNode>());
  rclcpp::shutdown();
  return 0;
}
