#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

int main() {

   pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.emplace_back(1.0f, 2.0f, 3.0f);

    std::cout << "PCL OK "<<std::endl;
    return 0;
}
