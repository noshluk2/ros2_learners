
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;


int main() {

  auto space = std::make_shared<ob::RealVectorStateSpace>(2);
  ob::RealVectorBounds bounds(2); bounds.setLow(-1.0); bounds.setHigh(1.0);
  space->setBounds(bounds);

  std::cout << "OMPL OK "<< std::endl;

    return 0;
}
