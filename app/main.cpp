
#include <iostream>

#include "base_slam/visual_odom.hpp"


int main()
{
    std::string ddir = "../config/config.yaml";
    base_slam::VisualOdom::Ptr vo(new base_slam::VisualOdom(ddir));
    assert(vo->init() == true);
    vo->run();
    return 0;
}
