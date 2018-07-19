#include "hamap/Hamap.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hamap");
    ros::NodeHandle nh_private("~");
    hamap::Hamap hamap(nh_private);
    ros::spin();
    return 0;
}
