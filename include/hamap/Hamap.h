#ifndef HAMAP_H
#define HAMAP_H

#include <algorithm>

#include <omp.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "hamap/lc/LoopCloser.h"
#include "hamap/map/HighLevelMap.h"
#include "hamap/util/Image.h"
#include "hamap/util/Statistics.h"
#include "hamap/util/Params.h"

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
         ( std::ostringstream() << std::dec << x ) ).str()

namespace hamap
{

class Hamap
{
    public:
        Hamap(const ros::NodeHandle nh);
        ~Hamap();

        void process();
        void processBatch();

private:
        // ROS
        ros::NodeHandle _nh;

        // Parameters
        Params* _params;

        // Statistics
        Statistics* _st;        

        void describeImages(std::vector<std::string>& images);
        void map(const std::vector<std::string>& images, HighLevelMap& map, LoopCloser& _lc);
        bool isNewLocation(const Image& img, HighLevelMap& map);
		void saveDescriptorInfo(const std::string& dir, HighLevelMap& map);
};

}

#endif // HAMAP_H
