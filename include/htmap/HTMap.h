/*
* This file is part of htmap.
*
* Copyright (C) 2018 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
*
* htmap is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* htmap is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with htmap. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HTMAP_H
#define HTMAP_H

#include <algorithm>

#include <omp.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "htmap/lc/LoopCloser.h"
#include "htmap/map/HighLevelMap.h"
#include "htmap/util/Image.h"
#include "htmap/util/Statistics.h"
#include "htmap/util/Params.h"

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
         ( std::ostringstream() << std::dec << x ) ).str()

namespace htmap
{

class HTMap
{
    public:
        HTMap(const ros::NodeHandle nh);
        ~HTMap();

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
};

}

#endif // HTMAP_H
