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

#ifndef _PARAMS_H
#define _PARAMS_H

#include <ros/ros.h>

#include "htmap/imgdesc/GlobalDescriptor.h"
#include "htmap/imgdesc/KeypointDescriptor.h"
#include "htmap/imgdesc/KeypointDetector.h"
#include "htmap/util/Statistics.h"
#include "htmap/util/Util.h"

namespace htmap
{

class Params
{
    public:
        // Global Parameters
        std::string dir_images;
        bool load_features;
        std::string dir_results;
        std::vector<std::string> filenames;
        std::vector<std::string> images;
        unsigned nimages;
        int match_method;
        double match_ratio;
        double locLC_thresh;
        int imageLC_min_inliers;
        int imageLC_disc_recent;
        double imageLC_tloop;
        int loc_max_images;
        int max_total_kps;
        double max_sim_newnode;

        // Batch parameters
        bool batch;
        int inliers_begin;
        int inliers_end;
        int inliers_step;

        // Keypoint Detector parameters
        std::string detector_name;
        KeypointDetector* detector;
        KeypointDetectorParams det_params;

        // Keypoint Descriptor parameters
        std::string descriptor_name;
        KeypointDescriptor* descriptor;
        KeypointDescriptorParams des_params;

        // Global Descriptor parameters
        std::string gdescriptor_name;
        GlobalDescriptor* gdescriptor;
        GlobalDescriptorParams gdes_params;

        // Public functions.
        static Params* getInstance();
        void readParams(const ros::NodeHandle& nh);

    protected:
        // Protected constructor. Singleton class.
        Params() :
            nimages(0),
            detector(0),
            descriptor(0),
            gdescriptor(0),
            load_features(false),
            match_method(1),
            match_ratio(0.8),
            locLC_thresh(0.6),
            imageLC_min_inliers(50),
            imageLC_disc_recent(70),
            imageLC_tloop(0.75),
            loc_max_images(300),
            max_total_kps(4000),
            max_sim_newnode(0.20),
            batch(false),
            inliers_begin(20),
            inliers_end(200),
            inliers_step(15)
        {
        }

        ~Params()
        {
            delete detector;
            delete descriptor;
            delete gdescriptor;
            delete _instance;
        }

        Params(const Params &);
        Params& operator=(const Params &);

    private:
        // Single instance.
        static Params* _instance;
};

}
#endif /* _PARAMS_H */
