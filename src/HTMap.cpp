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

#include "htmap/HTMap.h"

namespace htmap
{

HTMap::HTMap(const ros::NodeHandle nh)
    : _nh(nh),
      _params(0),
      _st(0)
{
    ROS_INFO("Initializing node");

    _st = Statistics::getInstance();

    ROS_INFO("Reading parameters ...");
    _params = Params::getInstance();
    _params->readParams(nh);
    ROS_INFO("Parameters read");

    if (_params->batch)
    {
        processBatch();
    }
    else
    {
        process();
    }
}

HTMap::~HTMap()
{
}

void HTMap::process()
{
    ROS_INFO("Mapping ...");

    // Variables to control the mapping time
    double init_time, end_time;
    init_time = omp_get_wtime();

    // STEP 1 ---- Image Description ----
    ROS_INFO("--- Image description ---");
    _params->images.resize(_params->nimages, "");
    describeImages(_params->images);
    // ---- END Image Description ----

    // STEP 2 ---- Mapping ----
    ROS_INFO("--- Mapping ---");
    HighLevelMap hmap;
    LoopCloser lc;
    map(_params->images, hmap, lc);
    // ---- END Mapping ----

    // STEP 3 ---- Writing results ----
    ROS_INFO("--- Writing results ---");
    _st->writeResults(_params->dir_results, _params->imageLC_min_inliers);
    // ---- END Results ----

    end_time = omp_get_wtime();
    ROS_INFO("Total time: %.5f seconds", end_time - init_time);
    ROS_INFO("Process finished");
}

void HTMap::processBatch()
{
    ROS_INFO("Mapping ...");

    // STEP 1 ---- Image Description ----
    ROS_INFO("--- Image description ---");
    _params->images.resize(_params->nimages, "");
    describeImages(_params->images);
    // ---- END Image Description ----

    for (int curr_inl = _params->inliers_begin; curr_inl < _params->inliers_end; curr_inl += _params->inliers_step)
    {
        // Updating information
        _params->imageLC_min_inliers = curr_inl;
        _st->init(_params->nimages);

        // STEP 2 ---- Mapping ----
        ROS_INFO("--- Mapping ---");
        HighLevelMap hmap;
        LoopCloser lc;
        map(_params->images, hmap, lc);
        // ---- END Mapping ----

        // STEP 3 ---- Writing results ----
        ROS_INFO("--- Writing results ---");
        _st->writeResults(_params->dir_results, _params->imageLC_min_inliers);
    }
    // ---- END Results ----

    ROS_INFO("Process finished");
}

void HTMap::describeImages(std::vector<std::string>& images)
{
    if (_params->load_features)
    {
        images.clear();
        ROS_INFO("Images will be loaded from disk.");
        getFilenames(_params->dir_results + "images/", images, false);
    }
    else
    {
        // Preparing the resulting directory for images.
        ROS_INFO("Preparing image directory ...");
        boost::filesystem::path res_imgs_dir = _params->dir_results + "images/";
        boost::filesystem::remove_all(res_imgs_dir);
        boost::filesystem::create_directory(res_imgs_dir);
        ROS_INFO("Image directory ready");

        #pragma omp parallel for
        for (unsigned image_ind = 0; image_ind < _params->nimages; image_ind++)
        {
            ROS_INFO("Describing image %i", image_ind);
            cv::Mat image = cv::imread(_params->filenames[image_ind]);

            std::string image_filename = _params->dir_results + "images/image%06d.bmp";
            char name[500];
            sprintf(name, image_filename.c_str(), image_ind);
            cv::imwrite(name, image);

            // Detecting and describing key points.
            std::vector<cv::KeyPoint> kps;
            cv::Mat dsc;
            cv::Mat gdsc;
            double init_time = omp_get_wtime();
            _params->detector->detect(image, kps);
            double end_time = omp_get_wtime();
            double ftime = end_time - init_time;
            init_time = omp_get_wtime();
            _params->descriptor->describe(image, kps, dsc);
            end_time = omp_get_wtime();
            double ltime = end_time - init_time;
            init_time = omp_get_wtime();
            _params->gdescriptor->describe(image, gdsc);
            end_time = omp_get_wtime();
            double ptime = end_time - init_time;
            gdsc.convertTo(gdsc, CV_64F);

            _st->registerDescTimes(ptime, ftime, ltime);

            ROS_INFO("%lu keypoints found.", kps.size());

            Image curr_image;
            curr_image.image_id = image_ind;
            curr_image.image_filename = std::string(name);
            curr_image.image = image;
            curr_image.kps = kps;
            curr_image.dscs = dsc;
            curr_image.gdsc = gdsc;

            std::string yfilename = _params->dir_results + "images/" + "image%06d.yml";
            sprintf(name, yfilename.c_str(), image_ind);
            curr_image.save(name);

            images[image_ind] = std::string(name);
        }
    }
}

void HTMap::map(const std::vector<std::string>& images, HighLevelMap& map, LoopCloser& _lc)
{
    // Initializing the first location.
    ROS_INFO("Adding first image to location 0.");
    Location* loc0 = new Location(_params->loc_max_images, _params->max_total_kps, _params->descriptor->getDescSize());

    // Adding the loc0 as the first location of the map.
    ROS_INFO("Adding location 0 to the map.");
    map.addLocation(loc0);
    map.setActive(0);

    // Load the first image.
    Image img0;
    img0.load(images[0]);
    unsigned l1,l2;
    _lc.process(img0, map, l1, l2);
    map.addImageToLocation(0, 0, images[0]);
    _st->registerImageToLocation(0, 0);

    // Processing the remaining images.
    for (unsigned img_idx = 1; img_idx < _params->nimages; img_idx++)
    {
        ROS_INFO("Processing image %u", img_idx);

        // Load the current image.
        Image img;
        img.load(images[img_idx]);

        // Loop Closure ?
        unsigned loop_loc;
        unsigned loop_img;
        bool lclosure;
        lclosure = _lc.process(img, map, loop_loc, loop_img);

        if (lclosure)
        {
            ROS_INFO("-----> Loop detected: %u --> (N %u, I %u)", img_idx, loop_loc, loop_img);

            // Registering the loop.
            _st->registerLoop(img_idx, loop_img);

            if (loop_loc != map.active)
            {
                // Changing active node.
                ROS_INFO("Changing active location to %u", loop_loc);
                map.linkLocations(map.active, loop_loc, 1.0);
                map.setActive(loop_loc);
            }
        }
        else
        {
            if (isNewLocation(img, map))
            {
                // New location.
                Location* loc = new Location(_params->loc_max_images, _params->max_total_kps, _params->descriptor->getDescSize());
                map.addLocation(loc);
                map.linkLocations(map.active, loc->id, 1.0);
                map.setActive(loc->id);
                ROS_INFO("Location %u added to the map.", loc->id);
            }
        }

        // Adding the image to the current active node.
        unsigned curr_lid = map.getActiveLocation()->id;
        map.addImageToLocation(curr_lid, img_idx, images[img_idx]);
        _st->registerImageToLocation(img_idx, curr_lid);
    }
}

bool HTMap::isNewLocation(const Image& img, HighLevelMap& map)
{
    // Get the current active location in the map.
    Location* loc = map.getActiveLocation();

    // If we achive the maximum number of images in a node.
    if (loc->numImages() > _params->loc_max_images)
    {
        return true;
    }

    double dist = GlobalDescriptor::dist(loc->desc, img.gdsc, _params->gdescriptor);
	//ROS_INFO("Distance %f", dist);
    if (dist > _params->max_sim_newnode)
    {
        return true;
    }
    else
    {
        return false;
    }
}

}
