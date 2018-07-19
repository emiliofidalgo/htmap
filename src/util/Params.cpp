#include "hamap/util/Params.h"

namespace hamap
{

Params* Params::_instance = 0;

Params* Params::getInstance()
{
	if (_instance == 0)
	{
        _instance = new Params;
	}
    return _instance;
}

void Params::readParams(const ros::NodeHandle& nh)
{
    nh.param<std::string>("images_dir", dir_images, "");
    ROS_INFO("[Params] Image directory: %s", dir_images.c_str());

    nh.param("load_feats", load_features, false);
    ROS_INFO("[Params] Load features: %i", load_features ? 1 : 0);

    getFilenames(dir_images, filenames);
    nimages = filenames.size();
    ROS_INFO("[Params] %u images found", nimages);

    nh.param<std::string>("working_dir", dir_results, "");
    ROS_INFO("[Params] Working directory: %s", dir_results.c_str());

    // Configuring the statistics directory.
    Statistics* st = Statistics::getInstance();
    st->init(nimages);

    std::string match_method_st;
    nh.param<std::string>("match_method", match_method_st, "ratio");
    if (match_method_st == "ratio")
        match_method = 1;
    else if (match_method_st == "crosscheck")
        match_method = 0;
    ROS_INFO("[Params] Matching Method %s", match_method_st.c_str());

    nh.param("match_ratio", match_ratio, 0.8);
    ROS_INFO("[Params] Matching Ratio %f", match_ratio);

    nh.param<std::string>("detector", detector_name, "FAST");
    ROS_INFO("[Params] Keypoint Detector: %s", detector_name.c_str());
    detector = hamap::KeypointDetector::create(detector_name, det_params);

    bool grid;
    int grid_rows, grid_cols;
    nh.param("grid", grid, true);
    nh.param("grid_rows", grid_rows, 4);
    nh.param("grid_cols", grid_cols, 4);
    nh.param("max_total_kps", max_total_kps, 4000);
    ROS_INFO("[Params] Grid: %i (%i, %i), Total Features: %i", grid ? 1 : 0, grid_rows, grid_cols, max_total_kps);
    if (grid)
    {
        detector->_detector = hamap::convertToGridDetector(grid_rows, grid_cols, max_total_kps, detector->_detector);
    }

    nh.param<std::string>("descriptor", descriptor_name, "BRIEF");
    ROS_INFO("[Params] Keypoint Descriptor: %s", descriptor_name.c_str());
    descriptor = hamap::KeypointDescriptor::create(descriptor_name, des_params);

    nh.param<std::string>("gdescriptor", gdescriptor_name, "WI-SIFT");
    ROS_INFO("[Params] Global Descriptor: %s", gdescriptor_name.c_str());
    gdescriptor = GlobalDescriptor::create(gdescriptor_name, gdes_params);

    nh.param("locLC_thresh", locLC_thresh, 0.6);
    ROS_INFO("[Params] Location LC Threshold: %f", locLC_thresh);

    nh.param("imageLC_min_inliers", imageLC_min_inliers, 50);
    ROS_INFO("[Params] Image LC Minimum Number of Inliers: %i", imageLC_min_inliers);

    nh.param("imageLC_tloop", imageLC_tloop, 0.75);
    ROS_INFO("[Params] Image LC Minimum Threshold for LC: %f", imageLC_tloop);

    nh.param("imageLC_disc_recent", imageLC_disc_recent, 70);
    ROS_INFO("[Params] Number of Recent Images Discarded: %i", imageLC_disc_recent);

    nh.param("loc_max_images", loc_max_images, 300);
    ROS_INFO("[Params] Max Images per Location: %i", loc_max_images);

    nh.param("max_sim_newnode", max_sim_newnode, 0.20);
    ROS_INFO("[Params] Max Similarity for a New Node: %f", max_sim_newnode);

    nh.param("batch", batch, false);
    ROS_INFO("[Params] Batch Process: %i", batch ? 1 : 0);

    nh.param("inliers_begin", inliers_begin, 20);
    ROS_INFO("[Params] Batch Inliers Begin: %i", inliers_begin);

    nh.param("inliers_end", inliers_end, 200);
    ROS_INFO("[Params] Batch Inliers End: %i", inliers_end);

    nh.param("inliers_step", inliers_step, 15);
    ROS_INFO("[Params] Batch Inliers Step: %i", inliers_step);
}

}
