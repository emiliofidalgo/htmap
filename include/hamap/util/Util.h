#ifndef UTIL_HAMAP_H
#define UTIL_HAMAP_H

#include <stdio.h>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <opencv2/opencv.hpp>

#include <hamap/util/Image.h>

namespace hamap
{    
    void ratioMatching(const Image& query, const Image& train, const double& ratio, std::vector<cv::DMatch>& matches);
    int checkEpipolarGeometry(const Image& query, const Image& train, const std::vector<cv::DMatch>& matches);
    double distEuclidean(const cv::Mat& a, const cv::Mat& b);
    double distMahalanobis(const cv::Mat& a, const cv::Mat& b, const cv::Mat& icovar);
    int distHamming(const uchar* a, const uchar* b, const int n);
    double distChiSquare(const cv::Mat& a, const cv::Mat& b);
    double distBhattacharyya(const cv::Mat& a, const cv::Mat& b);
    void getFilenames(const std::string& directory, std::vector<std::string>& filenames, bool images = true);
}

#endif
