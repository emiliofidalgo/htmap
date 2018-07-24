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

#ifndef UTIL_HAMAP_H
#define UTIL_HAMAP_H

#include <stdio.h>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <opencv2/opencv.hpp>

#include "htmap/util/Image.h"

namespace htmap
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
