/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// This code was taken from the original OpenCV 2.4 library and adapted to be
// used in HTMap. This functionality was deleted in OpenCV 3 and it was required
// to work with this software.

#include "htmap/imgdesc/GridAdaptedFeatureDetector.h"

namespace htmap {

/*
 *  GridAdaptedFeatureDetector
 */
GridAdaptedFeatureDetector::GridAdaptedFeatureDetector( const cv::Ptr<cv::Feature2D>& _detector,
                                                        int _maxTotalKeypoints, int _gridRows, int _gridCols )
    : detector(_detector), maxTotalKeypoints(_maxTotalKeypoints), gridRows(_gridRows), gridCols(_gridCols)
{}

bool GridAdaptedFeatureDetector::empty() const
{
    return detector.empty() || (cv::Feature2D*)detector->empty();
}

struct ResponseComparator
{
    bool operator() (const cv::KeyPoint& a, const cv::KeyPoint& b)
    {
        return std::abs(a.response) > std::abs(b.response);
    }
};

static void keepStrongest(int N, std::vector<cv::KeyPoint>& keypoints )
{
    if( (int)keypoints.size() > N )
    {
        std::vector<cv::KeyPoint>::iterator nth = keypoints.begin() + N;
        std::nth_element( keypoints.begin(), nth, keypoints.end(), ResponseComparator() );
        keypoints.erase( nth, keypoints.end() );
    }
}

namespace {
class GridAdaptedFeatureDetectorInvoker : public cv::ParallelLoopBody
{
private:
    int gridRows_, gridCols_;
    int maxPerCell_;
    std::vector<cv::KeyPoint>& keypoints_;
    const cv::Mat& image_;
    const cv::Mat& mask_;
    const cv::Ptr<cv::Feature2D>& detector_;
    cv::Mutex* kptLock_;

    GridAdaptedFeatureDetectorInvoker& operator=(const GridAdaptedFeatureDetectorInvoker&); // to quiet MSVC

public:

    GridAdaptedFeatureDetectorInvoker(const cv::Ptr<cv::Feature2D>& detector, const cv::Mat& image, const cv::Mat& mask,
                                      std::vector<cv::KeyPoint>& keypoints, int maxPerCell, int gridRows, int gridCols,
                                      cv::Mutex* kptLock)
        : gridRows_(gridRows), gridCols_(gridCols), maxPerCell_(maxPerCell),
          keypoints_(keypoints), image_(image), mask_(mask), detector_(detector),
          kptLock_(kptLock)
    {
    }

    void operator() (const cv::Range& range) const
    {
        for (int i = range.start; i < range.end; ++i)
        {
            int celly = i / gridCols_;
            int cellx = i - celly * gridCols_;

            cv::Range row_range((celly*image_.rows)/gridRows_, ((celly+1)*image_.rows)/gridRows_);
            cv::Range col_range((cellx*image_.cols)/gridCols_, ((cellx+1)*image_.cols)/gridCols_);

            cv::Mat sub_image = image_(row_range, col_range);
            cv::Mat sub_mask;
            if (!mask_.empty()) sub_mask = mask_(row_range, col_range);

            std::vector<cv::KeyPoint> sub_keypoints;
            sub_keypoints.reserve(maxPerCell_);

            detector_->detect( sub_image, sub_keypoints, sub_mask );
            keepStrongest( maxPerCell_, sub_keypoints );

            std::vector<cv::KeyPoint>::iterator it = sub_keypoints.begin(),
                                                end = sub_keypoints.end();
            for( ; it != end; ++it )
            {
                it->pt.x += col_range.start;
                it->pt.y += row_range.start;
            }

            cv::AutoLock join_keypoints(*kptLock_);
            keypoints_.insert( keypoints_.end(), sub_keypoints.begin(), sub_keypoints.end() );
        }
    }
};
} // namepace

void GridAdaptedFeatureDetector::detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask ) const
{
    if (image.empty() || maxTotalKeypoints < gridRows * gridCols)
    {
        keypoints.clear();
        return;
    }
    keypoints.reserve(maxTotalKeypoints);
    int maxPerCell = maxTotalKeypoints / (gridRows * gridCols);

    cv::Mutex kptLock;
    cv::parallel_for_(cv::Range(0, gridRows * gridCols),
        GridAdaptedFeatureDetectorInvoker(detector, image, mask, keypoints, maxPerCell, gridRows, gridCols, &kptLock));
}

}  // htmap