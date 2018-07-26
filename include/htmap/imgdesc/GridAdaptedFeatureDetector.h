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

#ifndef GAFEATURE_DETECTOR_H_
#define GAFEATURE_DETECTOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace htmap {

class CV_EXPORTS_W GridAdaptedFeatureDetector : public cv::Feature2D
{
public:
    /*
     * detector            Detector that will be adapted.
     * maxTotalKeypoints   Maximum count of keypoints detected on the image. Only the strongest keypoints
     *                      will be keeped.
     * gridRows            Grid rows count.
     * gridCols            Grid column count.
     */
    CV_WRAP GridAdaptedFeatureDetector( const cv::Ptr<cv::Feature2D>& detector,
                                        int maxTotalKeypoints=1000,
                                        int gridRows=4, int gridCols=4 );

    // TODO implement read/write
    virtual bool empty() const;
    cv::Algorithm* info() const;


    void detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;

protected:
    cv::Ptr<cv::Feature2D> detector;
    int maxTotalKeypoints;
    int gridRows;
    int gridCols;
};

}  // namespace HTMap

#endif  // GAFEATURE_DETECTOR_H_