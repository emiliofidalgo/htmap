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

#include "htmap/util/Util.h"

namespace htmap
{

static const uchar popCountTable[] =
{
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

void ratioMatching(const Image& query, const Image& train, const double& ratio, std::vector<cv::DMatch>& matches)
{
    matches.clear();
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    // Matching descriptors.
    std::vector<std::vector<cv::DMatch> > matches12;
    matcher.knnMatch(query.dscs, train.dscs, matches12, 2);

    // Filtering the resulting matchings according to the given ratio.
    for (unsigned m = 0; m < matches12.size(); m++)
    {
        if (matches12[m][0].distance <= matches12[m][1].distance * ratio)
        {
            matches.push_back(matches12[m][0]);
        }
    }
}

int checkEpipolarGeometry(const Image& query, const Image& train, const std::vector<cv::DMatch>& matches)
{
    // Convert matchings into Point2f.
    std::vector<cv::Point2f> points1, points2;
    for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); it++)
    {
        // Get the position of query keypoints
        float x = query.kps[it->queryIdx].pt.x;
        float y = query.kps[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x,y));

        // Get the position of train keypoints
        x = train.kps[it->trainIdx].pt.x;
        y = train.kps[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x,y));
    }

    // Computing the fundamental matrix.
    std::vector<uchar> inliers(points1.size(), 0);
    if (points1.size() > 7)
    {
        cv::findFundamentalMat(
                    cv::Mat(points1), cv::Mat(points2), // Matching points
                    CV_FM_RANSAC, 						// RANSAC method
                    3.0,								// Distance to epipolar line
                    0.985,  							// Confidence probability
                    inliers);							// Match status (inlier or outlier)
    }

    // Extract the surviving (inliers) matches
    std::vector<uchar>::const_iterator it = inliers.begin();
    int total_inliers = 0;
    for (; it != inliers.end(); ++it)
    {
        if (*it) total_inliers++;
    }

    return total_inliers;
}

double distEuclidean(const cv::Mat& a, const cv::Mat& b)
{
    return cv::norm(a, b, cv::NORM_L2);
}

double distMahalanobis(const cv::Mat& a, const cv::Mat& b, const cv::Mat& icovar)
{
    return cv::Mahalanobis(a, b, icovar);
}

int distHamming(const uchar* a, const uchar* b, const int n)
{
    int i = 0;
    int result = 0;

    for( ; i <= n - 4; i += 4 )
    {
        result += popCountTable[a[i] ^ b[i]] +
                  popCountTable[a[i+1] ^ b[i+1]] +
                  popCountTable[a[i+2] ^ b[i+2]] +
                  popCountTable[a[i+3] ^ b[i+3]];
    }

    for( ; i < n; i++ )
    {
        result += popCountTable[a[i] ^ b[i]];
    }

    return result;
}

void getFilenames(const std::string& directory, std::vector<std::string>& filenames, bool images)
{
    using namespace boost::filesystem;

    filenames.clear();
    path dir(directory);

    // Retrieving, sorting and filtering filenames.
    std::vector<path> entries;
    copy(directory_iterator(dir), directory_iterator(), back_inserter(entries));
    sort(entries.begin(), entries.end());
    for (std::vector<path>::const_iterator it(entries.begin()); it != entries.end(); ++it)
    {
        std::string ext = it->extension().c_str();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (images)
        {
            if (ext == ".png" || ext == ".jpg" || ext == ".ppm")
            {
                filenames.push_back(it->string());
            }
        }
        else
        {
            if (ext == ".yml")
            {
                filenames.push_back(it->string());
            }
        }
    }
}

double distChiSquare(const cv::Mat& a, const cv::Mat& b)
{
    int sz = a.cols;
    cv::Mat_<double> chsvals = cv::Mat::zeros(1, sz, CV_64F);

    #pragma omp parallel for
    for (int i = 0; i < sz; i++)
    {
        double hi = a.at<double>(0, i);
        double hj = b.at<double>(0, i);
        chsvals(0, i) = ((hi - hj) * (hi - hj)) / (hi + hj);
    }

    cv::Mat sumMat;
    cv::reduce(chsvals, sumMat, 1, CV_REDUCE_SUM);
    return sumMat.at<double>(0, 0);
}

double distBhattacharyya(const cv::Mat& a, const cv::Mat& b)
{
    int sz = a.cols;
    cv::Mat_<double> vals = cv::Mat::zeros(1, sz, CV_64F);

    #pragma omp parallel for
    for (int i = 0; i < sz; i++)
    {
        double hi = a.at<double>(0, i);
        double hj = b.at<double>(0, i);
        vals(0, i) = sqrt(hi * hj);
	}

    cv::Mat sumMat;
    cv::reduce(vals, sumMat, 1, CV_REDUCE_SUM);
    return sqrt(1.0 - sumMat.at<double>(0, 0));
}

}
