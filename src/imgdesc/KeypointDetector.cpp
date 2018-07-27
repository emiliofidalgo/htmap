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

#include "htmap/imgdesc/KeypointDetector.h"

namespace htmap
{

void FASTKeypointDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps)
{
	_detector->detect(image, kps);
}

void BRISKKeypointDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps)
{
	_detector->detect(image, kps);
}

void SIFTKeypointDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps)
{
	_detector->detect(image, kps);
}

void SURFKeypointDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps)
{
	_detector->detect(image, kps);
}

void ORBKeypointDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps)
{
	_detector->detect(image, kps);
}

void StarKeypointDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps)
{
	_detector->detect(image, kps);
}

KeypointDetector* KeypointDetector::create(const std::string& name, const KeypointDetectorParams& params)
{
	KeypointDetector* detector = 0;

	if (name == "FAST")
	{
		detector = new FASTKeypointDetector(params);
	}
	else if (name == "BRISK")
	{
		detector = new BRISKKeypointDetector(params);
	}
	else if (name == "SIFT")
	{
		detector = new SIFTKeypointDetector(params);
	}
	else if (name == "SURF")
	{
		detector = new SURFKeypointDetector(params);
	}
	else if (name == "ORB")
	{
		detector = new ORBKeypointDetector(params);
	}
	else if (name == "STAR")
	{
		detector = new StarKeypointDetector(params);
	}

	return detector;
}

}
