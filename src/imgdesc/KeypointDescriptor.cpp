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

#include "htmap/imgdesc/KeypointDescriptor.h"

namespace htmap
{

void FREAKKeypointDescriptor::describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs)
{
	_descriptor->compute(image, kps, descs);
}

void BRISKKeypointDescriptor::describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs)
{
	_descriptor->compute(image, kps, descs);
}

void BRIEFKeypointDescriptor::describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs)
{
	_descriptor->compute(image, kps, descs);
}

void ORBKeypointDescriptor::describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs)
{
	_descriptor->compute(image, kps, descs);
}

void SIFTKeypointDescriptor::describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs)
{
	_descriptor->compute(image, kps, descs);
}

void SURFKeypointDescriptor::describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs)
{
	_descriptor->compute(image, kps, descs);
}

void LDBKeypointDescriptor::describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs)
{
    _ldb.compute(image, kps, descs);
}

KeypointDescriptor* KeypointDescriptor::create(const std::string& name, const KeypointDescriptorParams& params)
{
	KeypointDescriptor* desc = 0;

	if (name == "FREAK")
	{
		desc = new FREAKKeypointDescriptor(params);
	}
	else if (name == "BRISK")
	{
		desc = new BRISKKeypointDescriptor(params);
	}
	else if (name == "BRIEF")
	{
		desc = new BRIEFKeypointDescriptor(params);
	}
	else if (name == "ORB")
	{
		desc = new ORBKeypointDescriptor(params);
	}
	else if (name == "SIFT")
	{
		desc = new SIFTKeypointDescriptor(params);
	}
	else if (name == "SURF")
	{
		desc = new SURFKeypointDescriptor(params);
    }
    else if (name == "LDB")
    {
        desc = new LDBKeypointDescriptor(params);
    }

	return desc;
}

}
