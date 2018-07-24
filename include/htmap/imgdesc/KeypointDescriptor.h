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

#ifndef KEYPOINTDESCRIPTOR_H_
#define KEYPOINTDESCRIPTOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "ldb.h"

namespace htmap
{

enum KeypointDescriptorType
{
	DESCRIPTOR_BRIEF,
	DESCRIPTOR_ORB,
	DESCRIPTOR_BRISK,
	DESCRIPTOR_FREAK,
	DESCRIPTOR_SIFT,
  DESCRIPTOR_SURF,
  DESCRIPTOR_LDB
};

// ---
// General parameters for keypoint descriptors.
// ---
typedef struct _KeypointDescriptorParams
{
		_KeypointDescriptorParams() :
			_type(DESCRIPTOR_FREAK),
			_freak_ori_norm(true),
			_freak_scale_norm(true),
			_freak_pattern_scale(22.0f),
			_freak_octaves(4),
			_brisk_thresh(30),
			_brisk_octaves(3),
			_surf_extended(1)
        {}

		// Keypoint descriptor params.
		KeypointDescriptorType _type;

		// FREAK params
		bool _freak_ori_norm;
		bool _freak_scale_norm;
		float _freak_pattern_scale;
		int _freak_octaves;

		// BRISK params
		int _brisk_thresh;
		int _brisk_octaves;

		// BRIEF params

		// ORB params

    // SIFT params

    // SURF params
    int _surf_extended;

    // LDB params

} KeypointDescriptorParams;

// ---
// Abstract keypoint descriptor class.
// ---
class KeypointDescriptor
{
	public:
		KeypointDescriptor(const KeypointDescriptorType& type, const int nbytes) :
			_type(type),
			_desc_size(nbytes) {}
		virtual ~KeypointDescriptor() {}

    inline int getDescSize() { return _desc_size; }
    inline KeypointDescriptorType getType() { return _type; }

		static KeypointDescriptor* create(const std::string& name, const KeypointDescriptorParams& params);

		virtual void parseParameters(const KeypointDescriptorParams& params) = 0;
		virtual void describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs) = 0;

	protected:
		KeypointDescriptorType _type;
		int _desc_size;
};

// ---
// FREAK keypoint descriptor class.
// ---
class FREAKKeypointDescriptor : public KeypointDescriptor
{
	public:
		FREAKKeypointDescriptor(const KeypointDescriptorParams& params) :
			KeypointDescriptor(DESCRIPTOR_FREAK, 64),
			_ori_norm(true),
			_scale_norm(true),
			_pattern_scale(22.0f),
			_octaves(4)
		{
			parseParameters(params);
			_descriptor = cv::xfeatures2d::FREAK::create(_ori_norm,
																			_scale_norm,
																			_pattern_scale,
																			_octaves);
	  }

		void parseParameters(const KeypointDescriptorParams& params)
		{
			_type = params._type;
			_ori_norm = params._freak_ori_norm;
			_scale_norm = params._freak_scale_norm;
			_pattern_scale = params._freak_pattern_scale;
			_octaves = params._freak_octaves;
		}

		void describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

	protected:
		bool _ori_norm;
		bool _scale_norm;
		float _pattern_scale;
		int _octaves;
		cv::Ptr<cv::Feature2D> _descriptor;
};

// ---
// BRISK keypoint descriptor class.
// ---
class BRISKKeypointDescriptor : public KeypointDescriptor
{
	public:
		BRISKKeypointDescriptor(const KeypointDescriptorParams& params) :
			KeypointDescriptor(DESCRIPTOR_BRISK, 64),
			_thresh(30),
			_octaves(3)
		{
			parseParameters(params);
			_descriptor = cv::BRISK::create(_thresh, _octaves);
    }

		void parseParameters(const KeypointDescriptorParams& params)
		{
			_type = params._type;
			_thresh = params._brisk_thresh;
			_octaves = params._brisk_octaves;
    }

		void describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

	protected:
		int _thresh;
		int _octaves;
		cv::Ptr<cv::Feature2D> _descriptor;
};

// ---
// BRIEF keypoint descriptor class.
// ---
class BRIEFKeypointDescriptor : public KeypointDescriptor
{
	public:
		BRIEFKeypointDescriptor(const KeypointDescriptorParams& params) :
			KeypointDescriptor(DESCRIPTOR_BRIEF, 32)
		{
			parseParameters(params);
			_descriptor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }

		void parseParameters(const KeypointDescriptorParams& params)
		{
			_type = params._type;
    }

		void describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

	protected:
		cv::Ptr<cv::Feature2D> _descriptor;
};

// ---
// ORB keypoint descriptor class.
// ---
class ORBKeypointDescriptor : public KeypointDescriptor
{
	public:
		ORBKeypointDescriptor(const KeypointDescriptorParams& params) :
			KeypointDescriptor(DESCRIPTOR_ORB, 32)
		{
			parseParameters(params);
			_descriptor = cv::ORB::create();
    }

		void parseParameters(const KeypointDescriptorParams& params)
		{
		_type = params._type;
    }

		void describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

	protected:
		cv::Ptr<cv::Feature2D> _descriptor;
};

// ---
// SIFT keypoint descriptor class.
// ---
class SIFTKeypointDescriptor : public KeypointDescriptor
{
	public:
		SIFTKeypointDescriptor(const KeypointDescriptorParams& params) :
			KeypointDescriptor(DESCRIPTOR_SIFT, 128)
		{
			parseParameters(params);
			_descriptor = cv::xfeatures2d::SIFT::create();
    }

		void parseParameters(const KeypointDescriptorParams& params)
		{
			_type = params._type;
    }

		void describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

	protected:
		cv::Ptr<cv::Feature2D> _descriptor;
};

// ---
// SURF keypoint descriptor class.
// ---
class SURFKeypointDescriptor : public KeypointDescriptor
{
	public:
		SURFKeypointDescriptor(const KeypointDescriptorParams& params) :
			KeypointDescriptor(DESCRIPTOR_SURF, 128),
			_extended(1)
		{
			parseParameters(params);
			if (!_extended)
			{
				_desc_size = 64;
			}

			_descriptor = cv::xfeatures2d::SURF::create(100, 4, 3, _extended);
    }

		void parseParameters(const KeypointDescriptorParams& params)
		{
			_type = params._type;
			_extended = params._surf_extended;
    }

		void describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

	protected:
		int _extended;
		cv::Ptr<cv::Feature2D> _descriptor;
};

// ---
// LDB keypoint descriptor class.
// ---
class LDBKeypointDescriptor : public KeypointDescriptor
{
    public:
      LDBKeypointDescriptor(const KeypointDescriptorParams& params) :
            KeypointDescriptor(DESCRIPTOR_LDB, 32)
    	{
            parseParameters(params);
    	}

    	void parseParameters(const KeypointDescriptorParams& params)
    	{
        _type = params._type;
    	}

    	void describe(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

    protected:
        LdbDescriptorExtractor _ldb;
};

}

#endif /* KEYPOINTDESCRIPTOR_H_ */
