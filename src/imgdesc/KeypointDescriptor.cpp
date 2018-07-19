#include "hamap/imgdesc/KeypointDescriptor.h"

namespace hamap
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
