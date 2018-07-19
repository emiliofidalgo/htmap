#include "hamap/imgdesc/KeypointDetector.h"

namespace hamap
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

cv::FeatureDetector* convertToGridDetector(const int grid_rows, const int grid_cols, const int max_feats, cv::Ptr<cv::FeatureDetector>& det)
{
	cv::GridAdaptedFeatureDetector* gridadapted = new cv::GridAdaptedFeatureDetector(det, max_feats, grid_rows, grid_cols);
	return gridadapted;
}

}
