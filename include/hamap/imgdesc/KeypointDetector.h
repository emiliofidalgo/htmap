#ifndef KEYPOINTDETECTOR_H_
#define KEYPOINTDETECTOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

namespace hamap
{

enum KeypointDetectorType
{
	DETECTOR_FAST,
	DETECTOR_ORB,
	DETECTOR_BRISK,
	DETECTOR_SIFT,
	DETECTOR_SURF,
	DETECTOR_STAR
};

// ---
// General parameters for keypoint detectors.
// ---
typedef struct _KeypointDetectorParams
{
		_KeypointDetectorParams() :
			_type(DETECTOR_SIFT),
			_fast_thresh(10),
			_fast_nonmaxSup(true),
			_brisk_thresh(30),
			_brisk_octaves(3),
			_sift_nfeats(0),
			_sift_noctaves(3),
			_sift_contrastThresh(0.04),
			_sift_edgeThresh(10),
			_sift_sigma(1.6),
			_surf_hessianThresh(300.0),
			_surf_noctaves(4),
			_surf_nlayers(2),
			_surf_upright(0),
			_orb_nfeats(500),
			_orb_scalefactor(1.2),
			_orb_nlevels(8),
			_orb_patchsize(31),
			_star_maxsize(45),
			_star_responsethresh(30),
			_star_linethreshproj(10),
			_star_linethreshbin(8),
			_star_suppressnonmaxsize(5)
        {}

		// Keypoint detector params.
		KeypointDetectorType _type;

		// FAST params
		int _fast_thresh;
		bool _fast_nonmaxSup;

		// BRISK
		int _brisk_thresh;
		int _brisk_octaves;

		// SIFT
		int _sift_nfeats;
		int _sift_noctaves;
		double _sift_contrastThresh;
		double _sift_edgeThresh;
		double _sift_sigma;

		// SURF
		double _surf_hessianThresh;
		int _surf_noctaves;
		int _surf_nlayers;
		int _surf_upright;

		// ORB
		int _orb_nfeats;
		double _orb_scalefactor;
		int _orb_nlevels;
		int _orb_patchsize;

		// STAR
		int _star_maxsize;
		int _star_responsethresh;
		int _star_linethreshproj;
		int _star_linethreshbin;
		int _star_suppressnonmaxsize;
} KeypointDetectorParams;

// ---
// Abstract keypoint detector class.
// ---
class KeypointDetector
{
	public:
        KeypointDetector(const KeypointDetectorType type) : _detector(0), _type(type) {}
        virtual ~KeypointDetector() {}

		static KeypointDetector* create(const std::string& name, const KeypointDetectorParams& params);

		virtual void parseParameters(const KeypointDetectorParams& params) = 0;
		virtual void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps) = 0;

		cv::Ptr<cv::FeatureDetector> _detector;

	protected:
		KeypointDetectorType _type;
};

// ---
// FAST keypoint detector class.
// ---
class FASTKeypointDetector : public KeypointDetector
{
	public:
		FASTKeypointDetector(const KeypointDetectorParams& params) :
			KeypointDetector(DETECTOR_FAST),
			_thresh(10),
			_nonmaxSup(true)
	{
			parseParameters(params);
			_detector = cv::FeatureDetector::create("FAST");
			_detector->setInt("threshold", _thresh);
			_detector->setBool("nonmaxSuppression", _nonmaxSup);

    }

	void parseParameters(const KeypointDetectorParams& params)
	{
		_type = params._type;
		_thresh = params._fast_thresh;
		_nonmaxSup = params._fast_nonmaxSup;
    }

	void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps);

	int _thresh;
	bool _nonmaxSup;
};

// ---
// BRISK keypoint detector class.
// ---
class BRISKKeypointDetector : public KeypointDetector
{
	public:
		BRISKKeypointDetector(const KeypointDetectorParams& params) :
			KeypointDetector(DETECTOR_BRISK),
			_thresh(30),
			_octaves(3)
	{
			parseParameters(params);
			_detector = cv::FeatureDetector::create("BRISK");
			_detector->setInt("thres", _thresh);
			_detector->setInt("octaves", _octaves);
    }

	void parseParameters(const KeypointDetectorParams& params)
	{
		_type = params._type;
		_thresh = params._brisk_thresh;
		_octaves = params._brisk_octaves;
    }

	void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps);

	int _thresh;
	int _octaves;
//	cv::Ptr<cv::FeatureDetector> _detector;
};

// ---
// SIFT keypoint detector class.
// ---
class SIFTKeypointDetector : public KeypointDetector
{
	public:
		SIFTKeypointDetector(const KeypointDetectorParams& params) :
			KeypointDetector(DETECTOR_SIFT),
			_nfeats(0),
			_noctaves(3),
			_contrastThresh(0.04),
			_edgeThresh(10.0),
			_sigma(1.6)
	{
			cv::initModule_nonfree();
			parseParameters(params);
			_detector = cv::FeatureDetector::create("SIFT");
			_detector->setInt("nFeatures", _nfeats);
			_detector->setInt("nOctaveLayers", _noctaves);
			_detector->setDouble("contrastThreshold", _contrastThresh);
			_detector->setDouble("edgeThreshold", _edgeThresh);
			_detector->setDouble("sigma", _sigma);
    }

	void parseParameters(const KeypointDetectorParams& params)
	{
		_nfeats = params._sift_nfeats;
		_noctaves = params._sift_noctaves;
		_edgeThresh = params._sift_edgeThresh;
		_contrastThresh = params._sift_contrastThresh;
		_sigma = params._sift_sigma;
		_type = params._type;
    }

	void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps);

	int _nfeats;
	int _noctaves;
	double _contrastThresh;
	double _edgeThresh;
	double _sigma;
//	cv::Ptr<cv::FeatureDetector> _detector;
};

// ---
// SURF keypoint detector class.
// ---
class SURFKeypointDetector : public KeypointDetector
{
	public:
		SURFKeypointDetector(const KeypointDetectorParams& params) :
			KeypointDetector(DETECTOR_SURF),
			_hessianThresh(300.0),
			_noctaves(4),
			_nlayers(2),
			_upright(0)
	{
			cv::initModule_nonfree();
			parseParameters(params);
			_detector = cv::FeatureDetector::create("SURF");
			_detector->setDouble("hessianThreshold", _hessianThresh);
			_detector->setInt("nOctaves", _noctaves);
			_detector->setInt("nOctaveLayers", _nlayers);
			_detector->setInt("upright", _upright);
    }

	void parseParameters(const KeypointDetectorParams& params)
	{
		_hessianThresh = params._surf_hessianThresh;
		_noctaves = params._surf_noctaves;
		_nlayers = params._surf_nlayers;
		_upright = params._surf_upright;
    }

	void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps);

	double _hessianThresh;
	int _noctaves;
	int _nlayers;
	int _upright;
//	cv::Ptr<cv::FeatureDetector> _detector;
};

// ---
// ORB keypoint detector class.
// ---
class ORBKeypointDetector : public KeypointDetector
{
	public:
		ORBKeypointDetector(const KeypointDetectorParams& params) :
			KeypointDetector(DETECTOR_ORB),
			_nfeats(500),
			_scalefactor(1.2f),
			_nlevels(8),
			_patchsize(31)
	{
			parseParameters(params);
			_detector = cv::FeatureDetector::create("ORB");
			_detector->setInt("nFeatures", _nfeats);
			_detector->setDouble("scaleFactor", _scalefactor);
			_detector->setInt("nLevels", _nlevels);
			_detector->setInt("patchSize", _patchsize);
			_detector->setInt("edgeThreshold", _patchsize);
    }

	void parseParameters(const KeypointDetectorParams& params)
	{
		_nfeats = params._orb_nfeats;
		_scalefactor = params._orb_scalefactor;
		_nlevels = params._orb_nlevels;
		_patchsize = params._orb_patchsize;
    }

	void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps);

	int _nfeats;
	double _scalefactor;
	int _nlevels;
	int _patchsize;
//	cv::Ptr<cv::FeatureDetector> _detector;
};

// ---
// STAR keypoint detector class.
// ---
class StarKeypointDetector : public KeypointDetector
{
	public:
		StarKeypointDetector(const KeypointDetectorParams& params) :
			KeypointDetector(DETECTOR_STAR),
			_maxsize(45),
			_responsethresh(30),
			_linethreshproj(10),
			_linethreshbin(8),
			_suppressnonmaxsize(5)
	{
			parseParameters(params);
			_detector = cv::FeatureDetector::create("STAR");
			_detector->setInt("maxSize", _maxsize);
			_detector->setInt("responseThreshold", _responsethresh);
			_detector->setInt("lineThresholdProjected", _linethreshproj);
			_detector->setInt("lineThresholdBinarized", _linethreshbin);
			_detector->setInt("suppressNonmaxSize", _suppressnonmaxsize);
    }

	void parseParameters(const KeypointDetectorParams& params)
	{
		_maxsize = params._star_maxsize;
		_responsethresh = params._star_responsethresh;
		_linethreshproj = params._star_linethreshproj;
		_linethreshbin = params._star_linethreshbin;
		_suppressnonmaxsize = params._star_suppressnonmaxsize;
    }

	void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps);

	int _maxsize;
	int _responsethresh;
	int _linethreshproj;
	int _linethreshbin;
	int _suppressnonmaxsize;
//	cv::Ptr<cv::FeatureDetector> _detector;
};

cv::FeatureDetector* convertToGridDetector(const int grid_rows, const int grid_cols, const int max_feats, cv::Ptr<cv::FeatureDetector>& det);

}

#endif /* KEYPOINTDETECTOR_H_ */

