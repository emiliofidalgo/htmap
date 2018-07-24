#ifndef _LOCATION_H_
#define _LOCATION_H_

#include <opencv2/opencv.hpp>

#include <hamap/imgdesc/GlobalDescriptor.h>
#include <hamap/util/Image.h>
#include <hamap/util/Params.h>
#include <hamap/util/Util.h>
#include "obindex/BinaryIndex.h"

namespace vi = obindex;

namespace hamap
{

struct LocationEntry
{
    LocationEntry(const unsigned& id, const std::string& image_filename) :
        filename(image_filename),
        image_id(id)
	{}

    std::string filename;
    unsigned image_id;
};

class Location
{
	public:
        Location(const int& max_images, const int& feat_per_img, const int& desc_bytes);
        ~Location();

        void addImage(const int& id, const std::string& image_filename);
        void searchImages(const cv::Mat& dscs, std::map<int, double>& image_matches, double match_ratio);
        unsigned numImages();

        std::vector<LocationEntry> images;

        // Global included in the image.
        cv::Mat gdescriptors;

        // Representative of the location.
        cv::Mat_<double> desc;

        // Inverse Covariance Matrix.
        cv::Mat_<double> invcov;

        // True if the location has been initialized.
        bool init;

        // Location identifier inside a map. It should be assigned externally.
        unsigned id;
        
		// Binary index that stores the features of the images inside this node.
        vi::BinaryIndex* _bindex;

    private:
        void initializeLocation(const int& id, const std::string& image_filename);
        void updateGlobalDescription();

        // Class for accessing parameters.
        Params* _params;
};

}

#endif /* _LOCATION_H_ */
