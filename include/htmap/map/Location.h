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

#ifndef _LOCATION_H_
#define _LOCATION_H_

#include <opencv2/opencv.hpp>

#include "htmap/imgdesc/GlobalDescriptor.h"
#include "htmap/util/Image.h"
#include "htmap/util/Params.h"
#include "htmap/util/Util.h"
#include "obindex/BinaryIndex.h"

namespace vi = obindex;

namespace htmap
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
