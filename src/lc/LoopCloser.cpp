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

#include "htmap/lc/LoopCloser.h"

namespace htmap
{

LoopCloser::LoopCloser()
{
    _params = Params::getInstance();
    _st = Statistics::getInstance();
	_prior.resize(_params->nimages, 0.0);
}

LoopCloser::~LoopCloser()
{
}

bool LoopCloser::process(const Image& image, HighLevelMap& hmap, unsigned& loop_loc, unsigned& loop_img)
{
    loop_loc = 0;
    loop_img = 0;
    bool response = false;

    std::vector<double> lc_times;
    double init_time, end_time;

    // Inserting new hypotheses in the filter.
    if (_buffer.size() == _params->imageLC_disc_recent)
    {
        _filter.addElement(_buffer.front());
        _buffer.pop();
    }
    _buffer.push(int(image.image_id));

    // Executing the Bayes filter.
    if (_filter.numElems() > 0)
    {
        // Predict
//        std::vector<double> prior;
        init_time = omp_get_wtime();
        _filter.predict(image.image_id, &_prior);
        end_time = omp_get_wtime();
        lc_times.push_back(end_time - init_time);

        // Computing the likelihood (LLC and ILC).
        std::map<int, double> lk;
        double llct, ilct;
        computeLikelihood(image, hmap, lk, llct, ilct);

        // Update the filter.
        init_time = omp_get_wtime();
        _filter.update(image.image_id, lk, &_prior);
        end_time = omp_get_wtime();
        lc_times.push_back(end_time - init_time);

        // Adding likelihood times.
        lc_times.push_back(llct);
        lc_times.push_back(ilct);

        // Obtaining best candidate for loop closing.
        std::vector<BayesFilterResult> results;
        _filter.getMostProbablyElements(results);
        if (results.size())
        {
            BayesFilterResult bcand = results[0];
            //if (bcand.score > _params->imageLC_tloop)
            //{
                // Loading the previous image.
                Image prev_img;
                prev_img.load(_params->images[bcand.elem_id]);

                // Matching images and filtering them using NNDR.
                init_time = omp_get_wtime();
                std::vector<cv::DMatch> feat_matches;
                ratioMatching(image, prev_img, _params->match_ratio, feat_matches);

                // Performing the epipolar check.
                int num_inliers = checkEpipolarGeometry(image, prev_img, feat_matches);
                end_time = omp_get_wtime();
                lc_times.push_back(end_time - init_time);

                ROS_INFO("Image Candidate: %i, Score %f, Matchings %lu, Inliers %i", bcand.elem_id, bcand.score, feat_matches.size(), num_inliers);

                // Taking a decision about the loop closing.
                if (num_inliers > _params->imageLC_min_inliers)
                {
                    loop_img = unsigned(bcand.elem_id);
                    loop_loc = hmap.im2loc[bcand.elem_id];
                    response = true;
                }
            //}
        }
    }

    _st->registerLCTimes(lc_times);

    return response;
}

void LoopCloser::computeLikelihood(const Image& image, HighLevelMap& hmap, std::map<int, double>& lik, double& llc_time, double& ilc_time)
{
    lik.clear();

    double init_time = omp_get_wtime();

    // --- LOCATION LOOP CLOSURE (LLC) ---
    // -----------------------------------

    // Vector to store the distances to the locations.
    cv::Mat_<double> loc_dists = cv::Mat::zeros(1, hmap.numLocations(), CV_64F);
    // Vector to store if the correspondent location has a score higher than T.
    std::vector<bool> loc_search(hmap.numLocations(), false);
    loc_search[hmap.active] = true; // We always search in the current node.

    // Computing the distances to all locations stored in the map.
    #pragma omp parallel for
    for (unsigned loc_idx = 0; loc_idx < hmap.numLocations(); loc_idx++)
    {
        //loc_dists(0, loc_idx) = GlobalDescriptor::dist(hmap.locations[loc_idx]->desc, image.gdsc, hmap.locations[loc_idx]->invcov);
        loc_dists(0, loc_idx) = GlobalDescriptor::dist(hmap.locations[loc_idx]->desc, image.gdsc, _params->gdescriptor);
    }

    // Computing the minimum and maximum value.
    cv::Mat_<double> minMat, maxMat;
    cv::reduce(loc_dists, minMat, 1, CV_REDUCE_MIN);
    cv::reduce(loc_dists, maxMat, 1, CV_REDUCE_MAX);
    double minv = minMat(0, 0);
    double maxv = maxMat(0, 0);

    // Converting the distances to similarities.
    if (hmap.numLocations() > 1)
    {
        #pragma omp parallel for
        for (unsigned loc_idx = 0; loc_idx < hmap.numLocations(); loc_idx++)
        {
            loc_dists(0, loc_idx) = 1.0 - ((loc_dists(0, loc_idx) - minv) / (maxv - minv));
			if (loc_dists(0, loc_idx) > _params->locLC_thresh)
			{
                loc_search[loc_idx] = true;
			}
        }
    }
    else
    {
        loc_dists(0, 0) = 1.0;
    }

    double end_time = omp_get_wtime();
    llc_time = end_time - init_time;

    init_time = omp_get_wtime();

    // --- IMAGE LOOP CLOSURE (ILC) ---
    // --------------------------------

    // Cache for searching for results in nodes.
    std::map<unsigned, std::map<int, double> > loc_imscores;

    // For each image that needs a likelihood to be updated in the filter.
    std::vector<int>* elems = _filter.getElements();
    for (unsigned elem_idx = 0; elem_idx < elems->size(); elem_idx++)
    {
        int image_id = elems->at(elem_idx);
        unsigned imloc = hmap.im2loc[image_id];

        if (loc_search[imloc])
        {
            // Higher than the threshold.
            // If we have not searched images in the node, we search and store the result.
            if (loc_imscores.count(imloc) == 0)
            {
                // Search for similar images.
                std::map<int, double> image_matches;
                hmap.locations[imloc]->searchImages(image.dscs, image_matches, _params->match_ratio);
                loc_imscores[imloc] = image_matches;
            }

            // Establishing the likelihood.
           	lik[image_id] = loc_dists(0, imloc) * loc_imscores[imloc][image_id];
//			std::cout << "Likelihood for image " << image_id << " Node " << imloc << ": "  << loc_dists(0, imloc) << " * " <<  loc_imscores[imloc][image_id] << ": " << lik[image_id] << std::endl;
        }
        else
        {
            // We do not need to search in the location.
            lik[image_id] = 0;
//			std::cout << "Likelihood for image " << image_id << " Node " << imloc << ": " << 0 << std::endl;
        }
    }

    end_time = omp_get_wtime();
    ilc_time = end_time - init_time;
}

}
