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


#include "htmap/util/Statistics.h"

namespace htmap
{

Statistics* Statistics::_instance = 0;

Statistics* Statistics::getInstance()
{
	if (_instance == 0)
	{
		_instance = new Statistics;
	}
	return _instance;
}

void Statistics::init(const int nimages)
{
    loops = cv::Mat::zeros(nimages, nimages, CV_32S);
    prior = cv::Mat::zeros(nimages, nimages, CV_64F);
    likelihood = cv::Mat::zeros(nimages, nimages, CV_64F);
    posterior = cv::Mat::zeros(nimages, nimages, CV_64F);
    locations.resize(nimages, 0);
}

void Statistics::registerLoop(const int a, const int b)
{
    loops(a, b) = 1;
}

void Statistics::registerPrior(const int imid, const std::vector<double>& _prior)
{
    for (int i = 0; i < _prior.size(); i++)
    {
        prior(imid, i) = _prior[i];
    }
}

void Statistics::registerLikelihood(const int imid, const std::vector<double>& _lik)
{
    for (int i = 0; i < _lik.size(); i++)
    {
        likelihood(imid, i) = _lik[i];
    }
}

void Statistics::registerPosterior(const int imid, const std::vector<double>& _post)
{
    for (int i = 0; i < _post.size(); i++)
    {
        posterior(imid, i) = _post[i];
    }
}

void Statistics::registerImageToLocation(const int imid, const unsigned loc)
{
    locations[imid] = loc;
}

void Statistics::registerDescTimes(double gdesc, double det, double desc)
{
    boost::mutex::scoped_lock lock(mlock);

    std::vector<double> times;
    times.push_back(gdesc);
    times.push_back(det);
    times.push_back(desc);

    desc_times.push_back(times);
}

void Statistics::registerLCTimes(std::vector<double>& values)
{
    lc_times.push_back(values);
}

void Statistics::writeResults(const std::string& dir, int inliers)
{
    std::string loops_results_filename = dir + "htmap_loops_" + SSTR(inliers) + ".txt";
    std::string prior_results_filename = dir + "htmap_prior_" + SSTR(inliers) + ".txt";
    std::string likelihood_results_filename = dir + "htmap_likelihood_" + SSTR(inliers) + ".txt";
    std::string posterior_results_filename = dir + "htmap_posterior_" + SSTR(inliers) + ".txt";
    std::string img2loc_results_filename = dir + "htmap_img2loc_" + SSTR(inliers) + ".txt";

    std::ofstream loops_file, prior_file, lik_file, post_file, im2loc_file;

    loops_file.open(loops_results_filename.c_str(), std::ios::out | std::ios::trunc);
    prior_file.open(prior_results_filename.c_str(), std::ios::out | std::ios::trunc);
    lik_file.open(likelihood_results_filename.c_str(), std::ios::out | std::ios::trunc);
    post_file.open(posterior_results_filename.c_str(), std::ios::out | std::ios::trunc);
    im2loc_file.open(img2loc_results_filename.c_str(), std::ios::out | std::ios::trunc);

    for (int i = 0; i < loops.rows; i++)
    {
        for (int j = 0; j < loops.cols; j++)
        {
            loops_file << loops(i, j) << "\t";
            // Uncomment these lines if you want to save the full Bayes filter info

            //prior_file << prior(i, j) << "\t";
            //lik_file << likelihood(i, j) << "\t";
            //post_file << posterior(i, j) << "\t";
		}
        im2loc_file << i << "\t" << locations[i] << std::endl;

		loops_file << std::endl;
        // Uncomment these lines if you want to save the full Bayes filter info

        //prior_file << std::endl;
        //lik_file << std::endl;
        //post_file << std::endl;
    }

	loops_file.close();
    prior_file.close();
    lik_file.close();
    post_file.close();
    im2loc_file.close();

    // Description times
    std::string dtimes_results_filename = dir + "htmap_dtimes_" + SSTR(inliers) + ".txt";
    std::ofstream dtimes_file;
    dtimes_file.open(dtimes_results_filename.c_str(), std::ios::out | std::ios::trunc);
    for (unsigned i = 0; i < desc_times.size(); i++)
    {
        for (unsigned j = 0; j < desc_times[i].size(); j++)
        {
            dtimes_file << desc_times[i][j] << "\t";
        }

        dtimes_file << std::endl;
    }

    dtimes_file.close();

    // LC times
    std::string lc_results_filename = dir + "htmap_lc_" + SSTR(inliers) + ".txt";
    std::ofstream lc_file;
    lc_file.open(lc_results_filename.c_str(), std::ios::out | std::ios::trunc);
    for (unsigned i = 0; i < lc_times.size(); i++)
    {
        for (unsigned j = 0; j < lc_times[i].size(); j++)
        {
            lc_file << lc_times[i][j] << "\t";
        }

        lc_file << std::endl;
    }

    lc_file.close();
}

}
