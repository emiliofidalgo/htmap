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

#ifndef _STATISTICS_H
#define _STATISTICS_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

namespace htmap
{

class Statistics
{
    public:
        // Public members.
        cv::Mat_<int> loops;
        cv::Mat_<double> prior;
        cv::Mat_<double> likelihood;
        cv::Mat_<double> posterior;
        std::vector<unsigned> locations;
        std::vector<std::vector<double> > desc_times;
        std::vector<std::vector<double> > lc_times;

        // Public functions.
        static Statistics* getInstance();
        void init(const int nimages);
        void registerLoop(const int a, const int b);
        void registerPrior(const int imid, const std::vector<double>& _prior);
        void registerLikelihood(const int imid, const std::vector<double>& _lik);
        void registerPosterior(const int imid, const std::vector<double>& _post);
        void registerImageToLocation(const int imid, const unsigned loc);
        void registerDescTimes(double gdesc, double det, double desc);
        void registerLCTimes(std::vector<double>& values);
        void writeResults(const std::string& dir, int inliers);

    protected:
        // Protected constructor. Singleton class.
        Statistics()
        {
        }

        ~Statistics()
        { delete _instance; }

        Statistics(const Statistics &);
        Statistics& operator=(const Statistics &);

    private:
        // Single instance.
        static Statistics* _instance;

        // Mutex for description times
        boost::mutex mlock;
};

}

#endif /* _STATISTICS_H */
