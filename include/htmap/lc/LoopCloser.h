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

#ifndef _LOOPCLOSER_H_
#define _LOOPCLOSER_H_

#include <queue>
#include <omp.h>

#include <opencv2/opencv.hpp>

#include "htmap/bayes/BayesFilter.h"
#include "htmap/map/HighLevelMap.h"
#include "htmap/util/Image.h"
#include "htmap/util/Params.h"

namespace htmap
{

class LoopCloser
{
    public:
        LoopCloser();
        ~LoopCloser();

        bool process(const Image& image, HighLevelMap& hmap, unsigned& loop_loc, unsigned& loop_img);

    private:
        BayesFilter _filter;
        Params* _params;
        Statistics* _st;
        std::queue<int> _buffer;
		std::vector<double> _prior;

        void computeLikelihood(const Image& image, HighLevelMap& hmap, std::map<int, double>& lik, double& llc_time, double& ilc_time);
};

}

#endif /* _LOOPCLOSER_H_ */
