#ifndef _LOOPCLOSER_H_
#define _LOOPCLOSER_H_

#include <queue>
#include <omp.h>

#include <opencv2/opencv.hpp>

#include "hamap/bayes/BayesFilter.h"
#include "hamap/map/HighLevelMap.h"
#include "hamap/util/Image.h"
#include "hamap/util/Params.h"

namespace hamap
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
