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

#include "htmap/bayes/BayesFilter.h"

namespace htmap
{

BayesFilter::BayesFilter(const BayesFilterParams params) :
    _total_elements(0),
    _trans_model(params.trans_model),
    _min_hyp(params.min_hyp),
    _elements(0)
{
    _probability.reserve(150000);
    _st = Statistics::getInstance();
    _elements = new std::vector<int>;
    _params = Params::getInstance();
}

BayesFilter::~BayesFilter()
{
}

void BayesFilter::addElement(const int elem_id)
{
    _elements->push_back(elem_id);
    _element_to_index[elem_id] = _total_elements;
    _index_to_element[_total_elements] = elem_id;
    _total_elements++;
    if (_probability.size())
    {
        _probability.push_back(0.0);
    }
    else
    {
        _probability.push_back(1.0);
    }
}

std::vector<int>* BayesFilter::getElements()
{
    return _elements;
}

void BayesFilter::predict(const unsigned curr_img, std::vector<double>* prior)
{
    if (_trans_model == BAYES_TRANS_GAUSS && _total_elements > 8)
    {
        prior->at(0) += _probability[0] * 0.2;
        prior->at(1) += _probability[0] * 0.2;
        prior->at(2) += _probability[0] * 0.2;
        prior->at(3) += _probability[0] * 0.2;
        prior->at(4) += _probability[0] * 0.2;

        prior->at(0) += _probability[1] * 0.165;
        prior->at(1) += _probability[1] * 0.165;
        prior->at(2) += _probability[1] * 0.165;
        prior->at(3) += _probability[1] * 0.165;
        prior->at(4) += _probability[1] * 0.165;
        prior->at(5) += _probability[1] * 0.165;

        prior->at(0) += _probability[2] * 0.143;
        prior->at(1) += _probability[2] * 0.143;
        prior->at(2) += _probability[2] * 0.143;
        prior->at(3) += _probability[2] * 0.143;
        prior->at(4) += _probability[2] * 0.143;
        prior->at(5) += _probability[2] * 0.143;
        prior->at(6) += _probability[2] * 0.143;

        prior->at(0) += _probability[3] * 0.125;
        prior->at(1) += _probability[3] * 0.125;
        prior->at(2) += _probability[3] * 0.125;
        prior->at(3) += _probability[3] * 0.125;
        prior->at(4) += _probability[3] * 0.125;
        prior->at(5) += _probability[3] * 0.125;
        prior->at(6) += _probability[3] * 0.125;
        prior->at(7) += _probability[3] * 0.125;

        for (size_t loc_id = 4; loc_id < _total_elements - 4; loc_id++)
        {
            if (_total_elements > 9)
            {
                prior->at(loc_id - 4) += _probability[loc_id] * 0.01;
                prior->at(loc_id - 3) += _probability[loc_id] * 0.01;
                prior->at(loc_id - 2) += _probability[loc_id] * 0.06;
                prior->at(loc_id - 1) += _probability[loc_id] * 0.19;
                prior->at(loc_id)     += _probability[loc_id] * 0.36;
                prior->at(loc_id + 1) += _probability[loc_id] * 0.19;
                prior->at(loc_id + 2) += _probability[loc_id] * 0.06;
                prior->at(loc_id + 3) += _probability[loc_id] * 0.01;
                prior->at(loc_id + 4) += _probability[loc_id] * 0.01;
            }
            else
            {
                prior->at(loc_id - 4) += _probability[loc_id] * 0.02;
                prior->at(loc_id - 3) += _probability[loc_id] * 0.02;
                prior->at(loc_id - 2) += _probability[loc_id] * 0.07;
                prior->at(loc_id - 1) += _probability[loc_id] * 0.2;
                prior->at(loc_id)     += _probability[loc_id] * 0.38;
                prior->at(loc_id + 1) += _probability[loc_id] * 0.2;
                prior->at(loc_id + 2) += _probability[loc_id] * 0.07;
                prior->at(loc_id + 3) += _probability[loc_id] * 0.02;
                prior->at(loc_id + 4) += _probability[loc_id] * 0.02;
            }
        }

        prior->at(_total_elements - 1) += _probability[_total_elements - 4] * 0.125;
        prior->at(_total_elements - 2) += _probability[_total_elements - 4] * 0.125;
        prior->at(_total_elements - 3) += _probability[_total_elements - 4] * 0.125;
        prior->at(_total_elements - 4) += _probability[_total_elements - 4] * 0.125;
        prior->at(_total_elements - 5) += _probability[_total_elements - 4] * 0.125;
        prior->at(_total_elements - 6) += _probability[_total_elements - 4] * 0.125;
        prior->at(_total_elements - 7) += _probability[_total_elements - 4] * 0.125;
        prior->at(_total_elements - 8) += _probability[_total_elements - 4] * 0.125;

        prior->at(_total_elements - 1) += _probability[_total_elements - 3] * 0.143;
        prior->at(_total_elements - 2) += _probability[_total_elements - 3] * 0.143;
        prior->at(_total_elements - 3) += _probability[_total_elements - 3] * 0.143;
        prior->at(_total_elements - 4) += _probability[_total_elements - 3] * 0.143;
        prior->at(_total_elements - 5) += _probability[_total_elements - 3] * 0.143;
        prior->at(_total_elements - 6) += _probability[_total_elements - 3] * 0.143;
        prior->at(_total_elements - 7) += _probability[_total_elements - 3] * 0.143;

        prior->at(_total_elements - 1) += _probability[_total_elements - 2] * 0.165;
        prior->at(_total_elements - 2) += _probability[_total_elements - 2] * 0.165;
        prior->at(_total_elements - 3) += _probability[_total_elements - 2] * 0.165;
        prior->at(_total_elements - 4) += _probability[_total_elements - 2] * 0.165;
        prior->at(_total_elements - 5) += _probability[_total_elements - 2] * 0.165;
        prior->at(_total_elements - 6) += _probability[_total_elements - 2] * 0.165;

        prior->at(_total_elements - 1) += _probability[_total_elements - 1] * 0.2;
        prior->at(_total_elements - 2) += _probability[_total_elements - 1] * 0.2;
        prior->at(_total_elements - 3) += _probability[_total_elements - 1] * 0.2;
        prior->at(_total_elements - 4) += _probability[_total_elements - 1] * 0.2;
        prior->at(_total_elements - 5) += _probability[_total_elements - 1] * 0.2;
    }
    else
    {
        double prob = 1.0 / _total_elements;
		for (int i = 0; i < _total_elements; i++)
		{
			prior->at(i) = prob;
		}
    }

    // Uncomment this line if you want to save the full Bayes filter info
    //    _st->registerPrior(curr_img, prior);
}

void BayesFilter::update(const unsigned curr_img, const std::map<int, double>& lk, std::vector<double>* prior)
{
    // Creating a vector for mean and stdev computation.
    std::vector<double> scores;
    std::map<int, double>::const_iterator itr;
    for(itr = lk.begin(); itr != lk.end(); itr++)
    {
        scores.push_back((*itr).second);
    }

    // Mean computation.
    double sum = std::accumulate(scores.begin(), scores.end(), 0.0);
    double mean =  sum / scores.size();

    // Stdev computation.
    double accum = 0.0;
    for (size_t score_ind = 0; score_ind < scores.size(); score_ind++)
    {
        accum += (scores[score_ind] - mean) * (scores[score_ind] - mean);
    }
    double stdev = sqrt(accum / (scores.size() - 1));

    // Setting the limit for scores.
    double limit = mean + stdev;

    std::vector<double> likelihood(_total_elements, 1.0);
    for(itr = lk.begin(); itr != lk.end(); itr++)
    {
        int index = _element_to_index[(*itr).first];
        double score = (*itr).second;

        //likelihood[index] = score;

        if (score > limit)
        {
            likelihood[index] = (score - stdev) / mean;
        }
    }

    // Uncomment this line if you want to save the full Bayes filter info
    // _st->registerLikelihood(curr_img, likelihood);

    sum = 0.0;
    for (unsigned image_id = 0; image_id < _total_elements; image_id++)
    {
        //_probability[image_id] = likelihood[image_id] * prior[image_id];
        _probability[image_id] = likelihood[image_id] * prior->at(image_id);
		prior->at(image_id) = 0.0;
        sum += _probability[image_id];
    }

    // Normalizing the final result.
    for (unsigned image_id = 0; image_id < _total_elements; image_id++)
    {
        _probability[image_id] /= sum;
    }

    // Uncomment this line if you want to save the full Bayes filter info
    // _st->registerPosterior(curr_img, _probability);
}

void BayesFilter::getMostProbablyElements(std::vector<BayesFilterResult>& elems)
{
    if (_total_elements > _min_hyp)
    {
		for (unsigned el_index = 0; el_index < _total_elements; el_index++)
		{
            BayesFilterResult elind(_index_to_element[el_index], _probability[el_index]);
			elems.push_back(elind);
		}

        std::sort(elems.begin(), elems.end());
    }
}

unsigned BayesFilter::numElems()
{
    return _total_elements;
}

}
