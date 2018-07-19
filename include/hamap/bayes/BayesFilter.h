#ifndef _BAYESFILTER_H_
#define _BAYESFILTER_H_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <numeric>
#include <vector>

#include "hamap/util/Params.h"
#include "hamap/util/Statistics.h"

namespace hamap
{

enum BayesTransitionModel
{
    BAYES_TRANS_GAUSS,
    BAYES_TRANS_UNIFORM
};

struct BayesFilterParams
{
    BayesFilterParams() :
        trans_model(BAYES_TRANS_GAUSS),
        min_hyp(20)
    {}

    BayesTransitionModel trans_model;
    int min_hyp;
};

struct BayesFilterResult
{
    BayesFilterResult(const int id, const double p) :
        elem_id(id),
        score(p)
    {
    }

    bool operator<(const BayesFilterResult& other) const { return score > other.score; }

    int elem_id;
    double score;
};


class BayesFilter
{
public:
    BayesFilter(const BayesFilterParams params = BayesFilterParams());
    virtual ~BayesFilter();

    void addElement(const int elem_id);
    std::vector<int>* getElements();
//    void predict(const unsigned curr_img, std::vector<double>& prior);
    void predict(const unsigned curr_img, std::vector<double>* prior);
    void update(const unsigned curr_img, const std::map<int, double>& lk, std::vector<double>* prior);
    void getMostProbablyElements(std::vector<BayesFilterResult>& elems);
    unsigned numElems();

protected:
    unsigned _total_elements;
    BayesTransitionModel _trans_model;
    unsigned _min_hyp;
    std::vector<double> _probability;
    std::map<int, int> _index_to_element;
    std::map<int, int> _element_to_index;
    std::vector<int>* _elements;
    Statistics* _st;
    Params* _params;
};

}

#endif /* BAYESFILTER_H_ */
