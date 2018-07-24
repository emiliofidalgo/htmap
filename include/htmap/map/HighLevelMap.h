#ifndef _HLEVELMAP_H_
#define _HLEVELMAP_H_

#include <opencv2/opencv.hpp>

#include "hamap/map/Location.h"

namespace hamap
{

const double max_weight = std::numeric_limits<double>::infinity();

struct neighbor
{
    int target;
    double weight;
    neighbor(int arg_target, double arg_weight) :
        target(arg_target), weight(arg_weight)
        {}
};

struct Graph
{
    std::vector<std::vector<neighbor> > adjlist;

    void addNode()
    {
        std::vector<neighbor> node;
        adjlist.push_back(node);
    }
};

class HighLevelMap
{
    public:
        HighLevelMap();

        unsigned addLocation(Location* loc);
        void addImageToLocation(const unsigned idloc, const int id, const std::string image_filename);
        void setActive(const unsigned& act);
        void linkLocations(const unsigned& a, const unsigned& b, const double& weight);
        unsigned numLocations();
        Location* getActiveLocation();

        std::vector<Location*> locations;
        unsigned active;
        Graph graph;
        std::map<int, unsigned> im2loc;

};

}

#endif /* _HLEVELMAP_H_ */
