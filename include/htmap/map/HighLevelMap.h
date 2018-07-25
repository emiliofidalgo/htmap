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

#ifndef _HLEVELMAP_H_
#define _HLEVELMAP_H_

#include <opencv2/opencv.hpp>

#include "htmap/map/Location.h"

namespace htmap
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
