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

#include "htmap/map/HighLevelMap.h"

namespace htmap
{

HighLevelMap::HighLevelMap() :
    active(-1)
{
}

unsigned HighLevelMap::addLocation(Location* loc)
{
    locations.push_back(loc);
    graph.addNode();
    loc->id = locations.size() - 1;

    return loc->id;
}

void HighLevelMap::addImageToLocation(const unsigned idloc, const int id, const std::string image_filename)
{
    locations[idloc]->addImage(id, image_filename);
    im2loc[id] = idloc;
}

void HighLevelMap::setActive(const unsigned& act)
{
    active = act;
}

void HighLevelMap::linkLocations(const unsigned& a, const unsigned& b, const double& weight)
{
    graph.adjlist[a].push_back(neighbor(b, weight));
    graph.adjlist[b].push_back(neighbor(a, weight));
}

unsigned HighLevelMap::numLocations()
{
    return locations.size();
}

Location* HighLevelMap::getActiveLocation()
{
    return locations[active];
}

}
