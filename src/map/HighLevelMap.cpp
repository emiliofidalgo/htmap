#include "hamap/map/HighLevelMap.h"

namespace hamap
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
