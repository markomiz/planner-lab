
#include "dubin.h"

using namespace std;

std::random_device rd;
std::mt19937 mt(rd());
std::uniform_real_distribution<double> dist(0.0, 1.0);


// Uniform non colliding sample
point2d Planner::uniform_point()
{
    point2d p;
    p.x = dist(mt) * (map->max_x - map->min_x) + map->min_x;
    p.y = dist(mt)* (map->max_y - map->min_y) + map->min_y;

    return p;
};

