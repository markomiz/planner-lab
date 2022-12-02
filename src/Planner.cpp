
#include "dubin.h"

using namespace std;

std::random_device rd;
std::mt19937 mt(rd());
std::uniform_real_distribution<double> dist(0.0, 1.0);


// Uniform non colliding sample
point2d Planner::uniform_point(float min_x,float max_x, float min_y, float max_y);
{
    point2d p;
    p.x = dist(mt) * (max_x - min_x) + min_x;
    p.y = dist(mt)* (max_y - min_y) + min_y;;

    return p;
};


