#include "helpers.h"
#include "map.h"


void Polygon::calculateCenter()
{
    float total_x = 0;
    float total_y = 0;
    int s = verteces.size();
    for (int i = 0; i < s; i++)
    {
        total_x += verteces[i].x;
        total_y += verteces[i].y;
    }
    center.x = total_x / float(s);
    center.y = total_y / float(s);

    radius = (center - verteces[0]).norm();

};

void Polygon::expandShape(float size)
{
    int s = verteces.size();
    for (int i = 0; i < s; i++)
    {
        // get vector connecting center to vertex
        point2d vec = verteces[i] - center;

        float th = atan2(vec.y,vec.x);

        point2d size_vec;
        size_vec.x = size * cos(th);
        size_vec.y = size * sin(th);
        // increase length by size
        vec = vec + size_vec;

        // swap vertex position for new expanded one
        verteces[i] = vec;
    };
};

void Map::addObstacle(Polygon shape)
{
    obstacles.push_back(shape);
};
