#include "helpers.h"
#include "map.h"
#include "collision_check.h"

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
void Polygon::processEdges()
{
    int s = verteces.size();
    for (int i = 0; i < s-1; i++)
    {
        line edge;
        edge.p_initial = verteces[i];
        edge.p_final = verteces[i+1];
        edges.push_back(edge);
    }
    line edge;
    edge.p_initial = verteces[s-1];
    edge.p_final = verteces[0];
}

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

bool Map::uncolliding(poin2d point)
{
    for (int i = 0; i < obstacles.size(); i++)
    {
        Polygon obs = obstacles[i];
        // rough pass - outside radius no chance of collision
        if ((point - obs.center).norm() > obs.radius) continue;

        // otherwise check for polygon collision
        if (CollisionCheck::point_in_polygon(point, obs)) return true;
    }
};
bool Map::uncolliding(arc a)
{
    // obstacle check
    for (int i = 0; i < obstacles.size(); i++)
    {
        Polygon obs = obstacles[i];
        // rough pass - if obstacle radius + arc radius  is more than distance than we're surely clear
        if ((a.radius + obs.radius) > (a.center - obs.center).norm()) continue;

        // otherwise check for polygon edge collision
        if (CollisionCheck::arc_with_polygon(a, obs)) return true;
    }
    // TODO - bounds check
};
bool Map::uncolliding(line l)
{
    
    for (int i = 0; i < obstacles.size(); i++)
    {
        Polygon obs = obstacles[i];
        // rough pass with radius of obstacles
        if ((CollisionCheck::point_lineseg_dist(obs.center, l)) > obs.radius) continue;

        // second check more detailed check if rough pass not passing
        for (int j = 0; j < obs.edges.size(); j++)
        {
            if (CollisionCheck::line_line_intersect(obs.edges[j], l)) return true;
        }
        
    }
    
    // TODO - bounds check
    return false;
};
