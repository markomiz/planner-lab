#include "geometry.h"
#include "map.h"
#include "collision_check.h"

void Polygon::calculateCenter()
{
    float total_x = 0;
    float total_y = 0;
    int s = verteces.size();
    for (auto i = 0; i < s; i++)
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
    for (auto i = 0; i < s-1; i++)
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
    for (auto i = 0; i < s; i++)
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
void Polygon::calculateArea()
{
    float a = edges[0].length();

    area = verteces.size() * a * a / (4 * sin(M_PI/ a));

}

void Map::addObstacle(Polygon shape)
{
    obstacles.push_back(shape);
    freeSpace -= shape.area;
};

bool Map::colliding(point2d point)
{
    for (auto i = 0; i < obstacles.size(); i++)
    {
        Polygon obs = obstacles[i];
        // rough pass - outside radius no chance of collision
        if ((point - obs.center).norm() > obs.radius) continue;

        // otherwise check for polygon collision
        if (CollisionCheck::point_in_polygon(point, obs)) return true;
    }
    if ( ! inBounds(point)) return true;
    return false;
};
bool Map::colliding(arc a)
{
    // obstacle check
    for (auto i = 0; i < obstacles.size(); i++)
    {
        Polygon obs = obstacles[i];
        // rough pass - if obstacle radius + arc radius  is more than distance than we're surely clear
        if ((a.radius + obs.radius) > (a.center - obs.center).norm()) continue;

        // otherwise check for polygon edge collision
        if (CollisionCheck::arc_with_polygon(a, obs)) return true;
    }
    //  bounds check
    for (auto i = 0; i < bounds.size(); i++)
    {
        if (CollisionCheck::line_arc_intersect(bounds[i], a).intersects) return true;
    }
};
bool Map::colliding(line l)
{
    
    for (auto i = 0; i < obstacles.size(); i++)
    {
        Polygon obs = obstacles[i];
        // rough pass with radius of obstacles
        if ((CollisionCheck::point_lineseg_dist(obs.center, l)) > obs.radius) continue;

        // second check more detailed check if rough pass not passing
        for (auto j = 0; j < obs.edges.size(); j++)
        {
            if (CollisionCheck::line_line_intersect(obs.edges[j], l).intersects)
            {
                cout << "line intersects";
                return true;
            } 
        }
        
    }
    if ( ! inBounds(l.p_final)) return true;
    if ( ! inBounds(l.p_initial)) return true;

    return false;
};


bool Map::inBounds(point2d p)
{
    if (p.x > max_x ) return false;
    if (p.y > max_y ) return false;
    if (p.x < min_x ) return false;
    if (p.y < min_y ) return false;
    return true;
};

void Map::processBounds(){

    point2d tl;
    tl.x = min_x;
    tl.y = max_y;
    point2d tr;
    tr.x = max_x;
    tr.y = max_y;
    point2d bl;
    bl.x = min_x;
    bl.y = min_y;
    point2d br;
    br.x = max_x;
    br.y = min_y;
    // left
    line l;
    l.p_initial = tl;
    l.p_final = bl;
    // right
    line r;
    r.p_initial = tr;
    r.p_final = br;
    // top
    line t;
    t.p_initial = tl;
    t.p_final = tr;
    // bottom
    line b;
    b.p_initial = br;
    b.p_final = bl;

    bounds.push_back(l);
    bounds.push_back(r);
    bounds.push_back(t);
    bounds.push_back(b);

    freeSpace = (max_x - min_x) * (max_y - min_y);

};
// RANDOM NUMBER STUFF ////
std::random_device rd;
std::mt19937 mt(rd());
std::uniform_real_distribution<double> dist(0.0, 1.0);

point2d Map::uniform_sample()
{
    point2d p;
    p.x = dist(mt) * (max_x - min_x) + min_x;
    p.y = dist(mt)* (max_y - min_y) + min_y;

    while (colliding(p))
    {
        p.x = dist(mt) * (max_x - min_x) + min_x;
        p.y = dist(mt)* (max_y - min_y) + min_y;
        std::cout << "colliding\n";
        std::cout << p.x;
    }

    return p;
};