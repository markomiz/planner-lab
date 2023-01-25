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
    edges.push_back(edge);
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
    // float a = edges[0].length();
    // area = verteces.size() * a * a / (4 * sin(M_PI/ a));

    int n = verteces.size();
    for(int i = 0; i < n - 1; i++) {
        area += verteces[i].x * verteces[i+1].y;

    }
    area += verteces[n-1].x * verteces[0].y;
    for(int i = 0; i < n - 1; i++) {
        area -= verteces[i].y * verteces[i+1].x;
    }
    area -= verteces[n-1].y * verteces[0].x;
    area = abs(area / 2.0);

};

void Polygon::getMinMax()
{
    vector<float> temp_vec_x;
    vector<float> temp_vec_y;
    for (int i = 0; i < verteces.size(); i++)
    {
        temp_vec_x.push_back(verteces[i].x);
        temp_vec_y.push_back(verteces[i].y);
    }

    x_min = *min_element(temp_vec_x.begin(), temp_vec_x.end());
    y_min = *min_element(temp_vec_y.begin(), temp_vec_y.end());
    x_max = *max_element(temp_vec_x.begin(), temp_vec_x.end());
    y_max = *max_element(temp_vec_y.begin(), temp_vec_y.end());
};

// Map Polygon::toMap()
// {
//     Map map(x_min,y_min,x_max,y_max);
//     return map;
// }

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

        float centre_dist = (point - obs.center).norm();

        if (centre_dist > obs.radius)
        {
            continue;
        }
        // otherwise check for polygon collision
        bool proof = CollisionCheck::point_in_polygon(point, obs);
        if (proof){
            return true;
        }
    }
    if (!inBounds(point)){

        return true;
    }
    
    return false;
};
bool Map::colliding(arcs A)
{
    for (int i=0; i < 3; i++)
    {
        if(A.a[i].radius == 0)
        {
            line temp;
            point2d temp_pt1(A.a[i].start.x.x, A.a[i].start.x.y);
            temp.p_initial = temp_pt1;
            point2d temp_pt2(A.a[i].end.x.x, A.a[i].end.x.y);
            temp.p_final = temp_pt2;
            if(colliding(temp)){
                return true;
            } 
        }
        else
        {
            
            if (colliding(A.a[i])){
                 return true;
            }
        }
    }
    // cout << "NOT COLLIDING!\n";
    return false;

}
bool Map::colliding(arc a)
{
    // obstacle check
    if (CollisionCheck::arc_with_polygon(a, total_map_poly)) return true;
    for (int i = 0; i < obstacles.size(); i++)
    {
        if ((a.center - obstacles[i].center).norm() > a.radius + obstacles[i].radius ) continue;
        if (CollisionCheck::arc_with_polygon(a, obstacles[i])) return true;
    }
    return false;
};
bool Map::colliding(line l)
{
    
    for (auto i = 0; i < obstacles.size(); i++)
    {
        Polygon obs = obstacles[i];
        // rough pass with radius of obstacles
        bool xout = false;
        bool yout = false;
        if ( (l.p_initial.x > obstacles[i].radius + obstacles[i].x_max && l.p_final.x > obstacles[i].radius  + obstacles[i].x_max) ) xout = true;
        else if ( (l.p_initial.x < obstacles[i].radius - obstacles[i].x_max && l.p_final.x < obstacles[i].radius  -  obstacles[i].x_max) ) xout = true;
        if (xout)
        {
        if ( (l.p_initial.y > obstacles[i].radius + obstacles[i].y_max && l.p_final.y > obstacles[i].radius  + obstacles[i].y_max) ) yout = true;
        else if ( (l.p_initial.y < obstacles[i].radius - obstacles[i].y_max && l.p_final.y < obstacles[i].radius  -  obstacles[i].y_max) ) yout = true;
        }
        if (xout & yout) continue;
        // second check more detailed check if rough pass not passing
        for (auto j = 0; j < obs.edges.size(); j++)
        {

            if (CollisionCheck::line_line_intersect(obs.edges[j], l).intersects)
            {
                return true;
            } 
        }
        
    }
    if (!inBounds(l.p_final)) return true;
    if (!inBounds(l.p_initial)) return true;

    return false;
};


bool Map::inBounds(point2d p)
{
    return CollisionCheck::point_in_polygon(p, total_map_poly);
};

void Map::processBounds(){
    vector<point2d> verteces = total_map_poly.verteces;
    vector<float> temp_vec_x;
    vector<float> temp_vec_y;
    for (int i = 0; i < verteces.size(); i++)
    {
        temp_vec_x.push_back(verteces[i].x);
        temp_vec_y.push_back(verteces[i].y);
    }

    min_x = *min_element(temp_vec_x.begin(), temp_vec_x.end());
    min_y = *min_element(temp_vec_y.begin(), temp_vec_y.end());
    max_x = *max_element(temp_vec_x.begin(), temp_vec_x.end());
    max_y = *max_element(temp_vec_y.begin(), temp_vec_y.end());
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
    }
    
    return p;
};

float Map::halton_min(int index, int base, float min, float max)
{
    float res = 0;
    double f = 1.0 / base;
    int j = index;
    while (j > 0) {
        res += f * (j % base);
        j /= base;
        f /= base;
    }
    return res;
};

point2d Map::halton_sample(int &i)
{
    point2d p;
    int base_x = 13, base_y = 7; //TODO - maybe move to config file

    p.x = (halton_min(i, base_x, min_x, max_x)*(max_x-min_x)) + min_x;
    p.y = (halton_min(i, base_y, min_y, max_y)*(max_y-min_y)) + min_y;

    while (colliding(p))
    {
        i++;
        p.x = (halton_min(i, base_x, min_x, max_x)*(max_x-min_x)) + min_x;
        p.y = (halton_min(i, base_y, min_y, max_y)*(max_y-min_y)) + min_y;
    }

    return p;
}

