#include "PRMstar.h"
#include "collision_check.h"

// max nodes
// radius of neighbourhood
void PRMstar::genRoadmap(int n)
{
    //  init empty graph
    for (int i = 0; i < n; i ++)
    {
        point2d new_point = uniform_point();
        float rad = yprm*(log(itr)/(itr))^(1/d)
        std::vector<Node> nearest = graph.nearest(point,rad); //find all nodes within a Rad
        for (int x = 0; x < nearest.size(); x++)
        {
            line l;
            l.p_final = new_point;
            l.p_initial = nearest[x].point;
            if (! map.uncolliding(l)) graph.add(new_point, nearest[x]);
        }

    }
};

void PRMstar::getPath(point2d start, point2d end)
{
    
}