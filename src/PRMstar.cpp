#include "PRMstar.h"
#include "collision_check.h"

// max nodes
// radius of neighbourhood
void PRMstar::genRoadmap(int n)
{
    float yprm  = sqrt(2*(1+ 1/2)) * sqrt(map->getFreeSpace()/M_PI);
    //  init empty graph
    for (int i = 0; i < n; i ++)
    {
        point2d new_point = uniform_point();
        float rad = yprm*sqrt(log(i)/(i));
        std::vector<Node*> nearest = graph->in_range(new_point,rad); //find all nodes within a Rad 

        Node* new_node = new Node(&new_point);
        for (int x = 0; x < nearest.size(); x++)
        {
            line l;
            l.p_final = new_point;
            l.p_initial = *nearest[x]->pt;
            if (map->uncolliding(l)){
                graph->add(new_node, nearest[x]);
            } 
        }
        graph->nodes.push_back(new_node);
        graph->points_quad.insert(new_node);
    }
};

vector<point2d> PRMstar::getPath(point2d start, point2d end)
{
    float TRSH = 0.5;
    // fisrt connect start and end to graph
    std::vector<Node*> nearest_s = graph->in_range(start, TRSH); // find all nodes within a Rad
    Node* start_node = new Node(&start);

    for (int x = 0; x < nearest_s.size(); x++)
    {
        line l;
        l.p_final = start;
        l.p_initial = *nearest_s[x]->pt;
        if (map->uncolliding(l)) graph->add(start_node, nearest_s[x]);
    }

    std::vector<Node*> nearest_e = graph->in_range(end, TRSH); // find all nodes within a Rad
    Node* end_node = new Node(&end);
    for (int x = 0; x < nearest_e.size(); x++)
    {
        line l;
        l.p_final = end;
        l.p_initial = *nearest_e[x]->pt;
        if (map->uncolliding(l)) graph->add(end_node, nearest_e[x]);
    }
    graph->nodes.push_back(start_node);
    graph->points_quad.insert(start_node);
    graph->nodes.push_back(end_node);
    graph->points_quad.insert(end_node);

    return graph->getPath(start_node, end_node);
}
