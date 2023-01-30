#include "Planner.h"
#include "collision_check.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

deque<arcs> Planner::getPath(point2d start, point2d end)
{
    float TRSH = config->getStartEndThrsh();
    // first connect start and end to graph
    std::vector<shared_ptr<Bundle>> nearest_s = graph->in_range(start, TRSH); // find all nodes within a Rad
    pose2d start_pose;
    start_pose.x = start;
    shared_ptr<Node> start_node(new Node(start_pose));
    
    for (auto x = 0; x < nearest_s.size(); x++)
    {
        for (auto a = 0; a < nearest_s[x]->nodes.size(); a++)
        {
            line l;
            l.p_final = start;
            l.p_initial = nearest_s[x]->nodes[a]->pt.x;
            if (!map->colliding(l)) {
                graph->add(start_node, nearest_s[x]->nodes[a]);
            };
        }
    }
    std::vector<shared_ptr<Bundle>> nearest_e = graph->in_range(end, TRSH); // find all nodes within a Rad
    pose2d end_pose;
    end_pose.x = end;
    shared_ptr<Node> end_node( new Node(end_pose));

    for (auto x = 0; x < nearest_e.size(); x++)
    {
        for (auto a = 0; a < nearest_e[x]->nodes.size(); a++)
        {
            line l;
            l.p_final = end;
            l.p_initial = nearest_e[x]->nodes[a]->pt.x;
            
            if (!map->colliding(l)){
                graph->add(nearest_e[x]->nodes[a], end_node);
            }
        }
    }
    cout << "start connections: " << nearest_s.size() << endl;
    cout << "end connections: " << nearest_e.size() << endl;
    shared_ptr<Bundle> start_bundle(new Bundle());
    start_bundle->pos = start;
    shared_ptr<Bundle> end_bundle(new Bundle());
    end_bundle->pos = end;
    end_bundle->nodes.push_back(end_node);
    start_bundle->nodes.push_back(start_node);
    // Add start and end node to the graph 
    graph->nodes.push_back(start_node);
    graph->points_quad.add_bundle(start_bundle);
    graph->nodes.push_back(end_node);
    graph->points_quad.add_bundle(end_bundle);
    cout << "Still good";
    deque<point2d> points = graph->getPath(start_node, end_node);
    deque<arcs> dubined = dCurve->calculateMultiPoint(start_pose, end_pose, points, config->getNumAngles());
    return dubined;
}

deque<arcs> Planner::smoothWithMulti(deque<arcs> original)
{
    pose2d start = original[0].a[0].start;
    pose2d end = original.back().a[2].end;
    deque<point2d> mids;
    for (int i = 2; i < original.size(); i++)
    {
        mids.push_back(original[i].a[0].start.x);
    }
    // mids.push_back(point2d(-4,-5));
    auto p = dCurve->calculateMultiPoint(start, end, mids, 36 );
    // deque<arcs> p2 = arcs(start, p);
    return p;

};