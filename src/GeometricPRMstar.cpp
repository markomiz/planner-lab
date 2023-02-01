
#include "Planner.h"
#include "collision_check.h"
#include <iostream>
#include <algorithm>
#include <vector>
void GeometricPRMstar::genRoadmap(int n, int angles)
{
    float yprm  = sqrt(2*(1+ 1/2)) * sqrt(map->getFreeSpace()/M_PI) * config->getConnectDist();
    n_connections = 0;
    int halton_index = 0;
    for (auto i = 0; i < n; i ++)
    {
        point2d new_point = map->halton_sample(halton_index);
        pose2d new_pose;
        new_pose.x = new_point;
        float rad = yprm*sqrt(log(i+1)/(i+1));
        std::vector<shared_ptr<Bundle>> nearest = graph->in_range(new_point,rad); //find all nodes within a Rad 
        shared_ptr<Node> new_node(new Node(new_pose));
        for (auto x = 0; x < nearest.size(); x++)
        {
            for (auto a = 0; a < nearest[x]->nodes.size(); a++)
            {
                line l;
                l.p_final = new_point;
                l.p_initial = nearest[x]->nodes[a]->pt.x;
                if (!map->colliding(l)){
                    graph->add(new_node, nearest[x]->nodes[a]);
                    n_connections++;
                };
            }
        }
        shared_ptr<Bundle> new_bundle(new Bundle());
        new_bundle->pos = new_point;
        new_bundle->nodes.push_back(new_node);
        graph->nodes.push_back(new_node);
        graph->points_quad.add_bundle(new_bundle);
    }
    cout << n_connections <<" connections test \n";
};