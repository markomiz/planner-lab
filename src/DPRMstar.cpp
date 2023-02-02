#include "Planner.h"
#include "collision_check.h"
#include <iostream>
#include <algorithm>
#include <vector>

void DPRMstar::genRoadmap(int n, int angles)
{
    cout <<" gen roadmap pluss\n";
    float yprm  = sqrt(2*(1+ 1/2)) * sqrt(map->getFreeSpace()/M_PI) * config->getConnectDist();
    //  init empty graph
    n_connections = 0;
    float d_ang = M_PI /float(angles);
    int halton_index = 0;
    ofstream myfile("nodes.txt");
    for (auto i = 0; i < config->getNumPoints(); i ++)
    {
        point2d new_p;
        new_p = map->halton_sample(halton_index);
        //if (map->colliding(new_p)) continue;
        float rad = yprm*sqrt(log(i+1)/(i+1));
        pose2d new_pose;
        new_pose.x = new_p;
        pose2d c_pose = new_pose;
        shared_ptr<Bundle> new_bundle(new Bundle());
        new_bundle->pos = new_p;
        std::vector<shared_ptr<Bundle>> nearest = graph->in_range(new_p,rad); //find all nodes within a Radius
        for (auto a = 0; a < angles; a ++)
        {
            shared_ptr<Node> new_node(new Node(new_pose));
            shared_ptr<Node> cor(new Node(c_pose));
            new_node->opposite = cor;
            cor->opposite = new_node;
            new_node->pt.theta = a * d_ang;
            cor->pt.theta = arc::mod2pi(a * d_ang+ M_PI);
            for (auto x = 0; x < nearest.size(); x++)
            {
                for (auto b = 0; b < nearest[x]->nodes.size(); b++)
                {
                    // one way
                    dubins_params sol = dCurve->calculateSinglePath(new_node->pt, nearest[x]->nodes[b]->pt);
                    arcs A = arcs(new_node->pt, sol);
                    float dist = (new_node->pt.x - nearest[x]->nodes[b]->pt.x).norm();
                    if (!map->colliding(A) &&  A.L < dist * M_PI/2){
                        graph->add(new_node, nearest[x]->nodes[b], A);
                        n_connections ++;
                    }
                    // cor
                    sol = dCurve->calculateSinglePath(cor->pt, nearest[x]->nodes[b]->pt);
                    arcs A2 = arcs(cor->pt, sol);
                    dist = (cor->pt.x - nearest[x]->nodes[b]->pt.x).norm();
                    if (!map->colliding(A) &&  A.L < dist * M_PI/2){
                        graph->add(cor, nearest[x]->nodes[b], A2);
                        n_connections ++;
                    }
                }
            };
            new_bundle->nodes.push_back(new_node);
            new_bundle->nodes.push_back(cor);
            graph->nodes.push_back(new_node);
            graph->nodes.push_back(cor);
            myfile << new_node->pt.x.x << "; " << new_node->pt.x.y << endl;
        }
        graph->points_quad.add_bundle(new_bundle);
    }
    cout << n_connections <<" connections test \n";
    myfile.close();
};

deque<arcs> DPRMstar::getPath(pose2d start, pose2d end)
{
    const float TRSH = config->getStartEndThrsh();
    // fisrt connect start and end to graph
    std::vector<shared_ptr<Bundle>> nearest_s = graph->in_range(start.x, TRSH); // find all nodes within a Rad
    shared_ptr<Node> start_node(new Node(start));
    pose2d start_c_pose = start;
    start_c_pose.theta = arc::mod2pi(start.theta + M_PI);
    shared_ptr<Node> cor(new Node(start_c_pose));
    start_node->opposite = cor;
    cor->opposite = start_node;
    for (auto x = 0; x < nearest_s.size(); x++)
    {
        for (auto a = 0; a < nearest_s[x]->nodes.size(); a++)
        {

            dubins_params sol = dCurve->calculateSinglePath(start, nearest_s[x]->nodes[a]->pt);
            arcs A(start, sol);
            // if doesn't collide add connections to graph
            float dist = (start_node->pt.x - nearest_s[x]->nodes[a]->pt.x).norm();
            if (map->colliding(A) ||  A.L < dist * M_PI/3) continue;
            graph->add(start_node, nearest_s[x]->nodes[a], A);
        }

    };

    std::vector<shared_ptr<Bundle>> nearest_e = graph->in_range(end.x, TRSH); // find all nodes within a Rad
    shared_ptr<Node> end_node( new Node(end));
    pose2d end_c_pose = end;
    end_c_pose.theta = arc::mod2pi(end.theta + M_PI);
    shared_ptr<Node> cor_e(new Node(end_c_pose));
    end_node->opposite = cor_e;
    cor_e->opposite = end_node;
    for (auto y = 0; y < nearest_e.size(); y++)
    {
        for (auto a = 0; a < nearest_e[y]->nodes.size(); a++)
        {
            dubins_params sol = dCurve->calculateSinglePath( nearest_e[y]->nodes[a]->pt, end);
            arcs A(nearest_e[y]->nodes[a]->pt, sol);
            // if doesn't collide add connections to graph
            float dist = (end_node->pt.x - nearest_e[y]->nodes[a]->pt.x).norm();
            if (map->colliding(A) ||  A.L < dist * M_PI/3) continue;
            graph->add(nearest_e[y]->nodes[a], end_node,  A);
        }

    };
    shared_ptr<Bundle> end_bundle(new Bundle());
    shared_ptr<Bundle> start_bundle(new Bundle());

    end_bundle->pos = end.x;
    start_bundle->pos = start.x;

    start_bundle->nodes.push_back(start_node);
    start_bundle->nodes.push_back(cor);
    end_bundle->nodes.push_back(end_node);
    end_bundle->nodes.push_back(cor_e);


    graph->points_quad.add_bundle(start_bundle);
    graph->points_quad.add_bundle(end_bundle);

    graph->nodes.push_back(start_node);
    graph->nodes.push_back(end_node);
    graph->nodes.push_back(cor);
    graph->nodes.push_back(cor_e);

    deque<arcs> points = graph->getPathPlus(start_node, end_node);
    return points;
}

deque<arcs> DPRMstar::getPathManyExits(pose2d start, vector<pose2d> end)
{
    float TRSH = config->getStartEndThrsh();
    // fisrt connect start and end to graph
    std::vector<shared_ptr<Bundle>> nearest_s = graph->in_range(start.x, TRSH); // find all nodes within a Rad
    shared_ptr<Node> start_node(new Node(start));
    pose2d start_c_pose = start;
    start_c_pose.theta = arc::mod2pi(start.theta + M_PI);
    shared_ptr<Node> cor(new Node(start_c_pose));
    start_node->opposite = cor;
    cor->opposite = start_node;
    for (auto x = 0; x < nearest_s.size(); x++)
    {
        for (auto a = 0; a < nearest_s[x]->nodes.size(); a++)
        {

            dubins_params sol = dCurve->calculateSinglePath(start, nearest_s[x]->nodes[a]->pt);
            arcs A(start, sol);
            // if doesn't collide add connections to graph
            float dist = (start_node->pt.x - nearest_s[x]->nodes[a]->pt.x).norm();
            if (map->colliding(A) ||  A.L < dist * M_PI/3) continue;
            graph->add(start_node, nearest_s[x]->nodes[a], A);
        }

    };
    vector<shared_ptr<Node>> end_nodes;
    
    for (auto i = 0 ; i < end.size(); i++)
    {
        shared_ptr<Bundle> end_bundle(new Bundle());
        end_bundle->pos = end[i].x;
        std::vector<shared_ptr<Bundle>> nearest_e = graph->in_range(end[i].x, TRSH); // find all nodes within a Rad
        shared_ptr<Node> end_node( new Node(end[i]));
        pose2d end_c_pose = end[i];
        end_c_pose.theta = arc::mod2pi(end[i].theta + M_PI);
        shared_ptr<Node> cor_e(new Node(end_c_pose));
        end_node->opposite = cor_e;
        cor_e->opposite = end_node;
        for (auto y = 0; y < nearest_e.size(); y++)
        {
            for (auto a = 0; a < nearest_e[y]->nodes.size(); a++)
            {
                dubins_params sol = dCurve->calculateSinglePath( nearest_e[y]->nodes[a]->pt, end[i]);
                arcs A(nearest_e[y]->nodes[a]->pt, sol);
                // if doesn't collide add connections to graph
                float dist = (end_node->pt.x - nearest_e[y]->nodes[a]->pt.x).norm();
                if (map->colliding(A) ||  A.L < dist * M_PI/3) continue;
                graph->add(nearest_e[y]->nodes[a], end_node,  A);
            }

        };
        graph->nodes.push_back(end_node);
        graph->nodes.push_back(cor_e);
        end_nodes.push_back(end_node);
        end_bundle->nodes.push_back(end_node);
        end_bundle->nodes.push_back(cor_e);
        graph->points_quad.add_bundle(end_bundle);

    }

    shared_ptr<Bundle> start_bundle(new Bundle());

    graph->points_quad.add_bundle(start_bundle);
    

    graph->nodes.push_back(start_node);
    graph->nodes.push_back(cor);

    deque<arcs> points = graph->getPathPlusManyExits(start_node, end_nodes);

    return points;
 };