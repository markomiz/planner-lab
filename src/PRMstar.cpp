#include "PRMstar.h"
#include "collision_check.h"

// max nodes
// radius of neighbourhood

void PRMstar::genRoadmap(int n)
{
    float yprm  = sqrt(2*(1+ 1/2)) * sqrt(map->getFreeSpace()/M_PI) * 1.0;
    //  init empty graph
    int cons = 0;
    for (auto i = 0; i < n; i ++)
    {
        point2d new_point = map->uniform_sample();
        pose2d new_pose;
        new_pose.x = new_point;
        float rad = yprm*sqrt(log(i+1)/(i+1));
        std::vector<shared_ptr<Node>> nearest = graph->in_range(new_point,rad); //find all nodes within a Rad 
        shared_ptr<Node> new_node(new Node(new_pose));
        // cout << " rad - " << rad << "   " << nearest.size() << " num near  \n";
        for (auto x = 0; x < nearest.size(); x++)
        {
            line l;
            l.p_final = new_point;
            l.p_initial = nearest[x]->pt.x;
            if (!map->colliding(l)){
                graph->add(new_node, nearest[x]);
                cons ++;
            };
        }
        graph->nodes.push_back(new_node);
        graph->points_quad.insert(new_node);
    }
    cout << cons << " TOTAL CONNECTIONS \n";
    // graph->print_nodes();
};

vector<point2d> PRMstar::getPath(point2d start, point2d end)
{
    float TRSH = 1.0;
    // first connect start and end to graph
    std::vector<shared_ptr<Node>> nearest_s = graph->in_range(start, TRSH); // find all nodes within a Rad
    pose2d start_pose;
    start_pose.x = start;
    shared_ptr<Node> start_node(new Node(start_pose));
    
    // Eval if new node connection dont collide linearly
    int num = nearest_s.size();
    for (auto x = 0; x < nearest_s.size(); x++)
    {
        line l;
        l.p_final = start;
        l.p_initial = nearest_s[x]->pt.x;
        if (!map->colliding(l)) {
            graph->add(start_node, nearest_s[x]);

        };
    }
    std::vector<shared_ptr<Node>> nearest_e = graph->in_range(end, TRSH); // find all nodes within a Rad
    pose2d end_pose;
    end_pose.x = end;
    shared_ptr<Node> end_node( new Node(end_pose));

    // Eval if new node connection dont collide linearly
    int nume = nearest_e.size();
    for (auto x = 0; x < nearest_e.size(); x++)
    {
        line l;
        l.p_final = end;
        l.p_initial = nearest_e[x]->pt.x;
        if (!map->colliding(l)){
             graph->add(end_node, nearest_e[x]);
        };
    }

    // Add start and end node to the graph 
    graph->nodes.push_back(start_node);
    graph->points_quad.insert(start_node);
    graph->nodes.push_back(end_node);
    graph->points_quad.insert(end_node);
    std::vector<point2d> points = graph->getPath(start_node, end_node);
    return points;
}

void PRMstar::genRoadmapPlus(int n, int angles)
{
    float yprm  = sqrt(2*(1+ 1/2)) * sqrt(map->getFreeSpace()/M_PI) * 1.0;
    //  init empty graph
    int cons = 0;
    float d_ang = M_PI * 2/float(angles);
    for (auto i = 0; i < n; i ++)
    {
        point2d new_p = map->uniform_sample();
        pose2d new_pose;
        new_pose.x = new_p;
        float rad = yprm*sqrt(log(i+1)/(i+1));
        std::vector<shared_ptr<Node>> nearest = graph->in_range(new_p,rad); //find all nodes within a Radius
        for (int a = 0; a < angles; a ++)
        {
            shared_ptr<Node> new_node(new Node(new_pose));
            for (auto x = 0; x < nearest.size(); x++)
            {
                new_node->pt.theta = a * d_ang;
                // gen dubins
                bool col_arc = false;
                dubins_params sol = dCurve->calculateSinglePath(new_pose, nearest[x]->pt);
                arcs A(new_node->pt, sol);
                if (map->colliding(A) || A.L > rad) continue;
                graph->add(new_node, nearest[x], A);
                cons ++;
            };
            graph->nodes.push_back(new_node);
            graph->points_quad.insert(new_node);
        }
    }
    cout << cons <<" connections  \n";
};

vector<arcs> PRMstar::getPath(pose2d start, pose2d end)
{
    float TRSH = 1.0;
    // fisrt connect start and end to graph
    std::vector<shared_ptr<Node>> nearest_s = graph->in_range(start.x, TRSH); // find all nodes within a Rad
    shared_ptr<Node> start_node(new Node(start));
    for (auto x = 0; x < nearest_s.size(); x++)
    {
        // gen dubins
        bool col_arc = false;
        dubins_params sol = dCurve->calculateSinglePath(start, nearest_s[x]->pt);
        arcs A(start, sol);
        // if doesn't collide add connections to graph
        if (map->colliding(A)) continue;
        graph->add(start_node, nearest_s[x], A);
    };

    std::vector<shared_ptr<Node>> nearest_e = graph->in_range(end.x, TRSH); // find all nodes within a Rad
    shared_ptr<Node> end_node( new Node(end));
    for (auto x = 0; x < nearest_s.size(); x++)
    {
        // gen dubins
        bool col_arc = false;
        dubins_params sol = dCurve->calculateSinglePath(end, nearest_s[x]->pt);
        arcs A(end, sol);
        // if doesn't collide add connections to graph
        if (map->colliding(A)) continue;
        graph->add(end_node, nearest_s[x], A);
    };
    graph->nodes.push_back(start_node);
    graph->points_quad.insert(start_node);
    graph->nodes.push_back(end_node);
    graph->points_quad.insert(end_node);
    cout << "hiit the graph \n";
    std::vector<arcs> points = graph->getPathPlus(start_node, end_node);
    return points;
}