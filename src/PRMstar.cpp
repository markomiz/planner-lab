#include "PRMstar.h"
#include "collision_check.h"

// max nodes
// radius of neighbourhood
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

void ExactCell::genRoadmap()
{
    vector<exactpoint2d> points;
    vector<Polygon> obstacles = map->getObstacles();
    for (int i = 0; i < obstacles.size(); i++)
    {
        for (int j = 0; j < obstacles[i].verteces.size(); j++)
        {
            exactpoint2d temp(obstacles[i].verteces[j].x, obstacles[i].verteces[j].y, i); 
            points.push_back(temp);
        }   
    }

    int n = points.size();

    quickSort(points, 0, n - 1);

    double maxy = 10;
    double miny = -10;
    double maxx = 10;
    double minx = -10;

    vector<point2d> nodes;
    point2d p(minx, miny + (maxy-miny)/2);
    
    nodes.push_back(p);

    // Line sweep to draw vertical lines from obstacle vertices

    for (int i = 0; i < n; i++)
    {
        // Create line downwards 
        point2d temp_down(points[i].x, miny);
        point2d temp_pt(points[i].x, points[i].y);
        line l_down(temp_pt, temp_down);
        int self_intersections = 0;
        for(int j = 0; j < obstacles[points[i].poly_id].edges.size(); j++)
        {
            if (CollisionCheck::line_line_intersect(obstacles[points[i].poly_id].edges[j], l_down).intersects) self_intersections++;
        }
        if (self_intersections <= 2)
        {
            float dist = __FLT_MAX__;
            point2d node(points[i].x, points[i].y);
            for (int j = 0; j < obstacles.size(); j++)
            {
                if (points[i].poly_id == j) continue;
                for (int edge_num = 0; edge_num < obstacles[j].edges.size(); edge_num++)
                {
                    if (CollisionCheck::line_line_intersect(obstacles[j].edges[j], l_down).intersects)
                    {
                        double new_dist = (CollisionCheck::line_line_intersect(obstacles[j].edges[j], l_down).intersection - temp_pt).norm();
                        if (new_dist < dist) 
                        {
                            dist = new_dist;
                        }
                    }
                }
                
            }
            if (dist == __FLT_MAX__) node.y -= (node.y-miny)/2;
            else node.y -= dist;
            nodes.push_back(node);

        }
        
        // Create line upwards
        point2d temp_up(points[i].x, maxy);
        line l_up(temp_pt, temp_up);
        self_intersections = 0;
        for(int j = 0; j < obstacles[points[i].poly_id].edges.size(); j++)
        {
            if (CollisionCheck::line_line_intersect(obstacles[points[i].poly_id].edges[j], l_up).intersects) self_intersections++;
        }
        if (self_intersections <= 2)
        {
            float dist = __FLT_MAX__;
            point2d node(points[i].x, points[i].y);
            for (int j = 0; j < obstacles.size(); j++)
            {
                if (points[i].poly_id == j) continue;
                for (int edge_num = 0; edge_num < obstacles[j].edges.size(); edge_num++)
                {
                    if (CollisionCheck::line_line_intersect(obstacles[j].edges[j], l_up).intersects)
                    {
                        double new_dist = (CollisionCheck::line_line_intersect(obstacles[j].edges[j], l_up).intersection - temp_pt).norm();
                        if (new_dist < dist) 
                        {
                            dist = new_dist;
                        }
                    }
                }
                
            }
            if (dist == __FLT_MAX__) node.y += (node.y-miny)/2;
            else node.y += dist;
            nodes.push_back(node);
        }
    }


    point2d p2(maxx, miny + (maxy-miny)/2);
    nodes.push_back(p2);

    // Start connecting nodes to form the graph
    
    ofstream node_file ("nodes.txt");
    int cons = 0;
    for (auto i = 0; i < nodes.size(); i++)
    {
        pose2d new_pose(nodes[i].x, nodes[i].y, 0);
        shared_ptr<Node> new_node(new Node(new_pose));
        for (auto j = 0; j < i; j++)
        {
            pose2d near_pose(nodes[j].x, nodes[j].y, 0);
            shared_ptr<Node> near_node(new Node(near_pose));
            // one way
            line l(new_pose.x, near_pose.x);
            // float dist = (new_node->pt.x - near_node->pt.x).norm();
            if (!map->colliding(l) /* &&  A.L < dist * M_PI/2 */){
                graph->add(new_node, near_node);
                cons ++;
            }
        }    
    }
    node_file.close();
    cout << cons <<" connections test \n";
};

bool ExactCell::comparePoints(exactpoint2d p1, exactpoint2d p2) {
    if (p1.x == p2.x) {
        return (p1.y < p2.y);
    }
    return (p1.x < p2.x);
}

void ExactCell::quickSort(vector<exactpoint2d> &points, int low, int high) {
    if (low < high) {
        int pivotIndex = (low + high) / 2;
        exactpoint2d pivot = points[pivotIndex];

        int i = low;
        int j = high;

        while (i <= j) {
            while (comparePoints(points[i], pivot)) {
                i++;
            }
            while (comparePoints(pivot, points[j])) {
                j--;
            }
            if (i <= j) {
                swap(points[i], points[j]);
                i++;
                j--;
            }
        }

        quickSort(points, low, j);
        quickSort(points, i, high);
    }
}

void GeometricPRMstar::genRoadmap(int n)
{
    float yprm  = sqrt(2*(1+ 1/2)) * sqrt(map->getFreeSpace()/M_PI) * 1.0;
    //  init empty graph
    int cons = 0;
    for (auto i = 0; i < n; i ++)
    {
        
        // point2d new_point = map->halton_sample(i);
        point2d new_point = map->uniform_sample();
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
                    cons ++;
                };
            }

        }
        shared_ptr<Bundle> new_bundle(new Bundle());
        new_bundle->pos = new_point;
        new_bundle->nodes.push_back(new_node);
        graph->nodes.push_back(new_node);
        graph->points_quad.add_bundle(new_bundle);
    }

};

vector<point2d> Planner::getPath(point2d start, point2d end)
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
                graph->add(end_node, nearest_e[x]->nodes[a]);
            };
        }

    }
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
    std::vector<point2d> points = graph->getPath(start_node, end_node);
    return points;
}

void DubinsPRMstar::genRoadmap(int n, int angles)
{
    cout <<" gen roadmap pluss\n";
    float yprm  = sqrt(2*(1+ 1/2)) * sqrt(map->getFreeSpace()/M_PI) * config->getConnectDist();
    //  init empty graph
    ofstream node_file ("nodes.txt");

    int cons = 0;
    float d_ang = M_PI /float(angles);
    for (auto i = 0; i < config->getNumPoints(); i ++)
    {

        point2d new_p;
        new_p = map->halton_sample(i);
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
                        cons ++;
                    }
                    // cor
                    sol = dCurve->calculateSinglePath(cor->pt, nearest[x]->nodes[b]->pt);
                    A = arcs(cor->pt, sol);
                    dist = (cor->pt.x - nearest[x]->nodes[b]->pt.x).norm();
                    if (!map->colliding(A) &&  A.L < dist * M_PI/2){
                        graph->add(cor, nearest[x]->nodes[b], A);
                        cons ++;
                    }
                }
            };
            node_file << new_node->pt.x.x << "; " << new_node->pt.x.y << "\n";
            new_bundle->nodes.push_back(new_node);
            new_bundle->nodes.push_back(cor);
            graph->nodes.push_back(new_node);
            graph->nodes.push_back(cor);
            
        }
        graph->points_quad.add_bundle(new_bundle);
    }
    node_file.close();
    cout << cons <<" connections test \n";
};

deque<arcs> DubinsPRMstar::getPath(pose2d start, pose2d end)
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

    cout << "hiit the graph \n";
    deque<arcs> points = graph->getPathPlus(start_node, end_node);
    return points;
}

deque<arcs> DubinsPRMstar::getPathManyExits(pose2d start, vector<pose2d> end)
{
    cout << "Gate 1:" << end[0].x.x << "," << end[0].x.y << "," << end[0].theta << endl;
    cout << "Gate 2:" << end[1].x.x << "," << end[1].x.y << "," << end[1].theta << endl;
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
    
    cout << "hiit the graph \n";
    deque<arcs> points = graph->getPathPlusManyExits(start_node, end_nodes);
    // TRY DO A MULTIPOINT ONCE IT'S DONE?
    return points;
};