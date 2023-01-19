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
    cout <<" gen roadmap pluss\n";
    float yprm  = sqrt(2*(1+ 1/2)) * sqrt(map->getFreeSpace()/M_PI) * 1.0;
    //  init empty graph
    int cons = 0;

    ofstream node_file ("nodes.txt");
    
    float d_ang = M_PI * 2/float(angles);
    for (auto i = 0; i < n; i ++)
    {

        point2d new_p = map->uniform_sample();
        pose2d new_pose;
        new_pose.x = new_p;
        pose2d c_pose = new_pose;
        float rad = yprm*sqrt(log(i+1)/(i+1));
        //cout << rad << " rad \n";
        cout <<" pre";
        std::vector<shared_ptr<Node>> nearest = graph->in_range(new_p,rad); //find all nodes within a Radius
        cout <<" post";
        //cout << nearest.size() << " near \n";
        for (int a = 0; a < angles; a ++)
        {
            pose2d new_pose;
            new_pose.x = new_p;
            pose2d c_pose = new_pose;
            shared_ptr<Node> new_node(new Node(new_pose));
            shared_ptr<Node> cor(new Node(c_pose));
            
            new_node->opposite = cor;
            cor->opposite = new_node;
            for (auto x = 0; x < nearest.size(); x++)
            {
                new_node->pt.theta = a * d_ang;
                cor->pt.theta = arc::mod2pi(a * d_ang+ M_PI);
                bool col_arc = false;

                dubins_params sol = dCurve->calculateSinglePath(new_node->pt, nearest[x]->pt);

                arcs A = arcs(new_node->pt, sol);

                if (map->colliding(A) || A.L > rad){
                    //if (map->colliding(A)) cout << "arc collides!" << i << "\n";
                    continue;
                }
                graph->add(new_node, nearest[x], A);
                cons ++;
            };
            graph->nodes.push_back(new_node);
            graph->points_quad.insert(new_node);
            graph->nodes.push_back(cor);
            graph->points_quad.insert(cor);
            node_file << new_node->pt.x.x << "; " << new_node->pt.x.y << "\n";
        }
    }
    node_file.close();
    cout << cons <<" connections test \n";
};

deque<arcs> PRMstar::getPath(pose2d start, pose2d end)
{
    float TRSH = 2.0;
    // fisrt connect start and end to graph
    std::vector<shared_ptr<Node>> nearest_s = graph->in_range(start.x, TRSH); // find all nodes within a Rad
    shared_ptr<Node> start_node(new Node(start));
    pose2d start_c_pose = start;
    start_c_pose.theta = arc::mod2pi(start.theta + M_PI);
    shared_ptr<Node> cor(new Node(start_c_pose));
    start_node->opposite = cor;
    cor->opposite = start_node;
    cout << "Nearby nodes: " << nearest_s.size() << endl;
    for (auto x = 0; x < nearest_s.size(); x++)
    {
        // gen dubins
        bool col_arc = false;
        dubins_params sol = dCurve->calculateSinglePath(start, nearest_s[x]->pt);
        arcs A(start, sol);
        // if doesn't collide add connections to graph
        if (map->colliding(A))
        {
            continue;
        }
        graph->add(start_node, nearest_s[x], A);
    };
    cout << "Still fine " << nearest_s.size() << endl;
    std::vector<shared_ptr<Node>> nearest_e = graph->in_range(end.x, TRSH); // find all nodes within a Rad
    shared_ptr<Node> end_node( new Node(end));
    pose2d end_c_pose = start;
    end_c_pose.theta = arc::mod2pi(start.theta + M_PI);
    shared_ptr<Node> cor_e(new Node(end_c_pose));
    end_node->opposite = cor_e;
    cor_e->opposite = end_node;
    for (auto x = 0; x < nearest_e.size(); x++)
    {
        // gen dubins
        bool col_arc = false;
        dubins_params sol = dCurve->calculateSinglePath( nearest_e[x]->pt, end);
        arcs A(nearest_e[x]->pt, sol);
        // if doesn't collide add connections to graph
        if (map->colliding(A)) continue;
        graph->add(nearest_e[x], end_node,  A);
    };
    cout << "Nok33: " << nearest_s.size() << endl;
    graph->nodes.push_back(start_node);
    graph->nodes.push_back(cor);
    graph->points_quad.insert(start_node);
    graph->nodes.push_back(end_node);
    graph->nodes.push_back(cor_e);
    graph->points_quad.insert(end_node);
    graph->points_quad.insert(cor);
    graph->points_quad.insert(cor_e);
    cout << "hiit the graph \n";
    deque<arcs> points = graph->getPathPlus(start_node, end_node);
    return points;
}