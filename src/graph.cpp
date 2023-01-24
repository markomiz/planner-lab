#pragma once
#include "geometry.h"
#include "graph.h"
#include <iostream>
#include <deque>
using namespace std;
Graph::Graph(int max_depth, point2d tl, point2d br) : points_quad(max_depth, tl, br)
{

};
quad::quad(int max_depth, point2d tl, point2d br) : tl(tl), br(br), max_depth(max_depth)
{
    bl.x = tl.x;
    bl.y = br.y;
    tr.x = br.x;
    tr.y = tl.y;
    tl_tree = NULL;
    bl_tree = NULL;
    tr_tree = NULL;
    br_tree = NULL;
    
    center.x = (tr.x + bl.x) / 2;
    center.y = (tr.x + bl.x) / 2;
    radius = (tr - bl).norm() / 2;

}
bool quad::insert(shared_ptr<Node> node)
{
    if (max_depth == 0) { nodes.push_back(node);
        return true;
    }
    if ((tl.x + br.x) / 2 >= node->pt.x.x) {
        // Indicates tl_tree
        if ((tl.y + br.y) / 2 >= node->pt.x.y) {
            if (tl_tree == NULL)
                tl_tree = new quad( max_depth - 1,
                    point2d(tl.x, tl.y),
                    point2d((tl.x + br.x) / 2,
                          (tl.y + br.y) / 2));
            tl_tree->insert(node);
        }
        else {
            if (bl_tree == NULL)
                bl_tree = new quad( max_depth - 1,
                    point2d(tl.x,
                          (tl.y + br.y) / 2),
                    point2d((tl.x + br.x) / 2,
                          br.y));
            bl_tree->insert(node);
        }
    }
    else {
        if ((tl.y + br.y) / 2 >= node->pt.x.y) {
            if (tr_tree == NULL)
                tr_tree = new quad( max_depth - 1,
                    point2d((tl.x + br.x) / 2,
                          tl.y),
                    point2d(br.x,
                          (tl.y + br.y) / 2));
            tr_tree->insert(node);
        }
        else {
            if (br_tree == NULL)
                br_tree = new quad( max_depth - 1,
                    point2d((tl.x + br.x) / 2,
                          (tl.y + br.y) / 2),
                    point2d(br.x, br.y));
            br_tree->insert(node);
        }
    }
};
bool quad::overlaps(point2d pt, float r)
{
    if ((pt - center).norm() < r + radius) return true; // circular approx
    return false;
}
vector<shared_ptr<Node>> quad::in_range(point2d pt, float radius)
{
    vector<shared_ptr<Node>> all;
    for (auto i = 0; i < nodes.size(); i++)
    {
        if (((nodes[i]->pt.x) - pt).norm() < radius) {
            all.push_back(nodes[i]);
        }
    }
    // otherwise, choose which subtrees within range,
    if (tl_tree)
    {
        if (tl_tree->overlaps(pt,radius))
        {
            vector<shared_ptr<Node>> tl_all = tl_tree->in_range(pt, radius);
            all.insert(all.end(), tl_all.begin(), tl_all.end());
        }
    }
    if (bl_tree)
    {
        if (bl_tree->overlaps(pt,radius))
        {
            vector<shared_ptr<Node>> bl_all = bl_tree->in_range(pt, radius);
            all.insert(all.end(), bl_all.begin(), bl_all.end());
        }
    }
    if (tr_tree)
    {
        if (tr_tree->overlaps(pt,radius))
        {
            vector<shared_ptr<Node>> tr_all = tr_tree->in_range(pt, radius);
            all.insert(all.end(), tr_all.begin(), tr_all.end());
        }
    }
    if (br_tree)
    {
        if (br_tree->overlaps(pt,radius))
        {
            vector<shared_ptr<Node>> br_all = br_tree->in_range(pt, radius);
            all.insert(all.end(), br_all.begin(), br_all.end());
        }
    }

    return all;
};

/*
* Adding node considering linear connection between nodes
* Inputs: point - point that we are considering to add to the graph
          existing - closest point in the nearest k members that we are connecting to
* Outputs: point - new node added to the graph
*/
shared_ptr<Node> Graph::add(shared_ptr<Node> point, shared_ptr<Node> existing)
{
    
    float dist = (point->pt.x - existing->pt.x).norm();
    connection c1;
    c1.node = existing;
    c1.cost = dist;
    point->connected.push_back(c1);
    connection c2;
    c2.node = point;
    c2.cost = dist;
    existing->connected.push_back(c2);
    return point;

};



shared_ptr<Node> Graph::add(shared_ptr<Node> point, shared_ptr<Node> existing, arcs A)
{
    connection c1;
    c1.node = existing;
    c1.cost = A.L;
    c1.A = A;

    connection c2;
    c2.node = point->opposite;
    c2.cost = A.L;
    c2.A = A.get_inverse();

    point->connected.push_back(c1);

    existing->opposite->connected.push_back(c2);



    return point;

};
vector<shared_ptr<Node>> Graph::in_range(point2d pt, float rad)
{
    vector<shared_ptr<Node>> points = points_quad.in_range( pt, rad);
    return points;
};
void Graph::reset_nodes()
{
    for (auto i = 0; i < nodes.size(); i++)
    {
        // delete nodes[i]->parent;
        nodes[i]->cost = 0;
        nodes[i]->opened = false;
    }
}
vector<point2d> Graph::getPath(shared_ptr<Node> start, shared_ptr<Node> end)
{
    vector<point2d> points;
    // init open list
    vector<shared_ptr<Node>> OPEN;
    // add start node on open list
    shared_ptr<Node> current = nullptr;
    OPEN.push_back(start);
    while (OPEN.size() > 0)
    {   
        auto cur_it = OPEN.begin();
        current = *cur_it;
        // always work on minimal cost node in open set
        for (auto it = OPEN.begin(); it != OPEN.end(); it++)
        {
            if ((*it)->cost < current->cost)
            { 
                cur_it = it;
                current = *cur_it;
            }
        }
        OPEN.erase(cur_it);
        if (current == end) {
            break;
        }
        // for all nodes connected to current
        for (auto i = 0; i < current->connected.size(); i++)
        {
            if ( ! current->connected[i].node->opened )
            {
                // if not opened, add to open, store that current is parent
                // update their cost to current + dist between nodes
                current->connected[i].node->parent = current; // 
                OPEN.push_back(current->connected[i].node);
                current->connected[i].node->opened = true;
                current->connected[i].node->cost = current->cost + current->connected[i].cost;
            }
        }
    }
    std::cout << "still working \n";
    if (!current->parent) {
        std::cout << "oopsie no path \n";
        return points;
    }
    while (current->parent != start)
    {
        points.push_back(current->pt.x);
        current = current->parent;
    }
    std::cout << "we made it this far! \n";
    reset_nodes(); 
    
    current = nullptr;
    OPEN.clear();
    std::cout << "further! \n";
    return points;
};
deque<arcs> Graph::getPathPlus(shared_ptr<Node> start_node, shared_ptr<Node> end_node)
{
    std::cout << "start get path \n";
    // add start and end points to graph - connecting them to nearest
    deque<arcs> points;
    // init open list
    vector<shared_ptr<Node>> OPEN;
    // add start node on open list
    shared_ptr<Node> current = nullptr;
    OPEN.push_back(start_node);
    int its = 0;
    bool end_reached = false;
    while (OPEN.size() > 0)
    {
        its++; 
        auto cur_it = OPEN.begin();
        current = *cur_it;
        // always work on minimal cost node in open set
        for (auto it = OPEN.begin(); it != OPEN.end(); it++)
        {
            if ((*it)->cost < current->cost)
            { 
                cur_it = it;
                current = *cur_it;
            }
        }
        OPEN.erase(cur_it);
        
        if (current == end_node) // end reached
        {
            end_reached = true;

            break;
        } 
        // for all nodes connected to current
        for (auto i = 0; i < current->connected.size(); i++)
        {
            auto &con = current->connected[i];
            // cout << "Current conncetions: " << current->connected.size() << endl;
            if (!con.node->opened)
            {
                // Check if node is available at that time
                float length = current->cost + con.cost;
                float time_stamp = length;
                
                if (!con.node->check_availability(time_stamp, config->getTPRM_T()))
                {
                    continue; //skip to the next node connected to current
                }

                else
                {
                    // cout<< "connected.";
                    // if not opened, add to open, store that current is parent
                    // update their cost to current + dist between nodes 
                    con.node->parent = current; // ok
                    con.node->parent_connection = make_shared<connection>(con); // something wrong here/
                    // cout << "\n\n" << current->pt.x.x << " " << current->pt.x.y << " " << current->pt.theta << " current pose";
                    // cout << "\n" << con.node->parent_connection->A.a[0].start.x.x << " " << con.node->parent_connection->A.a[0].start.x.y << " " << con.node->parent_connection->A.a[0].start.theta << " arc start connection";
                    // cout << "\n" << con.node->parent_connection->A.a[2].end.x.x << " " << con.node->parent_connection->A.a[2].end.x.y << " " << con.node->parent_connection->A.a[2].end.theta << " arc end connection";
                    // cout << "\n" << con.node->pt.x.x << " " << con.node->pt.x.y << " " << con.node->pt.theta << " pt connection\n";
                    OPEN.push_back(con.node);
                    con.node->opened = true;
                    con.node->cost = length;
                }
                
            }
        }
    }
    cout << " searched: " << its << " \n";
    if (!end_reached) cout << "\n END NOT REACHED :( ";
    if (!current->parent || !end_reached) {
        std::cout << "oopsie no path \n";
        return points;
    }
    int count = 0;
    while (current->parent )
    {
        count++;
        points.push_front(current->parent_connection->A);

        // cout << "\n" << current->pt.x.x << " " << current->pt.x.y << " " << current->pt.theta << " pose in path";
        // Calculate time of arrival to the node and add info to the node 
        
        float node_time = current->cost;
        current->arrival_time.push_back(node_time);

        point2d current_point = current->pt.x;
        vector<shared_ptr<Node>> nearby = points_quad.in_range(current_point, config->getTPRM_D()); 
        for (auto i = 0; i < nearby.size() ; i++)
        {
            nearby[i]->arrival_time.push_back(node_time);     
        }
  
        current = current->parent;
    }
    // cout << "\n" << current->pt.x.x << " " << current->pt.x.y << " " << current->pt.theta << " should be start";

    // std::cout << "we made it this far! Nodes in path: " << count << endl;
    reset_nodes(); 
    
    current = nullptr;
    OPEN.clear();
    std::cout << "further! \n";
    return points;
};
deque<arcs> Graph::getPathPlusManyExits(shared_ptr<Node> start_node, vector<shared_ptr<Node>> end_nodes)
{
    std::cout << "start get path \n";
    // add start and end points to graph - connecting them to nearest
    deque<arcs> points;
    // init open list
    vector<shared_ptr<Node>> OPEN;
    // add start node on open list
    shared_ptr<Node> current = nullptr;
    OPEN.push_back(start_node);
    int its = 0;
    bool end_reached = false;
    while (OPEN.size() > 0)
    {
        its++; 
        auto cur_it = OPEN.begin();
        current = *cur_it;
        // always work on minimal cost node in open set
        for (auto it = OPEN.begin(); it != OPEN.end(); it++)
        {
            if ((*it)->cost < current->cost)
            { 
                cur_it = it;
                current = *cur_it;
            }
        }
        OPEN.erase(cur_it);
        for (int nn = 0; nn < end_nodes.size(); nn++)
        {
            if (current == end_nodes[nn]) // end reached
            {
                end_reached = true;
                break;
            } 
        }

        // for all nodes connected to current
        for (auto i = 0; i < current->connected.size(); i++)
        {
            auto &con = current->connected[i];
            // cout << "Current conncetions: " << current->connected.size() << endl;
            if (!con.node->opened)
            {
                // Check if node is available at that time
                float length = current->cost + con.cost;
                float time_stamp = length;
                
                if (!con.node->check_availability(time_stamp, config->getTPRM_T()))
                {
                    continue; //skip to the next node connected to current
                }

                else
                {
                    // cout<< "connected.";
                    // if not opened, add to open, store that current is parent
                    // update their cost to current + dist between nodes 
                    con.node->parent = current; // ok
                    con.node->parent_connection = make_shared<connection>(con); // something wrong here/
                    // cout << "\n\n" << current->pt.x.x << " " << current->pt.x.y << " " << current->pt.theta << " current pose";
                    // cout << "\n" << con.node->parent_connection->A.a[0].start.x.x << " " << con.node->parent_connection->A.a[0].start.x.y << " " << con.node->parent_connection->A.a[0].start.theta << " arc start connection";
                    // cout << "\n" << con.node->parent_connection->A.a[2].end.x.x << " " << con.node->parent_connection->A.a[2].end.x.y << " " << con.node->parent_connection->A.a[2].end.theta << " arc end connection";
                    // cout << "\n" << con.node->pt.x.x << " " << con.node->pt.x.y << " " << con.node->pt.theta << " pt connection\n";
                    OPEN.push_back(con.node);
                    con.node->opened = true;
                    con.node->cost = length;
                }
                
            }
        }
    }
    cout << " searched: " << its << " \n";
    if (!end_reached) cout << "\n END NOT REACHED :( ";
    if (!current->parent || !end_reached) {
        std::cout << "oopsie no path \n";
        return points;
    }
    int count = 0;
    while (current->parent )
    {
        count++;
        points.push_front(current->parent_connection->A);

        // cout << "\n" << current->pt.x.x << " " << current->pt.x.y << " " << current->pt.theta << " pose in path";
        // Calculate time of arrival to the node and add info to the node 
        
        float node_time = current->cost;
        current->arrival_time.push_back(node_time);

        point2d current_point = current->pt.x;
        vector<shared_ptr<Node>> nearby = points_quad.in_range(current_point, config->getTPRM_D()); 
        for (auto i = 0; i < nearby.size() ; i++)
        {
            nearby[i]->arrival_time.push_back(node_time);     
        }
  
        current = current->parent;
    }
    // cout << "\n" << current->pt.x.x << " " << current->pt.x.y << " " << current->pt.theta << " should be start";

    // std::cout << "we made it this far! Nodes in path: " << count << endl;
    reset_nodes(); 
    
    current = nullptr;
    OPEN.clear();
    std::cout << "further! \n";
    return points;
};
void Graph::print_nodes(){
    for (auto i = 0; i < nodes.size(); i++)
    {
        // cout << (nodes[i]->pt).x.x << " x ins y " << (nodes[i]->pt).x.y << "\n";
    }
}

