#pragma once
#include "geometry.h"
#include "graph.h"
#include <iostream>
#include <deque>
using namespace std;
Graph::Graph(int max_depth, point2d tl, point2d br) : points_quad(-6.0, 6.0, -6.0, 6.0, 0)
{

};


void quad::add_bundle(shared_ptr<Bundle> point){

    //cout << " add node" << endl;
    if (children.size() == 0)
    {
        points.push_back(point);
        if (points.size() > 1 && depth < max_depth) {
            subdivide();
        }
    }
    else {
        add_bundle_to_children(point);
    }
};
void quad::subdivide(){
    float xmid =  (xmin + xmax) / 2;
    float ymid = (ymin + ymax) / 2;
    children.push_back(quad(xmin, xmid, ymin, ymid, depth + 1));
    children.push_back(quad(xmid, xmax, ymin, ymid, depth + 1));
    children.push_back(quad(xmin, xmid, ymid, ymax, depth + 1));
    children.push_back(quad(xmid, xmax, ymid, ymax, depth + 1));

    for (auto i = 0; i < points.size(); i++)
    {
        add_bundle_to_children(points[i]);
    }            
    points.clear();
};
void quad::add_bundle_to_children(shared_ptr<Bundle> point) {
    float x = point->pos.x;
    float y = point->pos.y;
    
    for (auto i = 0; i < children.size(); i++)
    {    
        if (children[i].xmin <= x && x <= children[i].xmax && children[i].ymin <= y && y <= children[i].ymax)
        {
            // cout << " x " << x << " y " << y << endl;
            children[i].add_bundle(point);
            break;
        }
    }
};
vector<shared_ptr<Bundle>> quad::get_nearest(point2d pt, float r)
{
    vector<shared_ptr<Bundle>> neighbors;
    find_neighbors_r(pt, r, neighbors);
    return neighbors;
};
void quad::find_neighbors_r(point2d point, float r, vector<shared_ptr<Bundle>> &neighbors)
{
    float x = point.x;
    float y = point.y;
    
    if (children.size() == 0){
        if (xmin <= x - r && x + r <= xmax && ymin <= y - r && y + r <= ymax) // whole quad is inside
            {
                neighbors.insert(neighbors.end(), points.begin(), points.end());
            }
        else
        {
            for (auto i = 0; i < points.size(); i++ )
            {
                point2d p = (point - points[i]->pos);
                if (p.x*p.x + p.y*p.y <= r*r)
                {
                    neighbors.push_back(points[i]);
                }   
            }
        }

    }
    else{
        for (auto i = 0; i < children.size(); i++ )
        {
            if (children[i].xmin <= x + r && x - r <= children[i].xmax && children[i].ymin <= y + r && y - r <= children[i].ymax) // quad partially inside 
            {
                children[i].find_neighbors_r(point, r, neighbors);
            }
        }
    }        
};
shared_ptr<Node> Graph::add(shared_ptr<Node> point, shared_ptr<Node> existing)
{
    
    point2d l = (point->pt.x - existing->pt.x);
    float sqdist = l.x*l.x + l.y+l.y;
    connection c1;
    c1.node = existing;
    c1.cost = sqdist;
    point->connected.push_back(c1);
    connection c2;
    c2.node = point;
    c2.cost = sqdist;
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
vector<shared_ptr<Bundle>> Graph::in_range(point2d pt, float rad)
{

    vector<shared_ptr<Bundle>> points = points_quad.get_nearest( pt, rad);

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
    start->cost = 0.0;
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
        float dist = (current->pt.x - end_node->pt.x).norm();
        if (dist < 0.2) // end reached
        // if (current == end_node)
        {
            end_reached = true;
            break;
        } 
        // for all nodes connected to current
        for (auto i = 0; i < current->connected.size(); i++)
        {
            auto &con = current->connected[i];
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
                   con.node->parent = current; // ok
                    con.node->parent_connection = make_shared<connection>(con); // something wrong here/
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
        float node_time = current->cost;
        current->arrival_time.push_back(node_time);
        point2d current_point = current->pt.x;
        vector<shared_ptr<Bundle>> nearby = points_quad.get_nearest(current_point, config->getTPRM_D()); 
        for (auto i = 0; i < nearby.size() ; i++)
        {
            for (auto a = 0; a < nearby[i]->nodes.size(); a++)
            {
               nearby[i]->nodes[a]->arrival_time.push_back(node_time);  
            }
                
        }
        current = current->parent;
    }

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
        
        for (auto nn = 0; nn < end_nodes.size(); nn++)
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
                    con.node->parent = current; // ok
                    con.node->parent_connection = make_shared<connection>(con); // something wrong here/
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
        float node_time = current->cost;
        current->arrival_time.push_back(node_time);

        point2d current_point = current->pt.x;
        vector<shared_ptr<Bundle>> nearby = points_quad.get_nearest(current_point, config->getTPRM_D()); 
        for (auto i = 0; i < nearby.size() ; i++)
        {
            for (auto a = 0; a < nearby[i]->nodes.size(); a++)
            {
               nearby[i]->nodes[a]->arrival_time.push_back(node_time);  
            }
                
        }
  
        current = current->parent;
    }

    reset_nodes(); 
    
    current = nullptr;
    OPEN.clear();
    std::cout << "further! \n";
    return points;
};
void Graph::print_nodes(){
    for (auto i = 0; i < nodes.size(); i++)
    {
        cout << (nodes[i]->pt).x.x << " x ins y " << (nodes[i]->pt).x.y << "\n";
    }
};
void quad::print_nodes(){
    // if (children.size() == 0 )
    // {
    //     for (auto i = 0; i < points.size(); i++)
    //     {
    //         cout << (points[i]->pt).x.x << " x ins y " << (points[i]->pt).x.y << "\n";
    //     }
    // }
    // else
    // {
    //     for (auto i = 0; i < children.size(); i++)
    //     {
    //         children[i].print_nodes();
    //     }
    // }
};
