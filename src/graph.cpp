#pragma once
#include "helpers.h"
#include "graph.h"
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
    nodes;
}

bool quad::insert(Node* node)
{
    if (max_depth == 0) {
        nodes.push_back(node);
        return true;
    }

    if ((tl.x + br.x) / 2 >= node->pt->x) {
        // Indicates tl_tree
        if ((tl.y + br.y) / 2 >= node->pt->y) {
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
        if ((tl.y + br.y) / 2 >= node->pt->y) {
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
bool quad::overlaps(point2d pt, float radius)
{
    if ((bl - pt).norm() < radius) return true;
    if ((tr - pt).norm() < radius) return true;
    if ((tl - pt).norm() < radius) return true;
    if ((br - pt).norm() < radius) return true;
    return false;
}

vector<Node*> quad::in_range(point2d pt, float radius)
{
    vector<Node*> all;
    for (int i = 0; i < nodes.size(); i++)
    {
        if ((*(nodes[i]->pt) - pt).norm() < radius) all.push_back(nodes[i]);
    }
    // otherwise, choose which subtrees within range,
    if (tl_tree)
    {
        if (tl_tree->overlaps(pt,radius))
        {
            vector<Node*> tl_all = tl_tree->in_range(pt, radius);
            all.insert(all.end(), tl_all.begin(), tl_all.end());
        }
    }
    if (bl_tree)
    {
        if (bl_tree->overlaps(pt,radius))
        {
            vector<Node*> bl_all = bl_tree->in_range(pt, radius);
            all.insert(all.end(), bl_all.begin(), bl_all.end());
        }
    }
    if (tr_tree)
    {
        if (tr_tree->overlaps(pt,radius))
        {
            vector<Node*> tr_all = tr_tree->in_range(pt, radius);
            all.insert(all.end(), tr_all.begin(), tr_all.end());
        }
    }
    if (br_tree)
    {
        if (br_tree->overlaps(pt,radius))
        {
            vector<Node*> br_all = br_tree->in_range(pt, radius);
            all.insert(all.end(), br_all.begin(), br_all.end());
        }
    }

    return all;
};
Node* Graph::add(Node* point, Node  *existing)
{
    
    float dist = (*point->pt - *existing->pt).norm();
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
vector<Node*> Graph::in_range(point2d pt, float rad)
{
    return points_quad.in_range( pt, rad);
};
void Graph::reset_nodes()
{
    for (int i = 0; i < nodes.size(); i++)
    {
        nodes[i]->parent = nullptr;
        nodes[i]->cost = 0;
        nodes[i]->opened = false;
    }
}
vector<point2d> Graph::getPath(Node* start, Node* end)
{
    // add start and end points to graph - connecting them to nearest TODO
    vector<point2d> points;
    // init open list
    vector<Node*> OPEN;
    // add start node on open list
    Node* current = nullptr;
    OPEN.push_back(start);
    while (OPEN.size() > 0)
    {
        current = OPEN[0];
        auto cur_it = OPEN.begin();
        // always work on minimal cost node in open set
        for (auto it = OPEN.begin(); it != OPEN.end(); it++)
        {
            if ((*it)->cost < current->cost)
            {
                current = *it;
                cur_it = it;
            }
        }

        if (current == end) break;

        // for all nodes connected to current
        for (int i = 0; i < current->connected.size(); i++)
        {
            if ( ! current->connected[i].node->opened )
            {
                // if not opened, add to open, store that current is parent
                // update their cost to current + dist between nodes
                current->connected[i].node->parent = current;
                OPEN.push_back(current->connected[i].node);
                current->connected[i].node->opened = true;
                current->connected[i].node->cost = current->cost + current->connected[i].cost;
            }
        }

        OPEN.erase(cur_it);
    }

    while (current->parent != start)
    {
        points.push_back(*current->pt);
        current = current->parent;
    }
    reset_nodes();
    return points;
}