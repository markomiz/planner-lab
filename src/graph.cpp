#pragma once
#include "helpers.h"

using namespace std;

bool quad::insert(point2d* node)
{
    if (max_depth == 0) {
        nodes.push_back(node);
        return true
    }

    if ((tl.x + br.x) / 2 >= node->x) {
        // Indicates tl_tree
        if ((tl.y + br.y) / 2 >= node->y) {
            if (tl_tree == NULL)
                tl_tree = new Quad( max_depth - 1,
                    point2d(tl.x, tl.y),
                    point2d((tl.x + br.x) / 2,
                          (tl.y + br.y) / 2));
            tl_tree->insert(node);
        }
        else {
            if (bl_tree == NULL)
                bl_tree = new Quad( max_depth - 1,
                    point2d(tl.x,
                          (tl.y + br.y) / 2),
                    point2d((tl.x + br.x) / 2,
                          br.y));
            bl_tree->insert(node);
        }
    }
    else {
        if ((tl.y + br.y) / 2 >= node->y) {
            if (tr_tree == NULL)
                tr_tree = new Quad( max_depth - 1,
                    point2d((tl.x + br.x) / 2,
                          tl.y),
                    point2d(br.x,
                          (tl.y + br.y) / 2));
            tr_tree->insert(node);
        }
        else {
            if (br_tree == NULL)
                br_tree = new Quad( max_depth - 1,
                    point2d((tl.x + br.x) / 2,
                          (tl.y + br.y) / 2),
                    point2d(br.x, br.y));
            br_tree->insert(node);
        }
    }
};


vector<point2d*> quad::in_range(point2d pt, float radius)
{
    vector<point2d*> all;
    // TODO
    // if no nodes or subtrees, return 

    // otherwise, choose which subtree is within range,
    // get in range from that
    // add it to all

    return all;
};

void Graph::add(edge* e)
{
    edges.push_back(e);
}

void Graph::add(point2d* pt, point2d  *existing)
{
    edge *e = new edge(pt, existing);
    add(e);

};
vector<point2d*> Graph::nearest(point2d pt, float rad)
{
    return graph->nearest(point2d pt, float rad);
};


