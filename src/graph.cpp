#pragma once
#include "helpers.h"

using namespace std;

quad::quad(int max_depth, point2d tl, point2d br) : tl(tl), br(br), max_depth(max_depth)
{
    tl_tree = NULL;
    bl_tree = NULL;
    tr_tree = NULL;
    br_tree = NULL;
}

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
bool quad::overlaps(point2d pt, float radius)
{
    if ((bl - pt).norm() < radius) return true;
    if ((tr - pt).norm() < radius) return true;
    if ((tl - pt).norm() < radius) return true;
    if ((br - pt).norm() < radius) return true;
    return false;
}

vector<point2d*> quad::in_range(point2d pt, float radius)
{
    vector<point2d*> all;
    // TODO

    for (int i = 0; i < nodes.size(); i++)
    {
        if ((nodes[i] - pt).norm() < radius) all.push_back(nodes[i]);
    }
    // otherwise, choose which subtrees within range,
    if (tl_tree)
    {
        if (tl_tree->overlaps(pt,radius))
        {
            vector<point2d*> tl_all = tl_tree->in_range(pt, radius);
            all.insert(all.end(), tl_all.begin(), tl_all.end());
        }
    }
    if (bl_tree)
    {
        if (bl_tree->overlaps(pt,radius))
        {
            vector<point2d*> bl_all = bl_tree->in_range(pt, radius);
            all.insert(all.end(), bl_all.begin(), bl_all.end());
        }
    }
    if (tr_tree)
    {
        if (tr_tree->overlaps(pt,radius))
        {
            vector<point2d*> tr_all = tr_tree->in_range(pt, radius);
            all.insert(all.end(), tr_all.begin(), tr_all.end());
        }
    }
    if (br_tree)
    {
        if (br_tree->overlaps(pt,radius))
        {
            vector<point2d*> br_all = br_tree->in_range(pt, radius);
            all.insert(all.end(), br_all.begin(), br_all.end());
        }
    }
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
vector<point2d*> Graph::in_range(point2d pt, float rad)
{
    return graph->in_range(point2d pt, float rad);
};


