#pragma once
#include "helpers.h"

using namespace std;

struct edge
{
    Node* a;
    Node* b;
    float dist;
}
struct Node
{
    point2d* pt;
    vector<edge*> edges;
}

class quad
{
    public:

        bool insert(point2d* node);
        bool isRoot(){return root;};
        bool withinRange(point2d p , float dist);
        vector<point2d*> nearest(float radius);

        int max_depth;

    private:
        bool root;
        point2d tl;
        point2d br;
        vector<point2d*> nodes;
        quad* tl_tree;
        quad* tr_tree;
        quad* bl_tree;
        quad* br_tree;

}

class Graph
{
    public:
        Graph();
        ~Graph();

        void add(point2d pt, point2d  *existing);
        void add(edge* e);
        vector<point2d> nearest(point2d pt, float rad);


    private:
        quad points_quad;
        vector<edge*> edges;
        vector<Node*> nodes;

}