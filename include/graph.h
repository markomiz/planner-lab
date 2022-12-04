#pragma once
#include "helpers.h"

using namespace std;

struct connection
{
    Node* node;
    float cost;
}
struct Node
{
    Node(Point2d &pt): pt(pt), opened(false), cost(0), parent(nullptr) {};
    point2d* pt;
    vector<connection> connected;
    float cost;
    bool opened;
    Node* parent;
}

class quad
{
    public:
        quad(point2d tl, point2d br);
        bool insert(Node* node);
        vector<Node*> in_range(point2d pt, float radius);
        bool overlaps(point2d pt, float radius);

        int max_depth;

    private:
        point2d tl;
        point2d br;
        point2d tr;
        point2d bl;
        vector<Node*> nodes;
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
        // creates node for point adds, returns new node ptr
        Node* add(point2d pt, Node  *existing);
        vector<point2d> in_range(point2d pt, float rad);
        void reset_nodes();


    private:
        quad points_quad;
        vector<Node*> nodes;

}