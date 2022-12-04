#pragma once
#include "helpers.h"

using namespace std;

struct connection;
struct Node
{
    Node(point2d *pt): pt(pt), opened(false), cost(0), parent(nullptr) {};
    point2d* pt;
    vector<connection> connected;
    float cost;
    bool opened;
    Node* parent;
};
struct connection
{
    Node* node;
    float cost;
};

class quad
{
    public:
        quad(int max_depth, point2d tl, point2d br);
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

};

class Graph
{
    public:
        Graph(int max_depth, point2d tl, point2d br);
        ~Graph();
        // creates node for point adds, returns new node ptr
        Node* add(Node* pt, Node  *existing);
        vector<Node*> in_range(point2d pt, float rad);
        void reset_nodes();
        vector<point2d> getPath(Node* start, Node* end);
        quad points_quad;
        vector<Node*> nodes;

    private:


};