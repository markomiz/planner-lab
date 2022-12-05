#pragma once
#include "geometry.h"

using namespace std;

struct connection;
struct Node
{
    Node(point2d pt): pt(pt), opened(false), cost(0), parent(nullptr) {};
    point2d pt;
    vector<connection> connected;
    float cost;
    bool opened;
    shared_ptr<Node> parent;
};
struct connection
{
    shared_ptr<Node> node;
    float cost;
};

class quad
{
    public:
        quad(int max_depth, point2d tl, point2d br);
        bool insert(shared_ptr<Node> node);
        vector<shared_ptr<Node>> in_range(point2d pt, float radius);
        bool overlaps(point2d pt, float radius);
        int max_depth;

    private:
        point2d tl;
        point2d br;
        point2d tr;
        point2d bl;
        vector<shared_ptr<Node>> nodes;
        quad* tl_tree;
        quad* tr_tree;
        quad* bl_tree;
        quad* br_tree;
        point2d center;
        float radius;

};

class Graph
{
    public:
        Graph(int max_depth, point2d tl, point2d br);
        // creates node for point adds, returns new node ptr
        shared_ptr<Node> add(shared_ptr<Node> pt, shared_ptr<Node> existing);
        vector<shared_ptr<Node>> in_range(point2d pt, float rad);
        void reset_nodes();
        vector<point2d> getPath(shared_ptr<Node> start, shared_ptr<Node> end);
        quad points_quad;
        vector<shared_ptr<Node>> nodes;
        void print_nodes();

    private:


};