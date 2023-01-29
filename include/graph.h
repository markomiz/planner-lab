#pragma once
#include "geometry.h"
#include <deque>
#include "config_server.h"

using namespace std;

struct connection;
struct Bundle;


struct Node
{
    Node(pose2d point): opened(false), cost(0), parent(nullptr) {
        pose2d x = point;
        pt = x;
    };
    pose2d pt;
    vector<connection> connected;
    float cost;
    bool opened;
    shared_ptr<Node> parent;
    shared_ptr<connection> parent_connection;
    vector<float> arrival_time;
    shared_ptr<Node> opposite;
    // vector<shared_ptr<Node>> siblings;
    bool check_availability(float current_time, float threshold)
    {
        for (int i = 0; i < arrival_time.size(); i++)
        {
            if (current_time > arrival_time[i] - threshold  && current_time < arrival_time[i] + threshold)
            {
                return false; 
            }
        }
        return true;
    }    
};
struct Bundle
{
    vector<shared_ptr< Node>> nodes;
    point2d pos;
};

struct connection
{
    shared_ptr<Node> node;
    float cost;
    arcs A; // these go from the connected node to the node which owns this
};

class quad
{
    public:
    quad(float xmin, float xmax, float ymin, float ymax, int depth): xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax), depth(depth)
    {};
    void add_bundle(shared_ptr<Bundle> point);
    void subdivide();
    void add_bundle_to_children(shared_ptr<Bundle> point);
    vector<shared_ptr<Bundle>> get_nearest(point2d point, float r);
    void find_neighbors_r(point2d point, float r, vector<shared_ptr<Bundle>> &neighbors);
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    int depth;
    const int max_depth = 10;
    vector<shared_ptr<Bundle>> points;
    vector<quad> children;
    void print_nodes();
};


class Graph
{
    public:
        Graph(int max_depth, point2d tl, point2d br);
        // creates node for point adds, returns new node ptr
        shared_ptr<Node> add(shared_ptr<Node> pt, shared_ptr<Node> existing);
        shared_ptr<Node> add(shared_ptr<Node> pt, shared_ptr<Node> existing, arcs A);
        vector<shared_ptr<Bundle>> in_range(point2d pt, float rad);
        void reset_nodes();
        deque<point2d> getPath(shared_ptr<Node> start, shared_ptr<Node> end);
        deque<arcs> getPathPlus(shared_ptr<Node> start, shared_ptr<Node> end);
        deque<arcs> getPathPlusManyExits(shared_ptr<Node> start_node, vector<shared_ptr<Node>> end_nodes);
        quad points_quad;
        vector<shared_ptr<Node>> nodes;
        void print_nodes();
        shared_ptr<ConfigParams> config;

    private:
};

