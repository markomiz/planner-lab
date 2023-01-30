#pragma once
#include "geometry.h"
#include "graph.h"
#include "map.h"
#include "dubinCurve.h"
#include "config_server.h"

using namespace std;

class Planner 
{
    public:
        explicit Planner(shared_ptr<Map> m) : map(m){
            graph = shared_ptr<Graph> (new Graph(15, point2d(m->min_x, m->max_y), point2d(m->max_x, m->min_y)));
        };
        ~Planner(){
        };

        vector<point2d> getPath(point2d start, point2d end);

        shared_ptr<dubinCurve> dCurve;
        shared_ptr<ConfigParams> config;
        shared_ptr<Graph> graph;
        shared_ptr<Map> map;
};

class GeometricPRMstar : public Planner
{
    public:
        explicit GeometricPRMstar(shared_ptr<Map> m) : Planner(m){};
        // Geometric PRM 
        void genRoadmap(int n);
        // Geometric Dijkstra
};

class DubinsPRMstar : public Planner
{
    public:
        explicit DubinsPRMstar(shared_ptr<Map> m) : Planner(m){};
        // Dubins PRM 
        void genRoadmap(int n, int angles);
        // Dubins Dijkstra
        deque<arcs> getPath(pose2d start, pose2d end);
        // Dubins Dijkstra + Multiple exit possibilities
        deque<arcs> getPathManyExits(pose2d start, vector<pose2d> end);
};

class ExactCell : public Planner
{
    public:
        explicit ExactCell(shared_ptr<Map> m) : Planner(m){};
        // Exact Cell Decomposition
        void genRoadmap();
    private:
        bool comparePoints(exactpoint2d p1, exactpoint2d p2);
        void quickSort(vector<exactpoint2d> &points, int low, int high);
};