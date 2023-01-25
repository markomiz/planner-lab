#pragma once
#include "geometry.h"
#include "graph.h"
#include "map.h"
#include "dubinCurve.h"
#include "config_server.h"

using namespace std;

class PRMstar 
{
    public:
        PRMstar(shared_ptr<Map> m) : map(m){
            graph = shared_ptr<Graph> (new Graph(7, point2d(m->min_x, m->max_y), point2d(m->max_x, m->min_y)));
        }; 
        ~PRMstar(){
        };

        void genRoadmap(int n);
        void genRoadmapPlus(int n, int angles);
        vector<point2d> getPath(point2d start, point2d end);
        deque<arcs> getPath(pose2d start, pose2d end);
        deque<arcs> getPathManyExits(pose2d start, vector<pose2d> end);
        shared_ptr<dubinCurve> dCurve;
        shared_ptr<ConfigParams> config;
        shared_ptr<Graph> graph;
    private:
        
        shared_ptr<Map> map;



};
