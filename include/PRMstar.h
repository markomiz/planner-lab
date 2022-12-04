#pragma once
#include "helpers.h"
#include "graph.h"
#include "map.h"

using namespace std;

class PRMstar 
{
    public:
        PRMstar(Map* m) : map(m){
            graph = new Graph(7, point2d(m->min_x, m->max_y), point2d(m->max_x, m->min_y));
        }; 
        ~PRMstar();

        void genRoadmap(int n);
        vector<point2d> getPath(point2d start, point2d end);

    private:
        Graph *graph;
        Map* map;



};
