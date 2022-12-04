#pragma once
#include "Planner.h"
#include "helpers.h"
#include "graph.h"

using namespace std;

class PRMstar : public Planner
{
    public:
        PRMstar(Map* m) : Planner(m){}; 
        ~PRMstar();

        void genRoadmap(int n);
        vector<point2d> getPath(point2d start, point2d end);

    private:
        Graph *graph;



};
