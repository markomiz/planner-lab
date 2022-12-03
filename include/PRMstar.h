#pragma once
#include "Planner.h"
#include "helpers.h"
#include "graph.h"

using namespace std;

class PRMstar : public Planner
{
    public:
        PRMstar() : Planner(){}; 
        ~PRMstar();

        void genRoadmap(int n);
        vector<point2d> getPath(point2d start, pointe2d end);

    private:
        Graph *graph;



};
