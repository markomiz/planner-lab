#pragma once
#include <iostream>
#include <string>
#include <bits/stdc++.h>
#include <random>
#include <boost/graph/adjacency_list.hpp>
#include "helpers.h"
#include "map.h"

class Planner
{
    public:
        Planner(Map m) : map(m)  {};
        ~Planner(){};

        point2d uniform_point();

        virtual vector<point2d> get_path(point2d start, point2d finish);

        point2d get_start() {return start;};
        point2d get_goal() {return goal;};

        void set_start(point2d s) {start = s;};
        void set_goal(point2d g) {goal = g;};

        void add_edge(Vertex &u, Vertex &v, Tree& tree);
        void add_vertex(Vertex &u, Tree& tree);

        bool close_enough(point2d a, point2d b); 

    protected:
        point2d start;
        point2d goal;

        float dist_thr;
        Map *map;
        

};