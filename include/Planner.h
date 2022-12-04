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
        Planner(Map *m) : map(m)  {};
        ~Planner(){};

        point2d uniform_point();

    protected:

        float dist_thr;
        Map *map;
        

};