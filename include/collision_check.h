#pragma once
#include <iostream>
#include <string>
#include <bits/stdc++.h>
#include "helpers.h"
#include "map.h"
using namespace std;


class CollisionCheck
{
    public:
        static intersection_result line_line_intersect(line l1, line l2);
        static intersection_result line_arc_intersect(line l1, arc arc1);
        static bool point_in_polygon(point2d p, Polygon shape);
        static bool arc_with_polygon(arc a, Polygon shape);
        static float point_lineseg_dist(point2d p, line l);
        static float distance(point2d p1, point2d p2);
        
};
