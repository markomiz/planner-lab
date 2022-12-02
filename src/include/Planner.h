#include <iostream>
#include <string>
#include <bits/stdc++.h>
#include <random>


class Planner
{
    public:
        Planner() {};
        ~Planner(){};

        point2d uniform_point(float min_x, float max_x, float min_y, float max_y);

        virtual vector<point2d> get_path(point2d start, point2d finish);

        point2d get_start() {return start;};
        point2d get_goal() {return goal;};

        void set_start(point2d s) {start = s;};
        void set_goal(point2d g) {goal = g;};

    protected:
        point2d start;
        point2d goal;

};