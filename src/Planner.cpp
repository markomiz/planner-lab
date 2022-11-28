#include <iostream>
#include <string>
#include <bits/stdc++.h>
#include <random>
using namespace std;

class Planner
{
    public:
        Planner() {};
        ~Planner(){};
        point2d uniform_point(bool no_collision);
        void generate_roadmap(int num_nodes);
        vector<point2d> nodes;
        void generate_path(point2d start, point2d finish);
        bool check_point_collision(point2d point);

    private:
        random_device rd;
        mt19937 gen;
        uniform_real_distribution<> dist_x;
        uniform_real_distribution<> dist_y;
        void init_random(float min_x, float max_x, float min_y, float max_y);

}
// initialise random number generator
void Planner::init_random(float min_x, float max_x, float min_y, float max_y)
{
    this->rd = random_device();
    this->gen = mt19937(this->rd());
    this->dist_x = uniform_real_distribution<> (min_x, max_x);
    this->dist_y = uniform_real_distribution<> (min_y, max_y);
    
};

// Uniform non colliding sample
point2d Planner::uniform_point(bool no_collision)
{
    point2d p;
    p.x = this->dist_x(this->gen);
    p.y = this->dist_y(this->gen);
    if (no_collision)
    {
        while (check_point_collision(p))
        {
            p.x = this->dist_x(this->gen);
            p.y = this->dist_y(this->gen);
        }
    }

    return p;
};

bool  Planner::check_point_collision(point2d point)
{
    // TODO: use collision detection file

}
bool Planner::generate_roadmap(int num_nodes)
{
    for (int n = 0; n < num_nodes; n++)
    {
        point2d node = this->uniform_point(true);

    }
}


