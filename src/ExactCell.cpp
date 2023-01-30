#include "Planner.h"
#include "collision_check.h"
#include <iostream>
#include <algorithm>
#include <vector>

void ExactCell::genRoadmap(int not_used, int angles_not_used)
{
    vector<exactpoint2d> points;
    vector<Polygon> obstacles = map->getObstacles();
    for (int i = 0; i < obstacles.size(); i++)
    {
        for (int j = 0; j < obstacles[i].verteces.size(); j++)
        {
            exactpoint2d temp(obstacles[i].verteces[j].x, obstacles[i].verteces[j].y, i); 
            points.push_back(temp);
        }   
    }

    int n = points.size();

    quickSort(points, 0, n - 1);

    double maxy = 10;
    double miny = -10;
    double maxx = 10;
    double minx = -10;

    vector<point2d> nodes;
    point2d p(minx, miny + (maxy-miny)/2);
    
    nodes.push_back(p);

    // Line sweep to draw vertical lines from obstacle vertices

    for (int i = 0; i < n; i++)
    {
        // Create line downwards 
        point2d temp_down(points[i].x, miny);
        point2d temp_pt(points[i].x, points[i].y);
        line l_down(temp_pt, temp_down);
        int self_intersections = 0;
        for(int j = 0; j < obstacles[points[i].poly_id].edges.size(); j++)
        {
            if (CollisionCheck::line_line_intersect(obstacles[points[i].poly_id].edges[j], l_down).intersects) self_intersections++;
        }
        if (self_intersections <= 2)
        {
            float dist = __FLT_MAX__;
            point2d node(points[i].x, points[i].y);
            for (int j = 0; j < obstacles.size(); j++)
            {
                if (points[i].poly_id == j) continue;
                for (int edge_num = 0; edge_num < obstacles[j].edges.size(); edge_num++)
                {
                    if (CollisionCheck::line_line_intersect(obstacles[j].edges[j], l_down).intersects)
                    {
                        double new_dist = (CollisionCheck::line_line_intersect(obstacles[j].edges[j], l_down).intersection - temp_pt).norm();
                        if (new_dist < dist) 
                        {
                            dist = new_dist;
                        }
                    }
                }
                
            }
            if (dist == __FLT_MAX__) node.y -= (node.y-miny)/2;
            else node.y -= dist;
            nodes.push_back(node);

        }
        
        // Create line upwards
        point2d temp_up(points[i].x, maxy);
        line l_up(temp_pt, temp_up);
        self_intersections = 0;
        for(int j = 0; j < obstacles[points[i].poly_id].edges.size(); j++)
        {
            if (CollisionCheck::line_line_intersect(obstacles[points[i].poly_id].edges[j], l_up).intersects) self_intersections++;
        }
        if (self_intersections <= 2)
        {
            float dist = __FLT_MAX__;
            point2d node(points[i].x, points[i].y);
            for (int j = 0; j < obstacles.size(); j++)
            {
                if (points[i].poly_id == j) continue;
                for (int edge_num = 0; edge_num < obstacles[j].edges.size(); edge_num++)
                {
                    if (CollisionCheck::line_line_intersect(obstacles[j].edges[j], l_up).intersects)
                    {
                        double new_dist = (CollisionCheck::line_line_intersect(obstacles[j].edges[j], l_up).intersection - temp_pt).norm();
                        if (new_dist < dist) 
                        {
                            dist = new_dist;
                        }
                    }
                }
                
            }
            if (dist == __FLT_MAX__) node.y += (node.y-miny)/2;
            else node.y += dist;
            nodes.push_back(node);
        }
    }


    point2d p2(maxx, miny + (maxy-miny)/2);
    nodes.push_back(p2);

    // THIS NEEDS REDOING!!!
    
    // ofstream node_file ("nodes.txt");
    // int cons = 0;
    // for (auto i = 0; i < nodes.size(); i++)
    // {
    //     pose2d new_pose(nodes[i].x, nodes[i].y, 0);
    //     shared_ptr<Node> new_node(new Node(new_pose));
    //     for (auto j = 0; j < i; j++)
    //     {
    //         pose2d near_pose(nodes[j].x, nodes[j].y, 0);
    //         shared_ptr<Node> near_node(new Node(near_pose));
    //         // one way
    //         line l(new_pose.x, near_pose.x);
    //         // float dist = (new_node->pt.x - near_node->pt.x).norm();
    //         if (!map->colliding(l) /* &&  A.L < dist * M_PI/2 */){
    //             graph->add(new_node, near_node);
    //             cons ++;
    //         }
    //     }    
    // }
    // node_file.close();
    // cout << cons <<" connections test \n";
};

bool ExactCell::comparePoints(exactpoint2d p1, exactpoint2d p2) {
    if (p1.x == p2.x) {
        return (p1.y < p2.y);
    }
    return (p1.x < p2.x);
}

void ExactCell::quickSort(vector<exactpoint2d> &points, int low, int high) {
    if (low < high) {
        int pivotIndex = (low + high) / 2;
        exactpoint2d pivot = points[pivotIndex];

        int i = low;
        int j = high;

        while (i <= j) {
            while (comparePoints(points[i], pivot)) {
                i++;
            }
            while (comparePoints(pivot, points[j])) {
                j--;
            }
            if (i <= j) {
                swap(points[i], points[j]);
                i++;
                j--;
            }
        }

        quickSort(points, low, j);
        quickSort(points, i, high);
    }
}
