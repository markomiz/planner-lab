#include "Planner.h"
#include "collision_check.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <deque>

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

    for(int i = 1; i < n; i++)
    {
        if (points[i].y < points[i-1].y && points[i].x == points[i-1].x)
        {
            swap(points[i], points[i-1]);
        } 
    }

    double maxy = map->max_y;
    double miny = map->min_y;
    double maxx = map->max_x;
    double minx = map->min_x;

    vector<point2d> roadmap_points;
    point2d p(minx + (points[0].x - minx)/2, miny + (maxy-miny)/2);


    // roadmap_points.push_back(p);

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
        if (self_intersections < 2)
        {
            // cout << "First check: " << points[i].x << "; " << points[i].y << endl;
            float dist = __FLT_MAX__;
            point2d node(points[i].x, points[i].y);
            for (int j = 0; j < obstacles.size(); j++)
            {
                if (points[i].poly_id == j) continue;
                for (int edge_num = 0; edge_num < obstacles[j].edges.size(); edge_num++)
                {
                    if (CollisionCheck::line_line_intersect(obstacles[j].edges[edge_num], l_down).intersects)
                    {
                        // cout << "Inter check: " << CollisionCheck::line_line_intersect(obstacles[j].edges[edge_num], l_down).intersection.x << "; " << CollisionCheck::line_line_intersect(obstacles[j].edges[edge_num], l_down).intersection.y << endl;
                        double new_dist = (CollisionCheck::line_line_intersect(obstacles[j].edges[edge_num], l_down).intersection - temp_pt).norm();
                        if (new_dist < dist) 
                        {
                            dist = new_dist;
                        }
                    }
                }
                
            }
            if (l_down.length() == 0) {}
            else if (dist == __FLT_MAX__)
            {
                node.y -= (node.y-miny)/2;
                // cout << "did the math"<< node.x << ";"<<node.y<<endl;;
            }
            else node.y -= dist/2;
            roadmap_points.push_back(node);
        }
        
        // Create line upwards
        point2d temp_up(points[i].x, maxy);
        line l_up(temp_pt, temp_up);
        self_intersections = 0;
        for(int j = 0; j < obstacles[points[i].poly_id].edges.size(); j++)
        {
            if (CollisionCheck::line_line_intersect(obstacles[points[i].poly_id].edges[j], l_up).intersects) self_intersections++;
        }
        if (self_intersections < 2)
        {
            // cout << "Second check: " << points[i].x << "; " << points[i].y << endl;
            float dist = __FLT_MAX__;
            point2d node(points[i].x, points[i].y);
            for (int j = 0; j < obstacles.size(); j++)
            {
                if (points[i].poly_id == j) continue;
                for (int edge_num = 0; edge_num < obstacles[j].edges.size(); edge_num++)
                {
                    if (CollisionCheck::line_line_intersect(obstacles[j].edges[edge_num], l_up).intersects)
                    {
                        // cout << "Inter check: " << CollisionCheck::line_line_intersect(obstacles[j].edges[edge_num], l_up).intersection.x << "; " << CollisionCheck::line_line_intersect(obstacles[j].edges[edge_num], l_up).intersection.y << endl;
                        double new_dist = (CollisionCheck::line_line_intersect(obstacles[j].edges[edge_num], l_up).intersection - temp_pt).norm();
                        if (new_dist < dist) 
                        {
                            dist = new_dist;
                        }
                    }
                }
                
            }
            if (l_up.length() == 0) {}
            else if (dist == __FLT_MAX__) node.y += (maxy - node.y)/2;
            else node.y += dist/2;
            roadmap_points.push_back(node);
        }
    }


    point2d p2(maxx - (maxx - points[n-1].x)/2, miny + (maxy-miny)/2);
    // roadmap_points.push_back(p2);

    
    ofstream node_file ("nodes.txt");
    n_connections = 0;

    for (auto i = 0; i < roadmap_points.size(); i++)
    {
        shared_ptr<Bundle> new_bundle(new Bundle());
        pose2d new_pose(roadmap_points[i].x, roadmap_points[i].y, 0);
        new_bundle->pos = new_pose.x;
        shared_ptr<Node> new_node(new Node(new_pose));
        new_bundle->nodes.push_back(new_node);
        
        graph->nodes.push_back(new_node);
        cout << "The node " << roadmap_points[i].x << "; " << roadmap_points[i].y << " is connected to: " << endl;
        for (auto j = 0; j < i; j++)
        {
            // one way
            line l(new_pose.x, graph->nodes[j]->pt.x);
            if (!map->colliding(l))
            {
                cout << graph->nodes[j]->pt.x.x << "; " << graph->nodes[j]->pt.x.y << endl;
                graph->add(new_node, graph->nodes[j]);
                n_connections++;
            } 
            // else cout << "collided" << endl;
        }
        graph->points_quad.add_bundle(new_bundle);
    }
    node_file.close();
    cout << n_connections <<" connections test \n";
};

int ExactCell::partition(vector<exactpoint2d> &points, int low, int high) {
    double pivot = points[high].x;
    int i = low - 1;

    for (int j = low; j <= high - 1; j++) {
        if (points[j].x <= pivot) {
            i++;
            swap(points[i], points[j]);
        }
    }
    swap(points[i + 1], points[high]);
    return (i + 1);
}

void ExactCell::quickSort(vector<exactpoint2d> &points, int low, int high) {
    if (low < high) {
        int pi = partition(points, low, high);

        quickSort(points, low, pi - 1);
        quickSort(points, pi + 1, high);
    }
}
