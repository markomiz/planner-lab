#pragma once
#include "helpers.h"

class Polygon
{
    public:
        Polygon(std::vector<point2d> v): verteces(v)
        {
            processEdges();
            calculateCenter();
        }

    private:
        void calculateCenter();
        void processEdges();
        void expandShape(float size);
        point2d center;
        float radius;
        std::vector<point2d> verteces;
        std::vector<line> edges;

};

class Map
{
    public:
        Map();
        ~Map();

        void addObstacle(Polygon shape);
        bool uncolliding(poin2d point);
        bool uncolliding(arc a);
        bool uncolliding(line l);
        
    private:
        std::vector<Polygon> obstacles;
        

};