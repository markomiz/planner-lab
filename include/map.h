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

        bool inBounds(point2s point);

        float min_x;
        float min_y;
        float max_x;
        float max_y;

        void processBounds();
        
    private:
        std::vector<Polygon> obstacles;
        std::vector<line> bounds;
        
        

};