#pragma once
#include "geometry.h"

class Polygon
{
    public:
        Polygon(std::vector<point2d> v): verteces(v)
        {
            processEdges();
            calculateCenter();
            calculateArea();
        }
        void calculateArea();
        void calculateCenter();
        void processEdges();
        void expandShape(float size);
        point2d center;
        float radius;
        float area;
        std::vector<point2d> verteces;
        std::vector<line> edges;

};

class Map
{
    public:
        Map();
        Map(float min_x ,float min_y ,float max_x ,float max_y)
        : min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y){ processBounds();};

        void addObstacle(Polygon shape);
        bool colliding(point2d point);
        bool colliding(arc a);
        bool colliding(line l);

        bool inBounds(point2d point);

        float min_x;
        float min_y;
        float max_x;
        float max_y;

        void processBounds();
        float getFreeSpace() { return freeSpace;};
        point2d uniform_sample();
    private:
        float freeSpace;
        std::vector<Polygon> obstacles;
        std::vector<line> bounds;
        
        

};