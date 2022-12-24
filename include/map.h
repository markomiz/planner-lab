#pragma once
#include "geometry.h"
#include <memory>


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
        // Map(float min_x ,float min_y ,float max_x ,float max_y)
        // : min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y), halton_index(0){ processBounds();};

        void addObstacle(Polygon shape); //NOT DONE
        bool colliding(point2d point); //DONE: MAP.CPP
        bool colliding(arc a); //DONE: MAP.CPP
        bool colliding(line l); //DONE: MAP.CPP
        bool colliding(arcs A); //DONE: MAP.CPP
        void createMap(Polygon map);

        bool inBounds(point2d point);
        float min_x;
        float min_y;
        float max_x;
        float max_y;

        void processBounds();
        float getFreeSpace() { return freeSpace;};
        point2d uniform_sample();
        float halton_min(int index, int base, float min, float max);
        point2d halton_sample();

    private:
        int halton_index;
        float freeSpace;
        std::vector<Polygon> obstacles;
        Polygon total_map_poly;
        
        

};