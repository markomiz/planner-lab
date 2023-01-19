#pragma once
#include "geometry.h"
#include <memory>

class Map;

class Polygon
{
    public:
        Polygon(){};

        Polygon(std::vector<point2d> v): verteces(v)
        {
            processEdges();
            getMinMax();
            calculateCenter();
            calculateArea();
        }
        void calculateArea();
        void calculateCenter();
        void processEdges();
        void expandShape(float size);
        void getMinMax();
        // Map toMap();
        point2d center;
        float radius;
        float area = 0;
        float x_min;
        float x_max;
        float y_min;
        float y_max;
        std::vector<point2d> verteces;
        std::vector<line> edges;
};

class Map
{
    private:
        int halton_index;
        float freeSpace;
        std::vector<Polygon> obstacles;
        Polygon total_map_poly;

    public:
        Map(Polygon map): total_map_poly(map.verteces), freeSpace(map.area), halton_index(0){
            // createMap(map);
            processBounds();
        };

        // Map(float min_x ,float min_y ,float max_x ,float max_y): min_x(min_x),
        // min_y(min_y), max_x(max_x), max_y(max_y), halton_index(0) 
        // {processBounds();};
        
        void addObstacle(Polygon shape); //NOT DONE
        bool colliding(point2d point); //DONE: MAP.CPP
        bool colliding(arc a); //DONE: MAP.CPP
        bool colliding(line l); //DONE: MAP.CPP
        bool colliding(arcs A); //DONE: MAP.CPP
        // void createMap(Polygon map);

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

        
};