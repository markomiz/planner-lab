#include "helpers.h"

class Polygon
{
    public:
        Polygon(std::vector<point2d> v): verteces(v)
        {
            calculateCenter();
        }
        point2d center;
        float radius;
        std::vector<point2d> verteces;
    private:
        void calculateCenter();
        void expandShape(float size);



};

class Map
{
    public:
        Map();
        ~Map();

        void addObstacle(Polygon shape);
        
    private:
        std::vector<Polygon> obstacles;
        

};