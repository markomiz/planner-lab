#include <iostream>
#include <string>
#include <bits/stdc++.h>
#include "helpers.h"
using namespace std;


struct line{
    point2d p_initial;
    point2d p_final;
    float length()
    {
        return sqrt((p_final.x-p_initial.x)*(p_final.x-p_initial.x)+(p_final.y-p_initial.y)*(p_final.y-p_initial.y));
    }
};
struct arc{
    point2d center;
    float radius;
    point2d starting_point;
    point2d ending_point;
    float theta[2];
    
    void angles() {
        static const double TWOPI = 6.2831853071795865;
        static const double RAD2DEG = 57.2957795130823209;
        // if (a1 = b1 and a2 = b2) throw an error
        theta[0] = atan2(starting_point.x - center.x, center.y - starting_point.y);
        theta[1] = atan2(ending_point.x - center.x, center.y - ending_point.y);
        if (theta[0] < 0.0)
            theta[0] += TWOPI;
        if (theta[1] < 0.0)
            theta[1] += TWOPI;
         if (theta[1] < theta[0])
         {
            float temp = theta[0];
            theta[0] = theta[1];
            theta[1] = temp;
         }
    }
};
struct intersection_result{
    point2d intersection;
    bool validity; //false if it intersects, true if not
};

float distance(point2d p1, point2d p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

intersection_result line_line_intersect(line l1, line l2)
{
    intersection_result result;
    // Line AB represented as a1x + b1y = c1
    double a1 = l1.p_final.y - l1.p_initial.y;
    double b1 = l1.p_initial.x - l1.p_final.x;
    double c1 = a1*(l1.p_initial.x)+ b1*(l1.p_initial.y);
    double m1 = -a1/b1;
    double offset1 = c1/b1;
    // Line CD represented as a2x + b2y = c2
    double a2 = l2.p_final.y - l2.p_initial.y;
    double b2 = l2.p_initial.x - l2.p_final.x;
    double c2 = a2*(l2.p_initial.x)+ b2*(l2.p_initial.y);
    double m2 = -a2/b2;
    double offset2 = c2/b2;
 
    double determinant = a1*b2 - a2*b1;

    if (determinant == 0)
    {
        double d = fabs(offset2 - offset1) / ((m1 * m2) - 1);
        result.intersection.x = FLT_MAX;
        result.intersection.y = FLT_MAX;
        if (d == 0)
        {
            result.validity = false;
            return result;
        }
        else
        {
            result.validity = true;
            return result;
        }
    }
    else
    {
        result.intersection.x = (b2*c1 - b1*c2)/determinant;
        result.intersection.y = (a1*c2 - a2*c1)/determinant;
        bool cond1 = l1.length() == distance(l1.p_initial,result.intersection) + distance(result.intersection, l1.p_final);
        bool cond2 = l2.length() == distance(l2.p_initial,result.intersection) + distance(result.intersection, l2.p_final);
        if (cond1 && cond2)
        {
            result.validity = false;
            return result;
        }
        else
        {
            result.validity = true;
            return result;
        }
    }
}

intersection_result line_arc_intersect(line l1, arc arc1)
{
    intersection_result result;
    point2d line_vector;
    line_vector.x = l1.p_final.x-l1.p_initial.x;
    line_vector.y = l1.p_final.y-l1.p_initial.y;
    point2d center2initial;
    center2initial.x = l1.p_initial.x - arc1.center.x;
    center2initial.y = l1.p_initial.y - arc1.center.y;
    
    float a = line_vector.x*line_vector.x + line_vector.y*line_vector.y;
    float b = 2*(center2initial.x*line_vector.x + center2initial.y*line_vector.y);
    float c = (center2initial.x*center2initial.x + center2initial.y*center2initial.y) - arc1.radius*arc1.radius;

    float discriminant = b*b-4*a*c;

    if( discriminant < 0 )
    {
        // no intersection
        result.validity = true;
    }
    else
    {
        
        discriminant = sqrt( discriminant);
        float t1 = (-b - discriminant)/(2*a);
        float t2 = (-b + discriminant)/(2*a);

        
        if( t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1)
        {
            result.intersection.x = a*t1*t1 + 2*b*t1 + c;
            result.intersection.y = a*t2*t2 + 2*b*t2 + c;
            float angle = atan2(result.intersection.y - arc1.center.y, result.intersection.x - arc1.center.x);
            arc1.angles();
            if (angle > arc1.theta[0] && angle < arc1.theta[1])
                result.validity = false;
        }
        else
            result.validity = true;
    }
    return result;
}

// I dont really know if this is necessary. Depends on what kind of info we get from the environment
// 
// intersection_result is_line_or_arc(point2d path_1, point2d path_2, point2d obstacle_1, point2d obstacle_2)
// {

// }

/*One thing to explore is prior to checking colision checking the values of the coordinat points. 
  Colision cant happen if obstacle is not in bounnds of the segment coords so we can reduce a priori the number of edges to check
*/

int main()
{
    point2d p1_1, p1_2, p2_1, p2_2;
    p1_1.x = 0.0;
    p1_1.y = 0.0;

    p1_2.x = 1.0;
    p1_2.y = 0.0;

    p2_1.x = 0.0;
    p2_1.y = 0.5;

    p2_2.x = 1.0;
    p2_2.y = 1.0;

    line l1, l2;
    l1.p_initial = p1_1;
    l1.p_final = p1_2;
    l2.p_initial = p2_1;
    l2.p_final = p2_2;

    intersection_result test_result;
    test_result = line_line_intersect(l1,l2);

    std::cout << test_result.validity << std::endl;
    
    return true;
}