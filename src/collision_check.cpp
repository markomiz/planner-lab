#pragma once
#include "geometry.h"
#include "collision_check.h"
#include "map.h"

float CollisionCheck::distance(point2d p1, point2d p2){
    return (p2 - p1).norm();
}

/// @brief Determines if two lines intersect efficiently, the triangles method
/// @param l1 first line to consider
/// @param l2 second line to consider
/// @return true if intersect, false if not
bool CollisionCheck::line_line_intersect2(line l1, line l2)
{

    float l1xmin = (l1.p_initial.x > l1.p_final.x) ? l1.p_final.x : l1.p_initial.x;
    float l1xmax = (l1.p_initial.x < l1.p_final.x) ? l1.p_final.x : l1.p_initial.x; 
    float l1ymin = (l1.p_initial.y > l1.p_final.y) ? l1.p_final.y : l1.p_initial.y;
    float l1ymax = (l1.p_initial.y < l1.p_final.y) ? l1.p_final.y : l1.p_initial.y;

    float l2xmin = (l2.p_initial.x > l2.p_final.x) ? l2.p_final.x : l2.p_initial.x;
    float l2xmax = (l2.p_initial.x < l2.p_final.x) ? l2.p_final.x : l2.p_initial.x; 
    float l2ymin = (l2.p_initial.y > l2.p_final.y) ? l2.p_final.y : l2.p_initial.y;
    float l2ymax = (l2.p_initial.y < l2.p_final.y) ? l2.p_final.y : l2.p_initial.y;

    if (l1xmin > l2xmax || l2xmin > l1xmax) return false;
    if (l1ymin > l2ymax || l2ymin > l1ymax) return false;

    point2d p1 = l1.p_initial;
    point2d p2 = l1.p_final;
    point2d p3 = l2.p_initial;
    point2d p4 = l2.p_final;

    float val1 = (float(p2.y - p1.y) * (p3.x - p2.x)) - 
        (float(p2.x - p1.x) * (p3.y - p2.y));

    float val2 = (float(p2.y - p1.y) * (p4.x - p2.x)) - 
        (float(p2.x - p1.x) * (p4.y - p2.y));

    // if (val1 == 0 || val2 == 0 ) return false;

    if (val1/abs(val1) == val2/abs(val2)) return false;

    return true;
}

/// @brief Determines if two lines intersect also returning the intersection point. Required for exact cell decomposition
/// @param l1 first line to consider
/// @param l2 second line to consider
/// @return intersection result object, containing a bool (true if intersect, false if not) and the intersection point
intersection_result CollisionCheck::line_line_intersect(line l1, line l2)
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
        if (std::isnan(offset1) && std::isnan(offset2))
        {
            result.intersection.x = FLT_MAX;
            result.intersection.y = FLT_MAX;
            result.intersects = true;
            return result;
        }
        double d = fabs(offset2 - offset1) / ((m1 * m2) - 1);
        result.intersection.x = FLT_MAX;
        result.intersection.y = FLT_MAX;
        if (d == 0)
        {
            result.intersects = true;
            return result;
        }
        else
        {
            result.intersects = false;
            return result;
        }
    }
    else
    {
        result.intersection.x = (b2*c1 - b1*c2)/determinant;
        result.intersection.y = (a1*c2 - a2*c1)/determinant;
        if ((result.intersection.x >= std::min(l1.p_initial.x, l1.p_final.x) && result.intersection.x <= std::max(l1.p_initial.x, l1.p_final.x)) 
            && (result.intersection.y >= std::min(l1.p_initial.y, l1.p_final.y) && result.intersection.y <= std::max(l1.p_initial.y, l1.p_final.y))
            && (result.intersection.x >= std::min(l2.p_initial.x, l2.p_final.x) && result.intersection.x <= std::max(l2.p_initial.x, l2.p_final.x))
            && (result.intersection.y >= std::min(l2.p_initial.y, l2.p_final.y) && result.intersection.y <= std::max(l2.p_initial.y, l2.p_final.y))
            )
        {
            result.intersects = true;
            return result;
        }
        else
        {
            result.intersects = false;
            return result;
        }
    }
}

/// @brief Determine if a line and a circular arc intersect
/// @param l1 line to consider
/// @param arc1 arc to consider
/// @return intersection result object, containing a bool (true if intersect, false if not) and the intersection point 

bool CollisionCheck::line_arc_intersect(line l1, arc arc1)
{
    point2d ab = l1.p_final - l1.p_initial; // line as vector
    point2d ac = arc1.center - l1.p_initial; // line from start to center

    point2d d;
    float k = (ac.x*ab.x + ac.y*ab.y) / (ab.x*ab.x + ab.y*ab.y) ;
    d.x = k* ab.x;
    d.y = k* ab.y;
    d = d + l1.p_initial;

    point2d dc = d - arc1.center;
    float dcnorm = dc.norm();
    if (dcnorm > arc1.radius) return false; // too far to intersect

    point2d ab_unit = ab; // unit vector in the direction of ab
    float ab_norm = ab.norm();
    ab_unit.x /= ab_norm;
    ab_unit.y /= ab_norm;

    float theta = acos(dcnorm/arc1.radius);
    float l = arc1.radius * sin(theta); // length of line from D to intersection

    point2d dif = ab_unit;
    dif.x *= l;
    dif.y *= l;

    point2d int1 = d + dif; // one intersection point
    point2d int2 = d - dif; // other intersection point

    bool int1on = (int1 - l1.p_initial).norm() < ab_norm;
    bool int2on = (int2 - l1.p_initial).norm() < ab_norm;

    if (!int1on && !int2on) return false;

    point2d cS = arc1.center - arc1.start.x;
    point2d cF = arc1.center - arc1.end.x;

    float thS = atan2(cS.y, cS.x); 
    float thF = atan2(cF.y, cF.x);

    point2d vint1 = arc1.center - int1;
    point2d vint2 = arc1.center - int2;

    float thint1 = atan2(vint1.y,vint1.x);
    float thint2 = atan2(vint2.y,vint2.x);

    if (arc1.K > 0 && thS > thF)
    {
        if (thint1 >= thS || thint1 <= thF && int1on) return true;
        if (thint2 >= thS || thint2 <= thF && int2on) return true;
    }
    else if (arc1.K > 0 && thS < thF)
    {
        if (thint1 <= thF  && thint1 >= thS && int1on) return true;
        if (thint2 <= thF  && thint2 >= thS && int2on) return true; 
    }
    else if (arc1.K < 0 && thS > thF)
    {
        if (thint1 >= thF && thint1 <= thS && int1on) return true;
        if (thint2 >= thF && thint2 <= thS && int2on) return true;
    }
    else if (arc1.K < 0 && thS < thF)
    {
        if (thint1 <= thS || thint1 >= thF && int1on) return true;
        if (thint2 <= thS || thint2 >= thF && int2on) return true;
    }

    return false;
}
/// @brief determines if a point is inside or outside a polygon using the infinite line method. Also works for non-convex case.
/// @param p 
/// @param shape 
/// @return true if point is inside, false if it isnt
bool CollisionCheck::point_in_polygon(point2d p, Polygon shape)
{
    // for each line of polygon - see if line from point crosses
    line l;
    l.p_initial = p;
    point2d end;
    end.x = p.x + 100000000;
    end.y = p.y;
    l.p_final = end;

    int num_intersections = 0;
    for (auto j = 0; j < shape.edges.size(); j++)
    {
        if( line_line_intersect2(shape.edges[j], l))
        {
             num_intersections += 1;
        }
    }
    if (num_intersections%2 == 0)
    {
        return false;
    }
    return true;
};


/// @brief Uses arc line intersection function to calculate if an arc intersects with a polygon
/// @param a arc input
/// @param shape polygon to consider
/// @return true if intersects, false if not
bool CollisionCheck::arc_with_polygon(arc a, Polygon shape)
{
    // for each line of polygon - see if arc crosses
    for (auto j = 0; j < shape.edges.size(); j++)
    {
        bool i = line_arc_intersect(shape.edges[j], a);
        if (i) return true;
    }

    return false;
};


float CollisionCheck::point_lineseg_dist(point2d p, line l)
{

    point2d ab = l.p_final - l.p_initial;
    point2d be = p - l.p_final;
    point2d ae = p - l.p_initial;
    // Variables to store dot product
    double ab_be, ab_ae;
    // Calculating the dot product
    ab_be = (ab.x * be.x + ab.y * be.y);
    ab_ae = (ab.x * ae.x + ab.y * ae.y);
    // Minimum distance from
    // point E to the line segment
    double reqAns = 0;
    // Case 1
    if (ab_ae > 0) {
        // Finding the magnitude
        double y = p.y - l.p_final.y;
        double x = p.x - l.p_final.x;
        reqAns = sqrt(x * x + y * y);
    }
    // Case 2
    else if (ab_ae < 0) {
        double y = p.y - l.p_initial.y;
        double x = p.x - l.p_initial.x;
        reqAns = sqrt(x * x + y * y);
    }
    // Case 3
    else {
        // Finding the perpendicular distance
        double x1 = ab.x;
        double y1 = ab.y;
        double x2 = ae.x;
        double y2 = ae.y;
        double mod = sqrt(x1 * x1 + y1 * y1);
        reqAns = abs(x1 * y2 - y1 * x2) / mod;
    }
    return reqAns;

};