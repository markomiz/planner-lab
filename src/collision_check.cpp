#pragma once
#include "geometry.h"
#include "collision_check.h"
#include "map.h"

float CollisionCheck::distance(point2d p1, point2d p2){
    return (p2 - p1).norm();
}

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
            // cout << "here2" << endl;
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

intersection_result CollisionCheck::line_arc_intersect(line l1, arc arc1)
{
    intersection_result result;
    point2d line_vector;
    line_vector.x = l1.p_final.x-l1.p_initial.x;
    line_vector.y = l1.p_final.y-l1.p_initial.y;
    
    point2d center2initial;
    
    center2initial.x = l1.p_initial.x - arc1.center.x;
    center2initial.y = l1.p_initial.y - arc1.center.y;
    float angle_start = atan2(abs(arc1.start.x.y-arc1.center.y),abs(arc1.start.x.x-arc1.center.x));
    float angle_end = atan2(abs(arc1.end.x.y-arc1.center.y),abs(arc1.end.x.x-arc1.center.x));

    float a = line_vector.x*line_vector.x + line_vector.y*line_vector.y;
    float b = 2*(center2initial.x*line_vector.x + center2initial.y*line_vector.y);
    float c = (center2initial.x*center2initial.x + center2initial.y*center2initial.y) - arc1.radius*arc1.radius;
    
    float discriminant = b*b-4*a*c;
    
    if( discriminant < 0 )
    {
        // no intersection
        result.intersects = false;
    }
    else
    {
        float angle = atan((line_vector.y/line_vector.x));
        
        discriminant = sqrt( discriminant);
        float t1 = (-b - discriminant)/(2*a);
        float t2 = (-b + discriminant)/(2*a);
        

        if((t1 > 0 && t1 < 1) && (t2 > 0 && t2 < 1))
        {
            result.intersection.x = a*t1*t1 + 2*b*t1 + c;
            result.intersection.y = a*t2*t2 + 2*b*t2 + c;

            float angle = atan2(result.intersection.y - arc1.center.y, result.intersection.x - arc1.center.x);
            angle = arc::mod2pi(angle);
            // cout << "\n " << angle << " angle \n"; 

            if (angle_start < angle_end)
            {
                if (angle > angle_start && angle < angle_end)
                {
                    result.intersects = true;
                }
                else
                {
                    result.intersects = false;
                }
            } else
            {
                if ((angle >= angle_start && angle <= M_PI) || (angle >= - M_PI && angle <= angle_end))
                {
                    result.intersects = true;
                }
                else
                {
                    result.intersects = false;
                }
            }
        }
        else
            result.intersects = false;
    }
    return result;
}

bool CollisionCheck::point_in_polygon(point2d p, Polygon shape)
{
    // for each line of polygon - see if line from point crosses
    line l;
    l.p_initial = p;
    point2d end;
    end.x = p.x + 100;
    end.y = p.y + 100;
    l.p_final = end;

    int num_intersections = 0;
    for (auto j = 0; j < shape.edges.size(); j++)
    {
        intersection_result i = line_line_intersect(shape.edges[j], l);
        if (i.intersects) 
        {
            num_intersections += 1;
        }
    }
    if (num_intersections == 1)
    {
        return true;
    }
    return false;
};

bool CollisionCheck::arc_with_polygon(arc a, Polygon shape)
{
    // for each line of polygon - see if arc crosses
    for (auto j = 0; j < shape.edges.size(); j++)
    {
        // cout << "\n intersection result \n";
        intersection_result i = line_arc_intersect(shape.edges[j], a);

        if (i.intersects) return true;
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