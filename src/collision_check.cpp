#pragma once
#include "geometry.h"
#include "collision_check.h"
#include "map.h"

float CollisionCheck::distance(point2d p1, point2d p2){
    return (p2 - p1).norm();
}
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

        if (l1xmin > l2xmax || l2xmin > l2xmax) return false;
        if (l1ymin > l2ymax || l2ymin > l2ymax) return false;

        point2d p1 = l1.p_initial;
        point2d p2 = l1.p_final;
        point2d p3 = l2.p_initial;
        point2d p4 = l2.p_final;

        float val1 = (float(p2.y - p1.y) * (p3.x - p2.x)) - 
           (float(p2.x - p1.x) * (p3.y - p2.y));

        float val2 = (float(p2.y - p1.y) * (p4.x - p2.x)) - 
           (float(p2.x - p1.x) * (p4.y - p2.y));

        if (val1 == 0 || val2 == 0 ) return false;

        if (val1/abs(val1) == val2/abs(val2)) return false;

        return true;
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
    float angle_start;
    float angle_end;
    if (arc1.K > 0)
    {
        angle_end = atan2(arc1.start.x.y-arc1.center.y, arc1.start.x.x-arc1.center.x);
        angle_start = atan2(arc1.end.x.y-arc1.center.y, arc1.end.x.x-arc1.center.x);
    }
    else
    {
        angle_start = atan2(arc1.start.x.y-arc1.center.y, arc1.start.x.x-arc1.center.x);
        angle_end = atan2(arc1.end.x.y-arc1.center.y, arc1.end.x.x-arc1.center.x);
    }

    float a = line_vector.x*line_vector.x + line_vector.y*line_vector.y;
    float b = 2*(center2initial.x*line_vector.x + center2initial.y*line_vector.y);
    float c = (center2initial.x*center2initial.x + center2initial.y*center2initial.y) - arc1.radius*arc1.radius;
    
    float discriminant = b*b-4*a*c;
    
    if( discriminant < 0 || a <= 0.0000001)
    {
        // no intersection
        result.intersects = false;
    }
    else if (discriminant == 0)
    {
        float t = -b/(2*a);
        if (t > 0 && t < 1)
        {
            result.intersection.x = l1.p_initial.x + t*line_vector.x;
            result.intersection.y = l1.p_initial.y + t*line_vector.y;
            float angle = atan2(result.intersection.y - arc1.center.y, result.intersection.x - arc1.center.x);
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
    }
    else
    {   
        discriminant = sqrt(discriminant);
        float t1 = (-b - discriminant)/(2*a);
        float t2 = (-b + discriminant)/(2*a);

        if((t1 > 0 && t1 < 1))
        {
            intersection_result temp1;
            temp1.intersection.x = l1.p_initial.x + t1*line_vector.x;
            temp1.intersection.y = l1.p_initial.y + t1*line_vector.y;

            float angle1 = atan2(temp1.intersection.y - arc1.center.y, temp1.intersection.x - arc1.center.x);
            if((t2 > 0 && t2 < 1))
            {
                intersection_result temp2;
                temp2.intersection.x = l1.p_initial.x + t2*line_vector.x;
                temp2.intersection.y = l1.p_initial.y + t2*line_vector.y;
                float angle2 = atan2(temp2.intersection.y - arc1.center.y, temp2.intersection.x - arc1.center.x);
                if (angle_start < angle_end)
                {
                    if ((angle1 > angle_start && angle1 < angle_end)||(angle2 > angle_start && angle2 < angle_end))
                    {
                        result.intersects = true;
                    }
                    else
                    {
                        result.intersects = false;
                    }
                } else
                {
                    if (((angle1 >= angle_start && angle1 <= M_PI) || (angle1 >= - M_PI && angle1 <= angle_end))||((angle2 >= angle_start && angle2 <= M_PI) || (angle2 >= - M_PI && angle2 <= angle_end)))
                    {
                        result.intersects = true;
                    }
                    else
                    {
                        result.intersects = false;
                    }
                }
            }
            
            if (angle_start < angle_end)
            {
                if ((angle1 > angle_start && angle1 < angle_end))
                {
                    result.intersects = true;
                }
                else
                {
                    result.intersects = false;
                }
            } else
            {
                if (((angle1 >= angle_start && angle1 <= M_PI) || (angle1 >= - M_PI && angle1 <= angle_end)))
                {
                    result.intersects = true;
                }
                else
                {
                    result.intersects = false;
                }
            }
        }
        else if(t2 > 0 && t2 < 1)
        {
            intersection_result temp2;
            temp2.intersection.x = l1.p_initial.x + t2*line_vector.x;
            temp2.intersection.y = l1.p_initial.y + t2*line_vector.y;
            float angle2 = atan2(temp2.intersection.y - arc1.center.y, temp2.intersection.x - arc1.center.x);
            if (angle_start < angle_end)
            {
                if (angle2 > angle_start && angle2 < angle_end)
                {
                    result.intersects = true;
                }
                else
                {
                    result.intersects = false;
                }
            } else
            {
                if (((angle2 >= angle_start && angle2 <= M_PI) || (angle2 >= - M_PI && angle2 <= angle_end)))
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
        {
            result.intersects = false;
        }
    }
    return result;
}

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