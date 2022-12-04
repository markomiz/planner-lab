#pragma once
#include <memory>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_selectors.hpp>

using namespace std;

///  -------------------- POINT DEFS ------------- ///
struct point2d
{
  point2d(){};
  point2d(float X, float Y): x(X), y(Y) {};
  float x;
  float y;
  float norm()
    {
        return sqrt(x*x + y*y);
    };
  point2d operator+(point2d &b)
  {
    point2d c;
    c.x = x + b.x;
    c.y = y + b.y;
    return c;
  };
  point2d operator-(point2d &b)
  {
    point2d c;
    c.x = x - b.x;
    c.y = y - b.y;
    return c;
  };
};
struct pose2d 
{
  point2d x;
  float theta;
  pose2d(){};
  pose2d(float xx,float yy,float th)
  {
    x.x = xx;
    x.y = yy;
    theta = th;
  };
};

/// ------------ DUBIN RELATED STUFF --------------- ///
struct ksigns {
  int l[3];
  ksigns(int l1, int l2, int l3){

  l[0] = l1;
  l[1] = l2;
  l[2] = l3;

  };
};
struct dubins_params
{
  float s[3];
  float L;
  ksigns k = ksigns(0,0,0);
  bool valid;
};
struct transformedVars
{
  float th0;
  float th1;
  float lambda;
};
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
    
    void angles() { // DOESNT THIS MEAN WE ARE ALWAYS TAKIN GTHE SHORTEST ARC? we don't really want that.
        static const double TWOPI = 6.2831853071795865;
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
    bool intersects;
};

///   -----------  GRAPH RELATED TYPES   ---------- ///
// struct VertexBundle
// {
//   size_t index;
  
//   point2d& q; // change to point2d?
// };
// struct TreeBundle;
// struct boost::adjacency_list<
//   boost::listS,
//   boost::listS,
//   boost::bidirectionalS,
//   shared_ptr<VertexBundle>,
//   boost::no_property,
//   TreeBundle
//   > Tree;
// struct boost::adjacency_list_traits<
//   boost::listS,
//   boost::listS,
//   boost::bidirectionalS,
//   boost::listS
//   >::vertex_descriptor Vertex;
// struct TreeBundle
// {
//   // NearestNeighbors* nn;
// };
// struct boost::graph_traits<Tree>::edge_descriptor Edge;
// struct boost::graph_traits<Tree>::edge_iterator EdgeIterator;
// struct pair<EdgeIterator, EdgeIterator> EdgeIteratorPair;
// struct pair<float, Vertex> Neighbor;
// struct boost::graph_traits<Tree>::vertex_iterator VertexIterator;
// struct pair<VertexIterator, VertexIterator> VertexIteratorPair;