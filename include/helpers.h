#include <memory>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_selectors.hpp>

using namespace std;

///  -------------------- POINT DEFS ------------- ///
struct point2d
{
  float x;
  float y;
  float norm()
    {
        return sqrt(x*x + y*y);
    }
  point2d operator+(point2d &b)
  {
    point2d c;
    c.x = x + b.x;
    c.y = y + b.y;
    return c;
  }
  point2d operator-(point2d &b)
  {
    point2d c;
    c.x = x - b.x;
    c.y = y - b.y;
    return c;
  }
};
struct pose2d 
{
  point2d x;
  float theta;

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

///   -----------  GRAPH RELATED TYPES   ---------- ///
struct VertexBundle
{
  size_t index;
  
  point2d& q; // change to point2d?
};
struct TreeBundle;
typedef boost::adjacency_list<
  boost::listS,
  boost::listS,
  boost::bidirectionalS,
  shared_ptr<VertexBundle>,
  boost::no_property,
  TreeBundle
  > Tree;
typedef boost::adjacency_list_traits<
  boost::listS,
  boost::listS,
  boost::bidirectionalS,
  boost::listS
  >::vertex_descriptor Vertex;
struct TreeBundle
{
  // NearestNeighbors* nn;
};
typedef boost::graph_traits<Tree>::edge_descriptor Edge;
typedef boost::graph_traits<Tree>::edge_iterator EdgeIterator;
typedef pair<EdgeIterator, EdgeIterator> EdgeIteratorPair;
typedef pair<float, Vertex> Neighbor;
typedef boost::graph_traits<Tree>::vertex_iterator VertexIterator;
typedef pair<VertexIterator, VertexIterator> VertexIteratorPair;