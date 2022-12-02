#include <memory>
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
  ::std::size_t index;
  
  VectorPtr q;
};
struct TreeBundle;
typedef ::boost::adjacency_list<
  ::boost::listS,
  ::boost::listS,
  ::boost::bidirectionalS,
  ::std::shared_ptr<VertexBundle>,
  ::boost::no_property,
  TreeBundle
> Tree;
typedef ::boost::adjacency_list_traits<
  ::boost::listS,
  ::boost::listS,
  ::boost::bidirectionalS,
  ::boost::listS
>::vertex_descriptor Vertex;
struct TreeBundle
{
  NearestNeighbors* nn;
};
typedef ::boost::graph_traits<Tree>::edge_descriptor Edge;
typedef ::boost::graph_traits<Tree>::edge_iterator EdgeIterator;
typedef ::std::pair<EdgeIterator, EdgeIterator> EdgeIteratorPair;
typedef ::std::pair<::rl::math::Real, Vertex> Neighbor;
typedef ::boost::graph_traits<Tree>::vertex_iterator VertexIterator;
typedef ::std::pair<VertexIterator, VertexIterator> VertexIteratorPair;