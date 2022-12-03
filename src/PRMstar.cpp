#include "PRMstar.h"
      

// max nodes
// radius of neighbourhood
void PRMstar::Solve(int n)
{
    //  init empty graph
    for (int i = 0; i < n; i ++)
    {
        point2d point = uniform_point();
        // Xnew = RandomPosition()
        // Rad = yprm*(log(itr)/(itr))^(1/d)
        // Xnearest = Near(G(V,E),Xnew,Rad) //find all nodes within a Rad
        // For node in Xnearest:
        //     if not Obstacle(Xnew,node):
        //         G(V,E) += {Xnew,node}connected component
    }
};