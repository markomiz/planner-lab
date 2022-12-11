# THE BEST PLANNER
By Marko Mizdrak and Ricardo Ezequiel

## Overview
2D non-holonmic planner(s) for interfacing with a real robot and its simulator (https://github.com/pla10/Shelfino_ROS2) with ROS2.

### Methods
Overall we generate roadmaps with PRM, do nearest neighbour searches using quadtrees, and A* on the roadmaps to find paths between nodes.

Our robot moves with Dubins curves; moving either forward with max velocity or turning left or right with maximal curvature with maximum speed.

### Classes & Files

#### Geometry Helpers (include/geometry.h)
Here is where the basic structs and simple fuctions for making calculations using these structs exist.

##### arc
Struct for arcs that are part of Dubins curves, with start/end points, radius of curvature, center and length. These can generalise to straight lines too (no curvature).
##### arcs
A set of 3 arcs, making up an entire optimal Dubin path between two poses.
##### point2d
Simple 2d point - with x and y coordinates.
##### pose2d
Made up of a point and an angle.
##### dubins_params
Alternative representation of a set of arcs - the output from the dubinCurve calclations.

#### Graph Classes (include/graph.h and src/graph.cpp)

##### Node
Stores pose2d as well as variables to help with graph search such as connections, and parent node.
##### Quad
Class for the quatree implementation. Allows inserting nodes and fetching all nodes within a radius of a point.
##### Connection
Struct for storing properties about a connection between nodes.
##### Graph
Contains nodes which have connections and a quadtree. This is used for higher level node storage and querying.
#### PRMstar
A class for implemenation of the variations of building the roadmap.

#### Collision Check
For individual collision checks between points, lines, arcs.

#### Map
Stores the obstacles and allows higher level collision checking with everything on the map while CollisionCheck class is used for individual queries of the components of the map.

#### Dubin Curve
Implements the mathematics of finding the optimal curve between two points as well as a multi-point Dubins interpolation method.

#### Config/Parameter Server
For loading parameters from a file and making them available to other classes.

#### Dubin 
ROS2 node for publishing/subscribing.

## Many Robots Escape

### Time Occupancy addition to PRM

## Pursuit / Evade