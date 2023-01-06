# include "map.h"

vector<Polygon> getAvailableSquares(Map map, int split = 4)
{
  vector<Polygon> totalSquares;
  vector<Polygon> availableSquares;
  
  // Compute the number of squares in the x and y dimensions - Will try directly with number of squares
  // int numX = static_cast<int>((map.max_x - map.min_x) / size);
  // int numY = static_cast<int>((map.max_y - map.min_y) / size);
  double sizeX = static_cast<double>((map.max_x - map.min_x) / split);
  double sizeY = static_cast<double>((map.max_y - map.min_y) / split);
  // Split the figure into smaller squares
  double min_x = map.min_x, min_y = map.min_y, max_x = map.max_x, max_y = map.max_y;

  for (int y = 0; y < split; y++) {
    for (int x = 0; x < split; x++) {
      vector<point2d> square_pts;
      min_x = min_x + x * sizeX;
      min_y = min_x + y * sizeY;
      max_x = min_x + sizeX;
      max_y = min_y + sizeY;
      point2d p1, p2, p3, p4;
      p1.x = min_x;
      p1.y = min_y;
      square_pts.push_back(p1);
      p2.x = min_x;
      p2.y = max_x;
      square_pts.push_back(p2);
      p3.x = max_x;
      p3.y = max_y;
      square_pts.push_back(p3);
      p4.x = max_x;
      p4.y = min_y;
      square_pts.push_back(p4);
      
      Polygon square(square_pts)

      totalSquares.push_back(square);
    }
  }

  // Eval valid squares to get the path
  for (int i = 0; i < total_Squares.size(); i++)
  {
    
    Polygon temp_square = total_Squares[i];
    bool available;
    if (temp_square.area < area_threshold) continue; //TO BE DEFINED
    
    for (int j = 0; j < 4; j++)
    {
      if(map.colliding(temp_square.center) && !map.colliding(temp_square.edges[j]))
      {
        continue;
      }
      else if (map.colliding(temp_square.edges[j]))
      {
        Map newSplit = temp_square.toMap();
        availableSquares.push_back(getAvailableSquares(newSplit));
      }
      else availableSquares.push_back(temp_square);
    }
  }
  return availableSquares;
}