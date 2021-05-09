///@file geometry.h
///@brief Some 2d geometry functions and data structures.
///@date 8-5-2021
///@author: s.aparajith@live.com
///@copyright : none reserved. No liabilities. MIT License

#ifndef GEOMETRY_H_
#define GEOMETRY_H_
#include <cmath>
namespace geo_tools{
  
    struct point2d
    {
      double x;
      double y;
    };
    // find the 2D shortest path between self and another point.
    inline double cart2d(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow((y1 - y2),2) + pow((x1 - x2),2));
    }
}
#endif