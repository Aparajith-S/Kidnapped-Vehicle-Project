/// @file map.h
/// @brief map definitions
/// @date 5-8-2021
/// @author: s.aparajith@live.com
///@copyright : none reserved. No liabilities. MIT License

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include "geometry.h"
namespace maps{
struct Map { 
  struct single_landmark_s {
    int id_i ; // Landmark ID
    geo_tools::point2d<float> pos_f; // Landmark x,y-position in the map (global coordinates)
  };

  std::vector<single_landmark_s> landmark_list; // List of landmarks in the map
};
}
#endif  // MAP_H_
