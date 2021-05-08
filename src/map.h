/// @file map.h
/// @brief map definitions
/// @date 5-8-2021
/// @author: s.aparajith@live.com
///@copyright : none reserved. No liabilities. MIT License

#ifndef MAP_H_
#define MAP_H_

#include <vector>

class Map {
 public:  
  struct single_landmark_s {
    int id_i ; // Landmark ID
    float x_f; // Landmark x-position in the map (global coordinates)
    float y_f; // Landmark y-position in the map (global coordinates)
  };

  std::vector<single_landmark_s> landmark_list; // List of landmarks in the map
};

#endif  // MAP_H_
