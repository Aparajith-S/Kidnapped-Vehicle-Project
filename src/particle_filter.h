/// @file : particle_filter.h
/// @brief : 2D particle filter class.
/// @date : 7-5-2021
/// @author : s.aparajith@live.com 
/// @copyright : none reserved. No liabilities. MIT License

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <string>
#include <vector>
#include "helper_functions.h"

struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;
};


class ParticleFilter {  
 public:
  // Constructor
  // @param num_particles Number of particles
  ParticleFilter() : num_particles(0), is_initialized(false) {}

  // Destructor
  ~ParticleFilter() {}

  /// @brief init Initializes particle filter by initializing particles to Gaussian
  ///   distribution around first position and all the weights to 1.
  /// @param[in] x Initial x position [m] (simulated estimate from GPS)
  /// @param[in] y Initial y position [m]
  /// @param[in] theta Initial orientation [rad]
  /// @param[in] std[] Array of dimension 3 [standard deviation of x [m], 
  ///   standard deviation of y [m], standard deviation of yaw [rad]]
  void init(const double x, const double y, const double theta, const double std[]);

  /// @brief prediction Predicts the state for the next time step
  ///        using the process model.
  /// @param[in] delta_t Time between time step t and t+1 in measurements [s]
  /// @param[in] std_pos[] Array of dimension 3 [standard deviation of x [m], 
  ///   standard deviation of y [m], standard deviation of yaw [rad]]
  /// @param[in] velocity Velocity of car from t to t+1 [m/s]
  /// @param[in] yaw_rate Yaw rate of car from t to t+1 [rad/s]
  void prediction(const double delta_t, const double std_pos[], const double velocity, 
                  const double yaw_rate);
  
  /// 
  /// @brief dataAssociation Finds which observations correspond to which landmarks 
  ///   (likely by using a nearest-neighbors data association).
  /// @param[in] predicted Vector of predicted landmark observations
  /// @param[out] observations Vector of landmark observations
  void dataAssociation(const std::vector<LandmarkObs> & predicted, 
                       std::vector<LandmarkObs>& observations);
  
  /// 
  /// @brief updateWeights Updates the weights for each particle based on the likelihood
  ///   of the observed measurements. 
  /// @param[in] sensor_range Range [m] of sensor
  /// @param[in] std_landmark[] Array of dimension 2
  ///   [Landmark measurement uncertainty [x [m], y [m]]]
  /// @param[in] observations Vector of landmark observations
  /// @param[in] map Map class containing map landmarks
  void updateWeights(const double sensor_range, 
                     const double std_landmark[], 
                     const std::vector<LandmarkObs> &observations,
                     const maps::Map &map_landmarks);
  
  /// @brief resample Resamples from the updated set of particles to form
  ///   the new set of particles.
  void resample();

  /// @brief Set a particles list of associations, along with the associations'
  ///   calculated world x,y coordinates
  /// @details This can be a very useful debugging tool to make sure transformations 
  ///   are correct and assocations correctly connected
  /// @param[out] particle particle whose association needs to be set
  /// @param[in] associations list of all associations
  /// @param[in] sense_x x position sensor value list
  /// @param[in] sense_y y position sensor value list
  void SetAssociations(Particle& particle, const std::vector<int>& associations,
                       const std::vector<double>& sense_x, 
                       const std::vector<double>& sense_y);

  /// @brief initialized Returns whether particle filter is initialized yet or not.
  const bool isInitialized() const {
    return m_initialized;
  }

  /// @brief Used for obtaining debugging information related to particles.
  std::string getAssociations(Particle best);
  std::string getSenseCoord(Particle best, std::string coord);

 private:
  // Number of particles to draw
  int m_num_particles; 
  
  // Flag, if filter is initialized
  bool m_initialized;
  
  // Vector of weights of all particles
  std::vector<double> m_weights; 

  // Set of current particles
  std::vector<Particle> m_particles;

};

#endif  // PARTICLE_FILTER_H_