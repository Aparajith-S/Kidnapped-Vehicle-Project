/// @file : particle_filter.cpp
/// @brief : 2D particle filter class.
/// @date : 8-5-2021
/// @author : s.aparajith@live.com 
/// @copyright : none reserved. No liabilities. MIT License

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(const double x, 
const double y, 
const double theta, 
const double std[]) {
  // TODO: Set the number of particles. Initialize all particles to 
  // first position (based on estimates of x, y, theta and their uncertainties
  // from GPS) and all weights to 1. 

}

void ParticleFilter::prediction(const double delta_t, const double std_pos[], 
                                const double velocity, const double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.

}

void ParticleFilter::dataAssociation(const vector<LandmarkObs>& predicted, 
                                     vector<LandmarkObs>& observations) {
  //
  //  TODO: Find the predicted measurement that is closest to each 
  //  observed measurement and assign the observed measurement to this 
  //  particular landmark.
  //  NOTE: this method will NOT be called by the grading code. But you will 
  //  probably find it useful to implement this method and use it as a helper 
  //  during the updateWeights phase.
  //
}

void ParticleFilter::updateWeights(const double sensor_range, const double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const maps::Map &map_landmarks) {
  //  TODO: Update the weights of each particle using a mult-variate Gaussian 
  //  distribution. You can read more about this distribution here: 
  //  https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  //  NOTE: The observations are given in the VEHICLE'S coordinate system. 
  //  Your m_particles are located according to the MAP'S coordinate system. 
  //  You will need to transform between the two systems. Keep in mind that
  //  this transformation requires both rotation AND translation (but no scaling).
  //  The following is a good resource for the theory:
  //  https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //  and the following is a good resource for the actual equation to implement
  //  (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html

}

void ParticleFilter::resample() {
  //  TODO: Resample m_particles with replacement with probability proportional 
  //  to their weight. 
  //  NOTE: You may find std::discrete_distribution helpful here.
  //  http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}