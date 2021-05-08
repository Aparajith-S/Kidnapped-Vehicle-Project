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
  // Set the number of particles. Initialize all particles to 
  // first position (based on estimates of x, y, theta and their uncertainties
  // from GPS) and all weights to 1. 
  m_num_particles = 100;  
  // creates an engine from which random numbers can be generated
  std::default_random_engine gen;
  //This creates a normal (Gaussian) distribution for x
  std::normal_distribution<double> dist_x(x, std[0]);
  // Create normal distributions for y and theta
  std::normal_distribution<double> dist_y(y, std[1]); 
  std::normal_distribution<double> dist_th(theta, std[2]);
  int id = 1; // 0 is an invalid id. starting with 1
  // Add random Gaussian noise to each particle.

  for (int idx=0; idx<m_num_particles ; idx++)
  {
    Particle particle;
    particle.id = id++;
    particle.weight=1.0;
    particle.pos.x = dist_x(gen);
    particle.pos.y = dist_y(gen);
    particle.theta = dist_th(gen);
    m_particles.push_back(particle);
  }
  m_initialized = true; 
}

void ParticleFilter::prediction(const double delta_t, const double std_pos[], 
                                const double velocity, const double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // creates an engine from which random numbers can be generated
  std::default_random_engine gen;
   
   for (auto & particle : m_particles)
   { state previous = {particle.pos,
     particle.theta};
     control_s ctrl = {velocity,yaw_rate};
     state next = advanceState(previous,ctrl,delta_t);
     //This creates a normal (Gaussian) distribution for x
     std::normal_distribution<double> dist_x(next.pos.x, std_pos[0]);
     // Create normal distributions for y and theta
     std::normal_distribution<double> dist_y(next.pos.y, std_pos[1]); 
     std::normal_distribution<double> dist_th(next.theta, std_pos[2]);
     particle.pos.x=dist_x(gen);
     particle.pos.y=dist_y(gen);
     particle.theta=dist_th(gen);
   }
}

void ParticleFilter::dataAssociation(const vector<LandmarkObs>& predicted, 
                                     vector<LandmarkObs>& observations) {
  //
  //  Find the predicted measurement that is closest to each 
  //  observed measurement and assign the observed measurement to this 
  //  particular landmark.
  //  NOTE: this method will NOT be called by the grading code. But you will 
  //  probably find it useful to implement this method and use it as a helper 
  //  during the updateWeights phase.
  //
   for(auto & obsv : observations)
   {
     LandmarkObs foundObs;
     foundObs = findClosest(predicted,obsv);
     obsv.id=foundObs.id;
   }
}

void ParticleFilter::updateWeights(const double sensor_range, const double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const maps::Map &map_landmarks) {
  //  Update the weights of each particle using a mult-variate Gaussian 
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

  //go through each particle
  for(auto & particle : m_particles)
  {
    state particleCopy = {particle.pos,
    particle.theta};
    vector<LandmarkObs> preds; // store the predictions within sensor range 
    for (const auto & landmark : map_landmarks.landmark_list)
    {// sensor_range is assumed as radial range of m meters. the field of vision is assumed to be 360 degrees.
    // hence find if landmark is in reach of particle sensor range
    if(landmark.pos_f.cart2d(particle.pos) <= sensor_range)
    {
      preds.push_back(LandmarkObs{landmark.id_i,landmark.pos_f});
    }
    }
    // transform the observations based on the particle orientation
    vector<LandmarkObs> transformed_obsv;
    for(auto & obs : observations)
    {
      transformed_obsv.push_back(transform(particle.pos,obs,particle.theta));
    }

    // associate the predictions with the observations
    dataAssociation(preds, transformed_obsv);
    // initialize once again to recompute
    particle.weight=1.0;
    for(auto & tobs : transformed_obsv)
    {
      auto found = std::find_if(preds.begin(),
      preds.end(),
      [tobs](const LandmarkObs & p)
      { return (p.id == tobs.id);});
      
      if(found != preds.end())
      {
       particle.weight*=gaussian(particle.pos,{std_landmark[0],std_landmark[1]},found->pos);
      }
    }
  }
}

void ParticleFilter::resample() {
  //  Resample m_particles with replacement with probability proportional 
  //  to their weight. 
  //  NOTE: You may find std::discrete_distribution helpful here.
  //  http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  // creates an engine from which random numbers can be generated
  std::default_random_engine gen;
  // create a copy of new particles
  vector<Particle> new_particles;

  // get all of the current weights
  vector<double> weights;
  for (auto & particle :m_particles) 
  {
    weights.push_back(particle.weight);
  }

  // random starting index for resampling wheel
  std::uniform_int_distribution<int> uniintdist(0, m_num_particles-1);
  auto index = uniintdist(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  std::uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;
  int loop_it = 0;
  // go through the resampling wheel
  while (loop_it < m_num_particles) 
  {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) 
    {
      beta -= weights[index];
      index = (index + 1) % m_num_particles;
    }
    new_particles.push_back(m_particles[index]);
    loop_it++;
  }
  m_particles = new_particles;
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