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

using std::normal_distribution;
using std::string;
using std::vector;
std::default_random_engine gen;
void ParticleFilter::init(const double x, 
const double y, 
const double theta, 
const double std[]) {
  // Set the number of particles. Initialize all particles to 
  // first position (based on estimates of x, y, theta and their uncertainties
  // from GPS) and all weights to 1. 

  m_num_particles = 100;

  // define normal distributions for sensor noise
  normal_distribution<double> N_x_init(0, std[0]);
  normal_distribution<double> N_y_init(0, std[1]);
  normal_distribution<double> N_theta_init(0, std[2]);

  // init particles
  for (int i = 0; i < m_num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;
    p.weight = 1.0;

    // add noise
    p.x += N_x_init(gen);
    p.y += N_y_init(gen);
    p.theta += N_theta_init(gen);

    m_particles.push_back(p);
  }

  m_initialized = true;
}

void ParticleFilter::prediction(const double delta_t, const double std_pos[], 
                                const double velocity, const double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // creates an engine from which random numbers can be generated
   // define normal distributions for sensor noise
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  // Create normal distributions for y and theta
  std::normal_distribution<double> dist_y(0, std_pos[1]); 
  std::normal_distribution<double> dist_th(0, std_pos[2]); 
   for (auto & particle : m_particles)
   { state previous = {particle.x,particle.y,
     particle.theta};
     control_s ctrl = {velocity,yaw_rate};
     state next = advanceState(previous,ctrl,delta_t);
     //This creates a normal (Gaussian) distribution for x

     particle.x=next.x+dist_x(gen);
     particle.y=next.y+dist_y(gen);
     particle.theta=next.theta+dist_th(gen);
   }
}

void ParticleFilter::dataAssociation(const vector<LandmarkObs>& predicted, 
                                     vector<LandmarkObs>& observations,
                                     Particle& particle) {
  //  Find the predicted measurement that is closest to each 
  //  observed measurement and assign the observed measurement to this 
  //  particular landmark.
  //  NOTE: this method will NOT be called by the grading code. But you will 
  //  probably find it useful to implement this method and use it as a helper 
  //  during the updateWeights phase.
   std::vector<int> associations;
   std::vector<double> sense_x;
   std::vector<double> sense_y; 
  for(auto & obsv : observations)
   {
     double dist = std::numeric_limits<double>::max();
     //initialize to invalid values
     LandmarkObs retValue{ -1,
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()};
    //take the nearest landmark
    for (auto& point : predicted)
    {
        double running_dist = geo_tools::cart2d(point.x,point.y,obsv.x,obsv.y);
        if (running_dist < dist)
        {
            dist = running_dist;
            retValue = point;
        }
    }
     obsv.id=retValue.id;
   //for the visualization.
   associations.push_back(retValue.id);
	 sense_x.push_back(retValue.x);
	 sense_y.push_back(retValue.y);
   }
  // to display on the simulator
  SetAssociations(particle, associations, sense_x, sense_y);
}

void ParticleFilter::updateWeights(const double sensor_range,const double std_landmark[], 
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
  double weight_sum = 0.0;
  for(auto & particle : m_particles)
  {
    vector<LandmarkObs> preds; // store the predictions within sensor range 
    for (const auto & landmark : map_landmarks.landmark_list)
    {// sensor_range is assumed as radial range of m meters. the field of vision is assumed to be 360 degrees.
    // to make a performance tweak the radial spot is approximated as a square.
    if((fabs(landmark.x_f-particle.x)<=sensor_range) && (fabs(landmark.y_f-particle.y) <=sensor_range))
    {
      preds.push_back(LandmarkObs{landmark.id_i,
                                  landmark.x_f,
                                  landmark.y_f});
    }
    }
    // transform the observations based on the particle orientation
    vector<LandmarkObs> transformed_obsv;
    for(auto & obs : observations)
    {
      transformed_obsv.push_back(transform(particle.x,particle.y,obs,particle.theta));
    }
                      
    // associate the predictions with the observations
    dataAssociation(preds, transformed_obsv, particle);
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
        double w = gaussian({tobs.x,tobs.y},{std_landmark[0],std_landmark[1]},{found->x,found->y});
        if(w > std::numeric_limits<double>::epsilon())
        {
          particle.weight*=w;
        }
      }
    }
    weight_sum+=particle.weight;
  }
  
   //Normalization of weights
   for(auto & particle : m_particles)
   {
   	particle.weight/=weight_sum;   	
   }
}

void ParticleFilter::resample() {
  //  Resample m_particles with replacement with probability proportional 
  //  to their weight. 
  //  NOTE: You may find std::discrete_distribution helpful here.
  //  http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  // creates an engine from which random numbers can be generated

  // create a copy of new particles
  vector<Particle> new_particles;
  double max_weight = std::numeric_limits<double>::min();
  // get all of the current weights
  m_weights.clear();
  for (auto & particle :m_particles) 
  {
    m_weights.push_back(particle.weight);
    if(max_weight<particle.weight)
    {
      max_weight = particle.weight;
    }
  }

  // random starting index for resampling wheel
  std::uniform_int_distribution<int> uniintdist(0, m_num_particles-1);
  auto index = uniintdist(gen);


  // uniform random distribution [0.0, max_weight)
  std::uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;
  int loop_it = 0;
  
  // go through the resampling wheel
  while (loop_it < m_num_particles) 
  {

    beta += unirealdist(gen) * 2.0;
    while (beta > m_weights[index]) 
    {      
      beta -= m_weights[index];
      index = (index + 1) % m_num_particles;
    }
    new_particles.push_back(m_particles[index]);
    loop_it++;
  }
  m_particles = new_particles;
}

const std::vector<Particle> & ParticleFilter::getParticleListReference(void) const
{
  return m_particles;
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