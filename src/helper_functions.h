/**
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 *
 * Created on: Dec 13, 2016
 * Author: Tiffany Huang
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "map.h"

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

/**
 * Struct representing one position/control measurement.
 */
struct control_s {
  double velocity;  // Velocity [m/s]
  double yawrate;   // Yaw rate [rad/s]
};

/**
 * Struct representing one ground truth position.
 */
struct ground_truth {
  double x;     // Global vehicle x position [m]
  double y;     // Global vehicle y position
  double theta; // Global vehicle yaw [rad]
};

/**
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
  
  int id;     // Id of matching landmark in the map.
  double x;   // Local (vehicle coords) x position of landmark observation [m]
  double y;   // Local (vehicle coords) y position of landmark observation [m]
};

/**
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
 * Computes the error between ground truth and particle filter data.
 * @param (gt_x, gt_y, gt_theta) x, y and theta of ground truth
 * @param (pf_x, pf_y, pf_theta) x, y and theta of particle filter
 * @output Error between ground truth and particle filter data.
 */
inline double * getError(double gt_x, double gt_y, double gt_theta, double pf_x,
                         double pf_y, double pf_theta) {
  static double error[3];
  error[0] = fabs(pf_x - gt_x);
  error[1] = fabs(pf_y - gt_y);
  error[2] = fabs(pf_theta - gt_theta);
  error[2] = fmod(error[2], 2.0 * M_PI);
  if (error[2] > M_PI) {
    error[2] = 2.0 * M_PI - error[2];
  }
  return error;
}

/**
 * Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
inline bool read_map_data(std::string filename, Map& map) {
  // Get file of map
  std::ifstream in_file_map(filename.c_str(),std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_map) {
    return false;
  }
  
  // Declare single line of map file
  std::string line_map;

  // Run over each single line
  while (getline(in_file_map, line_map)) {

    std::istringstream iss_map(line_map);

    // Declare landmark values and ID
    float landmark_x_f, landmark_y_f;
    int id_i;

    // Read data from current line to values
    iss_map >> landmark_x_f;
    iss_map >> landmark_y_f;
    iss_map >> id_i;

    // Declare single_landmark
    Map::single_landmark_s single_landmark_temp;

    // Set values
    single_landmark_temp.id_i = id_i;
    single_landmark_temp.x_f  = landmark_x_f;
    single_landmark_temp.y_f  = landmark_y_f;

    // Add to landmark list of map
    map.landmark_list.push_back(single_landmark_temp);
  }
  return true;
}

/**
 * Reads control data from a file.
 * @param filename Name of file containing control measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_control_data(std::string filename, 
                              std::vector<control_s>& position_meas) {
  // Get file of position measurements
  std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_pos) {
    return false;
  }

  // Declare single line of position measurement file:
  std::string line_pos;

  // Run over each single line:
  while (getline(in_file_pos, line_pos)) {

    std::istringstream iss_pos(line_pos);

    // Declare position values:
    double velocity, yawrate;

    // Declare single control measurement:
    control_s meas;

    //read data from line to values:
    iss_pos >> velocity;
    iss_pos >> yawrate;
    
    // Set values
    meas.velocity = velocity;
    meas.yawrate = yawrate;

    // Add to list of control measurements:
    position_meas.push_back(meas);
  }
  return true;
}

/**
 * Reads ground truth data from a file.
 * @param filename Name of file containing ground truth.
 * @output True if opening and reading file was successful
 */
inline bool read_gt_data(std::string filename, std::vector<ground_truth>& gt) {
  // Get file of position measurements
  std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_pos) {
    return false;
  }

  // Declare single line of position measurement file
  std::string line_pos;

  // Run over each single line
  while (getline(in_file_pos, line_pos)) {

    std::istringstream iss_pos(line_pos);

    // Declare position values
    double x, y, azimuth;

    // Declare single ground truth
    ground_truth single_gt; 

    //read data from line to values
    iss_pos >> x;
    iss_pos >> y;
    iss_pos >> azimuth;

    // Set values
    single_gt.x = x;
    single_gt.y = y;
    single_gt.theta = azimuth;

    // Add to list of control measurements and ground truth
    gt.push_back(single_gt);
  }
  return true;
}

/**
 * Reads landmark observation data from a file.
 * @param filename Name of file containing landmark observation measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_landmark_data(std::string filename, 
                               std::vector<LandmarkObs>& observations) {
  // Get file of landmark measurements
  std::ifstream in_file_obs(filename.c_str(),std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_obs) {
    return false;
  }

  // Declare single line of landmark measurement file
  std::string line_obs;

  // Run over each single line
  while (getline(in_file_obs, line_obs)) {

    std::istringstream iss_obs(line_obs);

    // Declare position values
    double local_x, local_y;

    //read data from line to values
    iss_obs >> local_x;
    iss_obs >> local_y;

    // Declare single landmark measurement
    LandmarkObs meas;

    // Set values
    meas.x = local_x;
    meas.y = local_y;

    // Add to list of control measurements
    observations.push_back(meas);
  }
  return true;
}

///@brief finds the multivariate gaussian of a point around the mean and given standard deviation 
///@param[in] point the interested point for which the multivariate gaussian has to be computed
///@param[in] sigma the stddev
///@param[in] mu the mean 
inline double gaussian(const geo_tools::point2d<double> & point, const geo_tools::point2d<double> &  sigma, const geo_tools::point2d<double> &  mu)
{
    return (1.0 / (2.0 * M_PI * sigma.x * sigma.y)) *
        exp(-((point.x - mu.x) * (point.x - mu.x) / (2.0 * sigma.x * sigma.x) +
            (point.y - mu.y) * (point.y - mu.y) / (2.0 * sigma.y * sigma.y)));
}

///@brief computes next state using vehicle model equations 
///@details based on its previous position and theta, and given its current velocity and yaw rate over the segment.
///@param[in] i_previous the previous state
///@param[in] i_ctrl the current control action leading to a velocity and cetrain yaw rate
///@param[in] i_dt the time period in seconds of the running application 
///@return next state  
inline state advanceState(const state & i_previous, const control_s & i_ctrl, const double i_dt)
{
    state next;
    next.pos.x = i_previous.pos.x + (i_ctrl.velocity / i_ctrl.yawrate) * (sin(i_previous.theta + i_ctrl.yawrate * i_dt) - sin(i_previous.theta));
    next.pos.y = i_previous.pos.y + (i_ctrl.velocity / i_ctrl.yawrate) * (-cos(i_previous.theta + i_ctrl.yawrate * i_dt) + cos(i_previous.theta));
    next.theta = i_previous.theta + i_ctrl.yawrate * i_dt;
    return next;
}

///@brief finds the closest map landmark corresponding to the observation 
///@details if the list is empty then id will be filled with an invalid -1 value
///@param[in] mapLandmarks list of all map landmarks 
///@param[in] obs the measured observation 
///@return closest Landmark
inline LandmarkObs findClosest(const std::vector<LandmarkObs>& mapLandmarks, const LandmarkObs& obs)
{
    float dist = std::numeric_limits<float>::max();
    LandmarkObs retValue{ -1,
        geo_tools::point2d<double>{std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()} };
    for (const auto& point : mapLandmarks)
    {
        float running_dist = point.pos.cart2d(obs.pos);
        if (running_dist < dist)
        {
            dist = running_dist;
            retValue = point;
        }
    }
    return retValue;
}

///@brief rotational and translational transform of the observation wrt the particle
///@param[in] particle list of all map landmarks 
///@param[in] observation the measured observation 
///@param[in] theta the heading angle of the particle.
///@return transformed Landmark
inline LandmarkObs transform(const geo_tools::point2d<double>& particle, const LandmarkObs & observation, const double theta)
{
    return{observation.id,
        {observation.pos.x * cos(theta) - sin(theta) * observation.pos.y + particle.x,
        observation.pos.x * sinf(theta) + cosf(theta) * observation.pos.y + particle.y}};
}
#endif  // HELPER_FUNCTIONS_H_
