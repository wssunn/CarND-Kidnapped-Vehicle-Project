/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

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
using std::normal_distribution;

//create only once the default random engine
static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   */
  num_particles = 100;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i=0; i < num_particles; ++i)
  {
    // initialise a particle
    Particle p;
    p.id      = int(i);
    p.weight  = 1.0;
    p.x       = dist_x(gen);
    p.y       = dist_y(gen);
    p.theta   = dist_theta(gen);

    //Add the particles to the particle filter list
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   */
  for (auto &p : particles)
  {

    //add gaussian noise to the particles
    normal_distribution<double> dist_x(0.0, std_pos[0]);
    normal_distribution<double> dist_y(0.0, std_pos[1]);
    normal_distribution<double> dist_theta(0.0, std_pos[2]);

    if (abs(yaw_rate) > 1e-5)
    {
      // Apply equations of motion model (turning)
      double theta_pred = p.theta + yaw_rate *delta_t;
      p.x += velocity / yaw_rate * (sin(theta_pred) - sin(p.theta));
      p.y += velocity / yaw_rate * (cos(theta_pred) - cos(p.theta));
      p.theta = theta_pred;
    }
    else
    {
      // Apply equations of motion model (linear)
      // this is crucial, if the yaw rate is too small, there is a 
      // division by zero and the model will freeze
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
    }

    //add noise to prediction
    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);

  }
  // for (size_t i = 0; i < particles.size(); ++i)
  // {
  //   Particle p = particles[i];
    
  //   if (abs(yaw_rate) > 1e-5){
  //     // Apply equations of motion model (turning)
  //     double theta_pred = p.theta + yaw_rate * delta_t;
  //     p.x += velocity / yaw_rate * (sin(theta_pred) - sin(p.theta));
  //     p.y += velocity / yaw_rate * (cos(theta_pred) - cos(p.theta));
  //   } else {
  //     // Apply equations of motion model (turning)
  //     p.x += velocity * delta_t * cos(p.theta);
  //     p.y += velocity * delta_t * sin(p.theta);
  //   }

  //   //add gaussian noise to the particles
  //   normal_distribution<double> dist_x(p.x, std[0]);
  //   normal_distribution<double> dist_y(p.y, std[1]);
  //   normal_distribution<double> dist_theta(p.theta, std[2]);

  //   //Update particle with noisy prediction
  //   particles[i].x = dist_x(gen);
  //   particles[i].y = dist_y(gen);
  //   particles[i].theta = dist_theta(gen);
  // }


}

// struct LandmarkObs
// {
//   int id;   // Id of matching landmark in the map.
//   double x; // Local (vehicle coords) x position of landmark observation [m]
//   double y; // Local (vehicle coords) y position of landmark observation [m]
// };
/**
   * dataAssociation Finds which observations correspond to which landmarks 
   *   (likely by using a nearest-neighbors data association).
   * update observation id with nearest landmarks within range
   * @param predicted Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   */
void ParticleFilter::dataAssociation(vector<LandmarkObs>& predicted, 
                                     vector<LandmarkObs>& observations) {
  // update observation id with nearest landmarks within range
  //auto automatically infer the type of variable
  for (auto &obs : observations){

    //initialise min_dist with a very big number
    double min_dist = std::numeric_limits<double>::max();

    for (const auto &pred_obs : predicted){
      double d = dist(obs.x, obs.y, pred_obs.x, pred_obs.y);
      if (d < min_dist){
        obs.id = pred_obs.id;
        min_dist = d;
      }
    }
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /*
  for each particle:
    select landmarks within range
    for each observation:
      transform observation into map_coord
      data association
      compute weight
    compute total weight by multiplying all possibilities
    normalise weight
  */

  // std values for landmark
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];

  for (auto &p : particles){
    double particle_likelihood = 1.0;
    //select landmarks within range*************************************
    vector<LandmarkObs> landmarks_within_range;

    for (const auto &map_landmark : map_landmarks.landmark_list)
    {
      double distance = dist(p.x, p.y, map_landmark.x_f, map_landmark.y_f);

      if (distance < sensor_range)
      {
        LandmarkObs l_within_range;
        l_within_range.id = map_landmark.id_i;
        l_within_range.x = map_landmark.x_f;
        l_within_range.y = map_landmark.y_f;
        landmarks_within_range.push_back(l_within_range);
      }
    }

    // for each observation:**********************************************
    //   transform observation into map_coord
    //   data association
    //   compute weight
    vector<LandmarkObs> observations_map_coord;

    for (const auto &obs_car : observations)
    {
      // transform observations into map_coord****************************
      LandmarkObs obs_map;
      obs_map.x = p.x + cos(p.theta)*obs_car.x - sin(p.theta)*obs_car.y;
      obs_map.y = p.y + sin(p.theta)*obs_car.x + cos(p.theta)*obs_car.y;

      // data association, update obs_map id********************************
      // create a very big number for min_dist use
      double min_dist = std::numeric_limits<double>::max();
      double landmark_x, landmark_y; //create x, y for computing weight
      for (const auto &landmark : landmarks_within_range)
      {
        // compute distance
        double dist_between = dist(obs_map.x, obs_map.y, landmark.x, landmark.y);
        if (dist_between < min_dist){
          min_dist = dist_between;
          obs_map.id = landmark.id;
          landmark_x = landmark.x;
          landmark_y = landmark.y;
        }
      }

      //compute weight for this observation**********************************
      double denominator = 2 * M_PI * std_x * std_y;
      double prob = exp(-(pow(obs_map.x - landmark_x, 2) / (2 * std_x * std_x) +
                          pow(obs_map.y - landmark_y, 2) / (2 * std_y * std_y)));
      prob /= denominator;
      observations_map_coord.push_back(obs_map);


      //compute total weight by multiplying all prob************************
      particle_likelihood *= prob;
    }//for each observation

    p.weight = particle_likelihood;

  }//for each particle

  //normalisation**********************************************************
  double norm_factor = 0.0;
  for (const auto &p: particles){norm_factor += p.weight;}
  for (auto &p: particles){
    p.weight /= (norm_factor + std::numeric_limits<double>::epsilon());
    }

}//main function:updateweight

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<double> particle_weights;
  particle_weights.clear();
  for (const auto &p : particles){particle_weights.push_back(p.weight);}

  std::discrete_distribution<int> weighted_dist(particle_weights.begin(),
                                                particle_weights.end());
  
  vector<Particle> resampled_particles;
  for (int i = 0; i < num_particles; ++i){
    int k = weighted_dist(gen);
    resampled_particles.push_back(particles[k]);
  }

  particles.clear();
  particles = resampled_particles;

  //reset all weights
  for (auto &p : particles){p.weight = 1.0;}

}//main function: resample

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