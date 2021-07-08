//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"

#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;


DEFINE_double(num_particles, 50, "Number of particles");

//using namespace ros_helpers;


namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

//HELPER FUNCTIONS -----------------------------------------

//return magnitude of vector 
float mag(const Vector2f& vect){
    return sqrt(pow(vect.x(),2) + pow(vect.y(),2));
}

//return distance between 2 vectors
float dist(const Vector2f& vect1, const Vector2f& vect2){
	const Vector2f vectDist(vect1.x()-vect2.x(),vect1.y()-vect2.y());
	return mag(vectDist);
}
//------------------------------------------------------------------------

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  float angleStart = -3*M_PI/4;
  float angleIncrement = 3*M_PI/(2*(scan.size()-1));
  int scanCount = 0;
  for(size_t j = 0; j< scan.size(); ++j){
	float rayAngle = angleStart + (j*angleIncrement);
	float min = range_max;

	  for (size_t i = 0; i < map_.lines.size(); ++i) {


		const line2f map_line = map_.lines[i];
		// The line2f class has helper functions that will be useful.
		// You can create a new line segment instance as follows, for :
		line2f rayLine(loc.x(),loc.y(), loc.x()+range_max*cos(angle+rayAngle), loc.y() + range_max*sin(angle+rayAngle)); // Line segment from (1,2) to (3.4).
		// Access the end points using `.p0` and `.p1` members:

		
		// Check for intersections:
		bool intersects = map_line.Intersects(rayLine);
		// You can also simultaneously check for intersection, and return the point
		// of intersection:
		Vector2f intersection_point; // Return variable
		intersects = map_line.Intersection(rayLine, &intersection_point);

		if (intersects) {

			float range = dist(intersection_point, loc);
			if(range < min){
				min = range;
			}

		} 
	  }
	if (min < range_min){
		 min = range_min;
	}
	
	scan[scanCount] = Vector2f(min*cos(rayAngle),min*sin(rayAngle));
	scanCount++;
  }
  

}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}
 
void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
         x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
	vector<Particle> new_particles;
	
	
	//banana distribution values 
	//float k1 = .04;
	//float k2 = .04;
	/*float k3 = 1;
	float k4 = 1;*/
	//float k5 = .2;
	//float k6 = .2;
	
	

	float k1 = .01;
	float k2 = .01;
	/*float k3 = 1;
	float k4 = 1;*/
	float k5 = .05;
	float k6 = .05;
	
	float distance = dist(odom_loc,prev_odom_loc_);
	float angleChange = abs(prev_odom_angle_ - odom_angle);
	//const Vector2f pose1(1,2);
	int n = 50;
	for (int i=0; i<n;i++){
		float x = rng_.Gaussian(distance, k1*distance + k2*angleChange);
		//float y = 0;//rng_.Gaussian(0, k3*distance + k4*angleChange);
		float ang = rng_.Gaussian(0, k5*distance + k6*angleChange);
		//float theta = rng.Gaussian(prev_odom_angle, k
		const Vector2f particlePose(x*cos(ang)+2,x*sin(ang));
		Particle new_p = {
			particlePose,
			ang,
			.05,
		};
		new_particles.push_back(new_p);
		//ROS_INFO("angle bruh= %f",ang);
	}
 
	
	prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
  particles_ = new_particles;
//ROS_INFO("odomx = %f odomy = %f angle = %f", odom_loc.x(), odom_loc.y(), odom_angle);
  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  //float x = rng_.Gaussian(0.0, 2.0);
  //printf("Random number drawn from Gaussian distribution with 0 mean and "
  //       "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  //distribut the particles around the car 
  int numParticles = 200;
  for(int i = 0; i < numParticles; ++i){
	 float px = rng_.Gaussian(0, 4) + loc.x();
	 float py = rng_.Gaussian(0, 4) + loc.y();
	 float pang = angle + rng_.Gaussian(0,M_PI);
	 const Vector2f newPose(px,py);
	 Particle new_p = {
		newPose,
			pang,
			1.0/numParticles,
		};
	particles_.push_back(new_p);
  }
}


void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}


}  // namespace particle_filter
