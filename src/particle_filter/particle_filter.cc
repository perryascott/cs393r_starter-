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

#include "amrl_msgs/VisualizationMsg.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"
#include "visualization/visualization.h"

using amrl_msgs::VisualizationMsg;
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;
using visualization::DrawLine;

DEFINE_double(num_particles, 50, "Number of particles");

//using namespace ros_helpers;


namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {
		

	}
	


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

int scanStep = 4;
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
	//const Vector2f zero(0,0);
	for(int g = 0; g< num_ranges; ++g){
		scan[g] = Vector2f(0,0);
	}

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:

  float angleIncrement = (angle_max-angle_min)/(scan.size()-1);
  int scanCount = 0;
  //int index = 0;
  const Vector2f laserOrigin(loc.x() + .2*cos(angle),loc.y()+.2*sin(angle));
  for(size_t j = 0; j< scan.size(); j=j+scanStep){
	float rayAngle = angle_min + (scanCount*angleIncrement);
	float min = range_max;

	  for (size_t i = 0; i < map_.lines.size(); ++i) {
		
		
		const line2f map_line = map_.lines[i];
		//const Vector2f point1(map_line.p0, map_line.
		//VisualizationMsg vis_msg_;
		//DrawLine(map_line.p0,map_line.p1,0x0000FF,vis_msg_);
		//viz_pub_.publish(vis_msg_);
		// The line2f class has helper functions that will be useful.
		// You can create a new line segment instance as follows, for :
	
		line2f rayLine(laserOrigin.x(),laserOrigin.y(), laserOrigin.x()+range_max*cos(angle+rayAngle), laserOrigin.y()+ range_max*sin(angle+rayAngle)); // Line segment from (1,2) to (3.4).
		// Access the end points using `.p0` and `.p1` members:

		
		// Check for intersections:
		bool intersects = map_line.Intersects(rayLine);
		// You can also simultaneously check for intersection, and return the point
		// of intersection:
		Vector2f intersection_point; // Return variable
		intersects = map_line.Intersection(rayLine, &intersection_point);

		if (intersects) {

			float range = dist(intersection_point, laserOrigin);
			if(range < min){
				min = range;
			}

		} 
	  }
	if (min < range_min){
		 min = range_min;
	}
	
	scan[scanCount] = Vector2f(min*cos(rayAngle), min*sin(rayAngle));
	scanCount=scanCount+scanStep;
  }
  

}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
								
								

	vector<Vector2f> prtcl_scan;
	Particle ptcl = *p_ptr;
	//float score = 0;
	ParticleFilter::GetPredictedPointCloud(p_ptr->loc,p_ptr->angle,ranges.size(),range_min,range_max,angle_min,angle_max,&prtcl_scan);
	//float gamma = .5;
	//float var = .01;
	float score = 0;
	//float coeff = gamma*(-1/(2*pow(var,2)))
	const Vector2f origin(0,0);
	for(size_t i=0; i< prtcl_scan.size(); i=i+scanStep){
		float predict_dist = dist(origin, prtcl_scan[i]);
		score += -pow(predict_dist - ranges[i],2);
	}
	if(score == 0){
		ptcl.weight = INT_MIN;
	} else {
	ptcl.weight = 1/score;
	}
	*p_ptr = ptcl;
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
  	vector<Particle> new_particles;
	new_particles.clear();


  	float weight_sum = 0;
	for (size_t i=0; i<particles_.size(); ++i){
		weight_sum += particles_[i].weight;
	}


	int num_particles = 40; //abs(weight_sum*scanStep) / 4;
	float adjusted_weight_sum = abs(weight_sum) / num_particles * scanStep;
	
	float M = abs(weight_sum)/num_particles;
	float r = rng_.UniformRandom(0, M);
	float c = abs(particles_[0].weight);
	float index = 0;

	for(int m= 0; m<num_particles; ++m){
		float U = r + (m*M);
		while(U > c){
			index++;
			c = c + abs(particles_[index].weight);
		}
		float c1 = .0;
		float c2 = .0;
		if(abs(weight_sum) / num_particles*scanStep < .001){
			c1 = .009;
			c2 = .0001;
		}
		else if(abs(weight_sum) / num_particles*scanStep < .5){
			c1 = .015;
			c2 = .00025;
		} else if(abs(weight_sum) / num_particles*scanStep < 4){
			c1 = .05;
			c2 = .025;
		} else if(abs(weight_sum) / num_particles*scanStep < 16.0){
			c1 = .4;
			c2 = .25;
		} else{
			c1 = 1.6;
			c2 = .4;
		}
			float x =  rng_.Gaussian(0, c1) + particles_[index].loc.x();
			float y = rng_.Gaussian(0, c1) + particles_[index].loc.y();
			float angle = rng_.Gaussian(0, c2) + particles_[index].angle;
			const Vector2f new_loc(x,y);
			Particle new_p = {
				new_loc,
				angle,
				1.0/num_particles
			};
		new_particles.push_back(new_p);
	}
	particles_ = new_particles;
	
	ROS_INFO("adjusted_weight_sum = %f", adjusted_weight_sum);
}

int laserObserveCount = 0;
void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  	laserObserveCount++;
  
	if(particles_.size() == 0){
		return;
	}
  

	float weightSum = 0;
	for (size_t i=0; i<particles_.size(); ++i){
		
		ParticleFilter::Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[i]);
		weightSum += particles_[i].weight;
		
	}
	
	//normalize the weights please
	for (size_t i=0; i<particles_.size(); ++i){
		particles_[i].weight = particles_[i].weight/weightSum;
		
	}

	for (size_t i=0; i<particles_.size(); ++i){
		particles_[i].weight = particles_[i].weight/weightSum;
		
	}
	if(laserObserveCount % 2 == 0){
		ParticleFilter::Resample();
	}
	//ROS_INFO("weight_sum: %f", weightSum);
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
	vector<Particle> new_particles;
	new_particles.clear();
	
	
	if(!odom_initialized_){
		prev_odom_loc_ = odom_loc;
		prev_odom_angle_ = odom_angle;
		odom_initialized_ = true;
		return;
	}
	
	//banana distribution values 
	//float k1 = .04;
	//float k2 = .04;
	//float k5 = .2;
	//float k6 = .2;

	float k1 = .03;
	float k2 = .03;
	float k5 = .075;
	float k6 = .075;
	
	float odomDiff = dist(odom_loc,prev_odom_loc_); //distance between odom measurements
	float angleChange = 0;
	if((odom_angle * prev_odom_angle_ < 0)&&(abs(odom_angle) > 3.0)){
		if(odom_angle > 0){
			angleChange = -2*M_PI + odom_angle - prev_odom_angle_;
		} else {
			angleChange = 2*M_PI + odom_angle - prev_odom_angle_;
		}
	} else {
		angleChange = odom_angle - prev_odom_angle_; //mean angle travelled (theta)
	}
	
	//ROS_INFO("odom angle = %f prev ang=  %f",odom_angle,prev_odom_angle_);
	//ROS_INFO("odom x = %f odom y=  %f",odom_loc.x(),odom_loc.y());

	for (size_t i=0; i<particles_.size(); ++i){
	
		//absolute location and angle before propagation
		float v1x = particles_[i].loc.x();
		float v1y = particles_[i].loc.y();
		float v1angle = particles_[i].angle; 

		if(angleChange!=0){
			
			float rad = (odomDiff/2)/sin(abs(angleChange)/2);//estimated radius of curvature
			float s = abs(angleChange)*rad;// mean distance travelled along curve
			float dist = rng_.Gaussian(0, k1*s + k2*abs(angleChange)) + s;
			float deltaAng = rng_.Gaussian(0, k5*s + k6*abs(angleChange));
			
			float odomDeltaX = odom_loc.x() - prev_odom_loc_.x();
			float odomDeltaY = odom_loc.y() - prev_odom_loc_.y();

			float refDeltaX = odomDeltaX*cos(-prev_odom_angle_)-odomDeltaY*sin(-prev_odom_angle_); //rad*sin(abs(angleChange));
			float refDeltaY = odomDeltaY*cos(-prev_odom_angle_)+odomDeltaX*sin(-prev_odom_angle_);//rad*(1-cos(abs(angleChange)));
			float v2x = v1x + refDeltaX*cos(v1angle) - refDeltaY*sin(v1angle); // mean x after propagation
			float v2y = v1y + refDeltaY*cos(v1angle) + refDeltaX*sin(v1angle);//mean y after propagation

			float Xnoise = dist*cos(deltaAng) - s;
			float Ynoise = dist*sin(deltaAng);
			
			float finalAngle = v1angle + angleChange + deltaAng;
			
			const Vector2f newParticlePose(v2x + Xnoise*cos(finalAngle)-Ynoise*sin(finalAngle), v2y + Ynoise*cos(finalAngle) + Xnoise*sin(finalAngle));
			Particle new_p = {
				newParticlePose,
				finalAngle,
				.05,
			};
			new_particles.push_back(new_p);
			
		} else{
			
			//distance traveled and angle traveled calculated using gaussian distribution
			float dist = rng_.Gaussian(0, k1*odomDiff + k2*abs(angleChange)) + odomDiff;
			float deltaAng = rng_.Gaussian(0, k5*odomDiff + k6*abs(angleChange));
			const Vector2f newParticlePose(v1x + dist*cos(deltaAng+v1angle), v1y + dist*sin(deltaAng+v1angle));
			Particle new_p = {
				newParticlePose,
				deltaAng+v1angle,
				.05,
			};
			new_particles.push_back(new_p);
		}
		
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
  particles_.clear();
  map_.Load(map_file);
  odom_initialized_ = false;
  //distribut the particles around the car 
  int numParticles = 40;
  
  /*
  for(int i = 0; i < numParticles; ++i){
	 float px = rng_.Gaussian(0, 10) + loc.x();
	 float py = rng_.Gaussian(0, 10) + loc.y();
	 float pang = angle + rng_.Gaussian(0,M_PI);
	 const Vector2f newPose(px,py);
	 Particle new_p = {
		newPose,
			pang,
			1.0/numParticles,
		};
	particles_.push_back(new_p);
  } */
  
	
    for(int i = 0; i < numParticles; ++i){
	 float px = loc.x();
	 float py = loc.y();
	 float pang = angle;
	 const Vector2f newPose(px,py);
	 Particle new_p = {
		newPose,
			pang,
			1.0/numParticles,
		};
	particles_.push_back(new_p);
	}
	
	
	/*
	float lag = 0;
	 const Vector2f newPose(loc.x() - lag*cos(angle),loc.y() - lag *sin(angle));
	 Particle new_p = {
		newPose,
			angle,
			1.0/numParticles,
		};
	particles_.push_back(new_p);
	*/
}


void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:

  
  	float weight_sum = 0;
	for (size_t i=0; i<particles_.size(); ++i){
		weight_sum += particles_[i].weight;
	}
	float x_mean = 0;
	float y_mean = 0;
	float angle_mean = 0;
	
	for (size_t i=0; i<particles_.size(); ++i){
		x_mean += (particles_[i].loc.x() * particles_[i].weight)/weight_sum;
		y_mean += (particles_[i].loc.y() * particles_[i].weight)/weight_sum;
		angle_mean += (particles_[i].angle * particles_[i].weight)/weight_sum;
	}
	
	loc = Vector2f(x_mean, y_mean);
	angle = angle_mean;
	//ROS_INFO("x_mean: %f, y_mean: %f, angle: %f",x_mean, y_mean, angle_mean);
	//loc = Vector2f(10,10);
	//angle = 0;
	//DrawParticle(loc, angle, vis_msg_);

}


}  // namespace particle_filter
