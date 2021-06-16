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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"


#include <iostream>  
#include <string>     

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using visualization::DrawCross;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawArc;
using visualization::ClearVisualizationMsg;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace


namespace navigation {




Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);

  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
    
}



void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
}
float actualAngle = 0;
static Vector2f actualLocation(0,0);
void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
	actualLocation = loc;
	actualAngle = angle;

}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    robot_loc_ = loc;
    robot_angle_ = angle;
    robot_vel_ = vel;
    robot_omega_ = ang_vel;
}

//point cloud stuff
static vector<Vector2f> points;

int cloudSize = 0;
void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time){
	points = cloud;
/*
    cloudSize = cloud.size();
    for (int i = 0; i < cloudSize; ++i) {
        DrawPoint(cloud.at(i), 0x77fc03, local_viz_msg_);
        viz_pub_.publish(local_viz_msg_);
    }*/
}
float mag(const Vector2f& vect){
    return sqrt(pow(vect.x(),2) + pow(vect.y(),2));
}

float dist(const Vector2f& vect1, const Vector2f& vect2){
	const Vector2f vectDist(vect1.x()-vect2.x(),vect1.y()-vect2.y());
	return mag(vectDist);
}



//vehicle parameters
float l = 0.7; //vehicle length
float w = 0.4; //vehicle width
//float b = 0; //vehicle wheel base length
//float d = 0; //track width
//float m = 0.05; //obstacle safety margin




 //time optimal control vars
float maxa = 12; //max acceleration m/s^2
float maxd = 4; //max deceleration m/s^2
float vmax = .05; //max speed m/s

float deltaT = .05; //time step between calls of Run()
float goalDistance = 10; //total distance to travel in x direction
float minStopDist = 0; 
int runCount = 0;
float startOdomY = 0;
float startOdomX = 0;
float currentOdomY = 0;
float currentOdomX = 0;
float velMag = 0;
float absDiffX = 0;
float absDiffY = 0;
float magDiff = 0;
float s = 0;
float phi = 0;
float r = 0;
const Vector2f zeroLocation(0,0);

float maxMagDiff = -1;

float isosGetAngle(const Vector2f& start, const Vector2f& end,float obstR){
	const Vector2f vect(start.x()-end.x(),start.y()-end.y());
	float startEndMag = mag(vect);
	return 2*asin(startEndMag/(2*obstR));
}

float getScanAngle(const Vector2f& obst, float baseLinkr, float pointRad){
	const Vector2f origin(0,pointRad-baseLinkr);
    if(obst.x() <0){
		return 2*M_PI - isosGetAngle(origin,obst,pointRad);
	}
	else{
		return isosGetAngle(origin,obst,pointRad);
	}
}

void plotLinkArc(float curve, float angle_diff,int color, float blOffset){
	
	float rad=abs(1/curve);
	float rSign = 1;
	if (curve<0){rSign = -1;}
	const Vector2f center(actualLocation.x() - rSign*rad*sin(actualAngle),actualLocation.y() + rSign*rad*cos(actualAngle));
	float start_angle = atan2(actualLocation.y()-center.y(),actualLocation.x()-center.x());
	if(curve<0){
		DrawArc(center, r + blOffset,start_angle+rSign*angle_diff,start_angle,color,local_viz_msg_) ;
	}
	else{
		DrawArc(center, r +blOffset,start_angle,start_angle+rSign*angle_diff,color,local_viz_msg_) ;
	}    
}

void plotLinkArcOrigin(float curve, float angle_diff,int color, float blOffset){
	
	float rad=abs(1/curve);
	float rSign = 1;
	if (curve<0){rSign = -1;}
	const Vector2f center( 0,  rSign*rad);
	float start_angle = atan2(-center.y(),-center.x());
	if(curve<0){
		DrawArc(center, r + blOffset,start_angle+rSign*angle_diff,start_angle,color,local_viz_msg_) ;
	}
	else{
		DrawArc(center, r +blOffset,start_angle,start_angle+rSign*angle_diff,color,local_viz_msg_) ;
	}    
}

void plotCar(int color){
	const Vector2f bl(actualLocation.x()-w/2*sin(actualAngle),actualLocation.y()+w/2*cos(actualAngle));
	const Vector2f br(actualLocation.x()+w/2*sin(actualAngle),actualLocation.y()-w/2*cos(actualAngle));
	const Vector2f fl(actualLocation.x()-w/2*sin(actualAngle)+l*cos(actualAngle),actualLocation.y()+w/2*cos(actualAngle)+l*sin(actualAngle));
	const Vector2f fr(actualLocation.x()+w/2*sin(actualAngle)+l*cos(actualAngle),actualLocation.y()-w/2*cos(actualAngle)+l*sin(actualAngle));
	DrawLine(bl,br, color,local_viz_msg_);
	DrawLine(bl,fl, color,local_viz_msg_);
	DrawLine(br,fr, color,local_viz_msg_);
	DrawLine(fl,fr, color,local_viz_msg_);
	DrawCross(actualLocation, .2, color,local_viz_msg_);

}

void Navigation::Run() {
	ClearVisualizationMsg(local_viz_msg_);
    viz_pub_.publish(local_viz_msg_);
	
    drive_msg_.curvature = .4;
	float curve = drive_msg_.curvature;
	r = abs(1/drive_msg_.curvature);
	

	
    if (runCount < 8){
	startOdomY = robot_loc_.y();
	startOdomX = robot_loc_.x();
	runCount++;
    const Vector2f startLocation(actualLocation.x(),actualLocation.y());

	ROS_INFO("initializing starting odometry");
    } 
    
    //calculate distance traveled and velocity magnitude
    currentOdomY = robot_loc_.y();
    currentOdomX = robot_loc_.x();
    absDiffX = abs(startOdomX - currentOdomX);
    absDiffY = abs(startOdomY - currentOdomY);
    magDiff = sqrt(pow(absDiffX,2)+pow(absDiffY,2));
    velMag = mag(robot_vel_);

	//if driving straight, arc length is simply distance traveled
	
	if(drive_msg_. curvature == 0){
		phi = 0;
		s = magDiff;
	}
	//if operating on a curve, determing arc length moved along curve
	else{			
			if(magDiff >= maxMagDiff){
				maxMagDiff = magDiff;
				phi = 2*asin(magDiff/(2*r));
				s = abs((phi)*r);
			} 
			else{
				phi = M_PI - 2*asin(magDiff/(2*r));
				s = abs((phi+M_PI)*r);
			}
	}

	//if at max speed
	if(velMag >= vmax){
		//first make sure enough room to decelerate, if not then stop.
		minStopDist = pow(velMag,2)/(2*maxd);
		if((goalDistance - s - deltaT*velMag) <= minStopDist){
			drive_msg_.velocity = 0;
			ROS_INFO("slowing down");
		}
		//otherwise remain at top speed
		else{
			drive_msg_.velocity = vmax;
			ROS_INFO("remain at top speed");
		}
	} 
	else {
		//if not at max speed
		//assume accerating and ask if at next time step there is sufficient room to stop. If not then stop
		minStopDist = pow((velMag+deltaT*maxa),2)/(2*maxd);
		if((goalDistance- s - deltaT*velMag-1/2*maxa*pow(deltaT,2)) <= minStopDist){
			drive_msg_.velocity = 0;
			ROS_INFO("slowing down");
		}

		//otherwise accelerate
		else {
			drive_msg_.velocity = vmax;
			ROS_INFO("speeding up");
		}
	}
     
	//obstacle detection
	//float distLeft = goalDistance - s;
	float rSign = 1;
	if(drive_msg_.curvature <0 ){ rSign = -1;}
	
	//float strAng = abs(atan(drive_msg_.curvature*l));
	//float p = w*cos(strAng) + l*sin(strAng);
	float innerRad = r - w/2;
	float outerRad = sqrt(pow(r+w/2,2)+pow(l,2));
	float p = outerRad - innerRad;
	
	//float f = l*sin(strAng);
	//float shortestPath = 0;
	//for curved obstacle detection
	if(curve!=0){
		//calculate steering angle 

		int cloudSize = points.size();
		//ROS_INFO("cloud size is: %i",cloudSize);
		const Vector2f centerRef(0,rSign*r);
		static vector<Vector2f> rel_points;
		float minObstAngle = 2*M_PI;
		//float maxPointRad = 0;
		float pointRad = 0;
		for(int i =0; i<cloudSize; i++){
			//const Vector2f point(points[i].x(),points[i].y());
			pointRad = dist(points.at(i),centerRef);
			
			//need to fix this  ^^^ calculate magnitude of points.at(i) to center of rotation
			//ROS_INFO("point x: %f y: %f",points.at(i).x(),points.at(i).y());
			//ROS_INFO("i: %i",i);
			
			if((pointRad >= innerRad)&&(pointRad<= outerRad)){
				float obstAngle = getScanAngle(points.at(i),r,pointRad);
				if(obstAngle < minObstAngle){
					minObstAngle = obstAngle;
					//maxPointRad = pointRad;
				}
				
				rel_points.push_back(points.at(i));
				const Vector2f relPoint(points.at(i).x()*cos(actualAngle)-points.at(i).y()*sin(actualAngle)+actualLocation.x(),points.at(i).y()*cos(actualAngle)+points.at(i).x()*sin(actualAngle)+actualLocation.y());
				DrawPoint(points.at(i), 0x6A0DAD, local_viz_msg_);
				DrawPoint(relPoint, 0x6A0DAD, local_viz_msg_);
				
				
				//viz_pub_.publish(local_viz_msg_);
				//ROS_INFO("point x: %f y: %f",points.at(i).x(),points.at(i).y());
			}
		}
		//float angle_diff = (distLeft)/r;
		plotLinkArc(drive_msg_.curvature, minObstAngle, 0x77fc03, 0);
		plotLinkArcOrigin(drive_msg_.curvature, minObstAngle, 0x77fc03, 0);

		
	}
   
    //output information on console
	ROS_INFO("Inner: %f, Outer: %f, P: %f",innerRad,outerRad,p);
    ROS_INFO("Distance Traveled: %f",s);
    ROS_INFO("Velocity: %f",mag(robot_vel_));
	ROS_INFO("Phi: %f",phi);
	ROS_INFO("True-Location(%f,%f), Angle(%f)",actualLocation.x(),actualLocation.y(),actualAngle);
    ROS_INFO("--------------");
    drive_pub_.publish(drive_msg_);
	
    //draw the curve or line that the bot is following

	//plot remaining path to traverse
	/*
	if(drive_msg_.curvature !=0){
		float angle_diff = (distLeft)/r;
		plotLinkArc(drive_msg_.curvature, angle_diff, 0x77fc03, 0);
	}
	//if moving in a straight line
	else{
		const Vector2f end(actualLocation.x() + distLeft*cos(actualAngle),actualLocation.y() + distLeft*sin(actualAngle));
		DrawLine(actualLocation, end, 0x77fc03,local_viz_msg_);
	}*/

	//draw the actual bot
	plotCar(0x0000ff);
	viz_pub_.publish(local_viz_msg_);


}  // namespace navigation

}
