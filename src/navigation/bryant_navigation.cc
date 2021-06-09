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
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "math.h"
// #include "iostream.h"

#include "sensor_msgs/LaserScan.h"
/*● Minimum angle
● Maximum angle
● Number of ray / angular resolution
● Minimum range
● Maximum range
Data: List of ranges
 */

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using visualization::DrawCross;
using visualization::DrawPoint;
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
    nav_goal_angle_(0),
    time_(0) {
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
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    int cloudSize = cloud.size();
    for (int i = 0; i < cloudSize; ++i) {
        DrawPoint(cloud.at(i), 0x77fc03, local_viz_msg_);
        viz_pub_.publish(local_viz_msg_);
    }
}

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
   float laserLim = scan->range_max;
}

void LaserCallback(const sensor_msgs::Range::ConstPtr& scan)
{
   float FoV = scan->field_of_view;
}

void Navigation::Run() {
    double L = ; // length
    double w = ; // width
    double b = ; // whell base
    double d = ; // track width
    double m = ; // margin
    double xObst[]={}; // x-axis for obstacle
    double yObst[]={}; // y-axis for obstacle
    double xInt = 3.0; // x-axis of waypoint ahead of car
    double minCurv=-1.0; // curvature to right
    double maxCurv=1.0; // curvature to left
    double curvIncrement=0.05;
    for (double i=minCurv; i<=maxCurv; i+=curvIncrement){  // generate curves
        double r=1/i; // turning radius
        double angle = atan(xInt/r);
        double arcIntLim = r*angle; // uper bond for closest path to the waypoint along curve
        
        if (laserLim<2*r){ //find laser sensor range limit
            double angleLaserLim = acos((r^2+r^2-laserLim^2)/(2*r*r)); // law of cosine
            double arcLaserLim = r*angleLaserLim; // limit arc length due to sensor range
        }
        else {
            double arcLaserLim = 2*M_PI*r; // if the sensor range is bigger than twice of the turning radius, there is no limit
        }
        
        double arcFoVLim = r*FoV; // upper bond from field of view
        double dArcUB = min(arcLaserLim, arcFoVLim, arcIntLim); // constraint the curve with upper bonds listed above
        
        int arrSize=sizeof(xObst)/sizeof(xObst[0]);
        double dArc[arrsize-1];
        for (int i=0; i<arrSize; i++){ //iterate through each point from LIDAR
            double dObst=sqrt(xObst[i]^2+(r-yObst[i])^2); // distance from center of turning to the opstacle point
            if ((r-w/2-m)<dObst<sqrt((r-w/2-m)^2+((L+b)/2+m)^2)){ // inner vehicle
                dArc[i] = r*(M_PI_2-atan((r-yObst[i])/xObst[i])-acos((r-m-w/2)/sqrt(xObst[i]^2+(r-yObst[i])^2)));
            }
            else if (sqrt((r-w/2-m)^2+((L+b)/2+m)^2)<dObst<sqrt((r+w/2+m)^2+((L+b)/2+m)^2)){ // front vehicle
                dArc[i] = r*(M_PI_2-atan((r-yObst[i])/xObst[i])-asin(((L+b)/2+m)/sqrt(xObst[i]^2+(r-yObst[i])^2)));
            }
            else if (dObst<sqrt((r+w/2+m)^2+((L-b)/2+m)^2) && yObst[i]<-w/2-m){ // outer vehicle
                dArc[i] = r*(atan((r-yObst[i])/xObst[i])+acos(r/sqrt(xObst[i]^2+(r-yObst[i])^2)));
            }
            else { // no collision
                dArc[i] = 2*M_PI*r;
            }
            double dArcMin = *min_element(dArc, dArc+arrSize); // find the minimum path length among each obstacle point
            double arcFree = min(dArcMin, dArcUB); // take into account the upper bond constraint
            
            
        }
            
        }
        
        
    
        
    }

  double a = 4.0; //4 m/s^2 acceleration
  double d = -4.0; // 4 m/s^ deceleration
  double vmax = 1.0; //1 m/s cruising speed
  double x1 = pow(vmax, 2)/(2*a);
  double x3 = pow(vmax, 2)/(2*-1*d);
  double xtot = 2.0;
  double x2 = xtot - x1 - x3;
  
  double t1 = vmax/a;
  double t2 = (xtot - x1 - x3)/vmax;
  double t3 = -1*vmax/d; //t1 = time for acceleration phase, t2 = time for cruise phase, t3 = time for deceleration phase
  double v0 = 0.0;
  double x0 = 0.0;
    
//    DrawPathOption(1, 1, 0.75, local_viz_msg_);
//    viz_pub_.publish(local_viz_msg_);
  
	if(robot_vel_.x() < vmax && robot_loc_.x() < x1){ // acceleration phase of TOC
    time_ += 0.05;
		robot_vel_.x() = a*t1 + v0;
		robot_loc_.x() = 0.5*a*pow(t1, 2.0) + v0*t1 + x0;
		drive_msg_.velocity = robot_vel_.x();
		drive_msg_.curvature = 0.0;
		drive_pub_.publish(drive_msg_);
	}

	else if(robot_loc_.x() < (x1+x2)){ // cruise phase of TOC
		time_ += .05;
		robot_vel_.x() = vmax;
		robot_loc_.x() = vmax*(time_ - t1) + x1;		
		drive_msg_.velocity = robot_vel_.x();
		drive_msg_.curvature = 0.0;
		drive_pub_.publish(drive_msg_);
	}

	else if(robot_loc_.x() < xtot){ // deceleration phase of TOC
		time_ += 0.05;
    robot_vel_.x() = d*t3 + vmax;
		robot_loc_.x() = 0.5*d*pow(time_ - t1 - t2, 2)+ vmax*(time_ - t1 - t2) + x1 + x2;
		drive_msg_.velocity = robot_vel_.x();
		drive_msg_.curvature = 0.0;
		drive_pub_.publish(drive_msg_);
	}
    else{ // stopping phase of TOC
    robot_vel_.x() = 0.0;
    drive_msg_.velocity = robot_vel_.x();
    drive_msg_.curvature = 0.0;
    drive_pub_.publish(drive_msg_);
    }
    
    ClearVisualizationMsg(local_viz_msg_);

  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.

}  // namespace navigation
