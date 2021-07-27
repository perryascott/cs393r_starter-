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
#include "shared/math/geometry.h"

#include <iostream>  
#include <string>     
#include <queue>

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
using geometry::line2f;
using std::map;
using std::queue;

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



float goalAngle = 0;
static Vector2f goalLocation(0,0);
//vector_map::VectorMap map_;
vector_map::VectorMap map_ = vector_map::VectorMap("maps/GDC2.txt");

float actualAngle = 0;
static Vector2f actualLocation(0,0);

/*
int map_width_left = 22*2; //meters
int map_width_right = 22*2; //meters
int map_height_top = 11*2;//meters
int map_height_bottom = 18*2;//meters 
*/

int map_width_left = 2*2; //meters
int map_width_right = 2*2; //meters
int map_height_top = 2*2;//meters
int map_height_bottom = 2*2;//meters


float map_res = .25; //meters

float start_x = 0;
float start_y = 0;
int start_index = 0;

int row_length = (map_width_left + map_width_right )/map_res;
int col_height = (map_height_top + map_height_bottom )/map_res;

map<int, int> came_from = {{start_index, -1}}; //node #, node # it came from 

void plotCameFrom(int node1, int node2, int color){

	const Vector2f vect1((node1 % row_length)*map_res - map_width_left+map_res/2,map_height_top - (node1 / row_length)*map_res - map_res/2);
	const Vector2f vect2((node2 % row_length)*map_res - map_width_left+map_res/2,map_height_top - (node2 / row_length)*map_res - map_res/2);
	DrawLine(vect1,vect2, color,local_viz_msg_);
	return;
}

//GRAPH SEARCH -------------------------------------------------------------------------------
float nav_x = 0;
float nav_y = 0;
int nav_index = 0;

void addEdge(vector<int> graph[], int u, int v){
	graph[u].push_back(v);
	graph[v].push_back(u);
	return;
}

bool checkObstacle(const Vector2f& vect1, const Vector2f& vect2){
	
	line2f rayLine(vect1.x(),vect1.y(), vect2.x(),vect2.y());
	
	for (size_t i = 0; i < map_.lines.size(); ++i) {
		const line2f map_line = map_.lines[i];
		bool intersects = map_line.Intersects(rayLine);

		Vector2f intersection_point; 
		intersects = map_line.Intersection(rayLine, &intersection_point);

		if (intersects) {
			return false;
		} 
	}
	return true;
}


void selectionSort(int proximity[], int neighbor[], int size){

	for(int i = 0; i < size; ++i){
		int min_index = i;
		int min_proximity = proximity[i];
		int min_proximity_neighbor = neighbor[i];
		for(int j = i+1; j< size; ++j){
			if(proximity[j] < min_proximity){
				min_proximity_neighbor = neighbor[j];
				min_proximity = proximity[j];
				min_index = j;
			}
		}
		proximity[min_index] = proximity[i];
		proximity[i] = min_proximity;
		neighbor[min_index] = neighbor[i];
		neighbor[i] = min_proximity_neighbor;

	}
}
	
float hueristic(int node_index){
	float x = (node_index % row_length)-(nav_index % row_length);
	float y = (node_index / row_length) - (nav_index/ row_length);
	return sqrt(pow(x, 2) + pow(y, 2));
}

void BFS(){

	int row_length = (map_width_left + map_width_right )/map_res;
	int col_height = (map_height_top + map_height_bottom )/map_res;
	
	int size = col_height * row_length;
	
	vector<int> graph[size];
	map_.Load( "maps/GDC2.txt");
	//create graph 
	for(int i = 0; i < size; i++){
		const Vector2f point1(((i%row_length)-map_width_left/map_res)*map_res + map_res/2, (map_height_top/map_res - (i/row_length))*map_res - map_res/2);
		//above 
		if(i >= row_length){
			int i2 = i - row_length;
			const Vector2f point2(((i2%row_length)-map_width_left/map_res)*map_res + map_res/2, (map_height_top/map_res - (i2/row_length))*map_res - map_res/2);
			if(checkObstacle(point1,point2)){
				addEdge(graph, i, i2);
				//DrawLine(point1,point2, 0x77fc03,local_viz_msg_);
			}
		}
		//below
		if((i/row_length) != (col_height - 1)){
			int i2 = i + row_length;
			const Vector2f point2(((i2%row_length)-map_width_left/map_res)*map_res + map_res/2, (map_height_top/map_res - (i2/row_length))*map_res - map_res/2);
			if(checkObstacle(point1,point2)){
				addEdge(graph, i, i2);
				//DrawLine(point1,point2, 0x77fc03,local_viz_msg_);
			}
		}
			
		//left of
		if(i % row_length != 0){
			int i2 = i - 1;
			const Vector2f point2(((i2%row_length)-map_width_left/map_res)*map_res + map_res/2, (map_height_top/map_res - (i2/row_length))*map_res - map_res/2);
			if(checkObstacle(point1,point2)){
				addEdge(graph, i, i2);
				//DrawLine(point1,point2, 0x77fc03,local_viz_msg_);
			}
		}
		//right of 
		if((i % row_length != row_length - 1)){
			int i2 = i + 1;
			const Vector2f point2(((i2%row_length)-map_width_left/map_res)*map_res + map_res/2, (map_height_top/map_res - (i2/row_length))*map_res - map_res/2);
			if(checkObstacle(point1,point2)){
				addEdge(graph, i, i2);
				//DrawLine(point1,point2, 0x77fc03,local_viz_msg_);
			}
		}
		
		//left and above
		if(i >= row_length && i % row_length != 0){
			int i2 = i - 1 - row_length;
			const Vector2f point2(((i2%row_length)-map_width_left/map_res)*map_res + map_res/2, (map_height_top/map_res - (i2/row_length))*map_res - map_res/2);
			if(checkObstacle(point1,point2)){
				addEdge(graph, i, i2);
				//DrawLine(point1,point2, 0x77fc03,local_viz_msg_);
			}
		}
		//above and right
		if((i % row_length != row_length - 1)&&(i >= row_length)){
			int i2 = i + 1 - row_length;
			const Vector2f point2(((i2%row_length)-map_width_left/map_res)*map_res + map_res/2, (map_height_top/map_res - (i2/row_length))*map_res - map_res/2);
			if(checkObstacle(point1,point2)){
				addEdge(graph, i, i2);
				//DrawLine(point1,point2, 0x77fc03,local_viz_msg_);
			}
		}	
		
	}

	map<int, float> queueOrder = {{start_index, hueristic(start_index)}}; //node # and proximity
	ROS_INFO("proximity = %f", hueristic(start_index));
	//queue<int> frontier;
	//frontier.push(start_index);
	came_from[start_index] = -1;

	while(!queueOrder.empty()){

		
		int current_node = frontier.front();
		frontier.pop();
		if(current_node == nav_index){
			break;
		}
		//ROS_INFO("current node = %i",current_node);
		for (int i : graph[current_node]){
			if(came_from.find(i) == came_from.end()){
				frontier.push(i);
				came_from[i] = current_node;
				plotCameFrom(i,current_node, 0x77fc03);
			}
		}
		
	}
	return;
}

void Navigation::UpdateTruePose(const Eigen::Vector2f& loc, float angle) {
	if(loc != actualLocation){
		actualLocation = loc;
		actualAngle = angle;
		start_x = round((actualLocation.x() +map_width_left  - map_res/2) / map_res);
		start_y = round((map_height_top - actualLocation.y() + map_res/2)/ map_res ) -1;
		start_index = start_y * row_length + start_x;
		//const Vector2f quantLoc(start_x*map_res - map_width_left+map_res/2,map_height_top - start_y*map_res  - map_res/2 );
		const Vector2f quantLoc((start_index % row_length)*map_res - map_width_left+map_res/2,map_height_top - (start_index / row_length)*map_res - map_res/2);
		DrawCross(quantLoc, .1, 0xFF00FF,local_viz_msg_);
		came_from.clear();
		ClearVisualizationMsg(local_viz_msg_);
		ROS_INFO("helo");
		BFS();

	}
	

	

}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
	goalAngle = angle;
	goalLocation = loc;
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
	
	nav_x = round((goalLocation.x() +map_width_left  - map_res/2) / map_res);
	nav_y = round((map_height_top - goalLocation.y() + map_res/2)/ map_res ) -1;
	nav_index = nav_y * row_length + nav_x;
	//const Vector2f quantLoc(start_x*map_res - map_width_left+map_res/2,map_height_top - start_y*map_res  - map_res/2 );
	const Vector2f quantLoc((nav_index % row_length)*map_res - map_width_left+map_res/2,map_height_top - (nav_index / row_length)*map_res - map_res/2);
	DrawCross(quantLoc, .1, 0xFFFF00,local_viz_msg_);

	int i = nav_index;
	while(came_from[i] != -1){

		plotCameFrom(i, came_from[i], 0x000000);
		i = came_from[i];
	}
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {

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

}

//PARAMETERS ---------------------------------------------------------


//vehicle parameters
float l = 0.5461; //vehicle length
float w = 0.2794; //vehicle width
float b = 0.3175; //vehicle wheel base length
//float d = 0; //track width
float m = 0.05; //obstacle safety margin
float vl = l/2+b/2 + m; //the "virtual length" used for obstacle detection
float vw = w + 2*m; //the "virutal width" used for obstacle detection
float maxC = .4; // maximum clearance bound. clearance larger than this is not considered.
float rearL = l/2-b/2;
float frontL = l/2 + b/2;

 //time optimal control vars
float maxa = 12; //max acceleration m/s^2
float maxd = 4; //max deceleration m/s^2
float vmax = .5;//.5; //max speed m/s


float deltaT = .05; //time step between calls of Run()

float minStopDist = 0; 
int runCount = 0;
float startOdomY = 0;
float startOdomX = 0;


float r = 0;
const Vector2f zeroLocation(0,0);
float rSign = 1;

float maxMagDiff = -1;
float distLeft = 10;

//HELPER FUNCTIONS---------------------------------------------------------------

//return magnitude of vector 
float mag(const Vector2f& vect){
    return sqrt(pow(vect.x(),2) + pow(vect.y(),2));
}

//return distance between 2 vectors
float dist(const Vector2f& vect1, const Vector2f& vect2){
	const Vector2f vectDist(vect1.x()-vect2.x(),vect1.y()-vect2.y());
	return mag(vectDist);
}

int getMaxIndex(float arr[30],int count){
	float max = INT_MIN;
	float index = 0;
	float counter = 0;
	for(int i = 0; i < 30; i++){
		if(counter >= count){
			break;
		}
		if(arr[i]>max){
			max = arr[i];
			index = i;
		}
		counter++;
	}
	return index;
}

//return angle of isoseles triangle using vector start and end (the two that have same angle in a isoseles)
//and their radius to the 3rd point
float isosGetAngle(const Vector2f& start, const Vector2f& end,float obstR){
	const Vector2f vect(start.x()-end.x(),start.y()-end.y());
	float startEndMag = mag(vect);
	return 2*asin(startEndMag/(2*obstR));
}


//returns angle between reference vector obst and origin using baselink radius and point radius 
float getScanAngle(const Vector2f& obst, float baseLinkr){
	
	if(rSign == 1){
		if(obst.x() <0){
			return 2*M_PI + atan2(obst.x(),(obst.y() - rSign*baseLinkr)*-1);
		}
		else{
			return atan2(obst.x(),(obst.y() - rSign*baseLinkr)*-1);
		}
	}
	else{
		if(obst.x() <0){
			return M_PI - atan2(obst.x(),(obst.y() - rSign*baseLinkr)*-1);
		}
		else{
			return M_PI - atan2(obst.x(),(obst.y() - rSign*baseLinkr)*-1);
		}
	}
	
}

float * getScanVector(float angle, float rad){
	static float scan[2];
	scan[0] = rad*sin(angle); //x 
	scan[1] = rad*(1-cos(angle)); //y
	if(rSign == -1){
		scan[1] = -scan[1];
	}

	return scan;
}



//PLOTTER FUNCTIONS----------------------------------------------------

//plots arc from base link using curve curvature, rCent as the center of rotation radius, angle_diff as length
void plotLinkArc(float curve, float rCent, float angle_diff,int color){
	
	float rad=abs(1/curve);
	if(curve <0 ){ rSign = -1;}else{rSign = 1;}
	const Vector2f center(actualLocation.x() - rSign*rCent*sin(actualAngle),actualLocation.y() + rSign*rCent*cos(actualAngle));
	float start_angle = atan2(actualLocation.y()-center.y(),actualLocation.x()-center.x());
	if(curve<0){
		DrawArc(center, rad ,start_angle-angle_diff,start_angle,color,local_viz_msg_) ;
	}
	else{
		DrawArc(center, rad ,start_angle,start_angle+angle_diff,color,local_viz_msg_) ;
	}    
}

//plots arc from front edge using curve curvature, rCent as the center of rotation radius, angle_dist as length
// and ang_offset as angle from baselink to the leading edge
void plotObstArc(float curve, float rCent, float ang_dist,float ang_offset,int color){
	
	float rad=abs(1/curve); 

	const Vector2f center(actualLocation.x() - rSign*rCent*sin(actualAngle),actualLocation.y() + rSign*rCent*cos(actualAngle));
	float start_angle = atan2(actualLocation.y()-center.y(),actualLocation.x()-center.x());

	if(curve<0){
		DrawArc(center, rad ,start_angle - (ang_dist+ang_offset),start_angle - (ang_offset),color,local_viz_msg_) ;
	}
	else{
		DrawArc(center, rad ,start_angle+(ang_offset),start_angle+(ang_dist+ang_offset),color,local_viz_msg_) ;
	}  
}

//plots arc from front edge using curve curvature, rCent as the center of rotation radius, angle_dist as length
// and ang_offset as angle from baselink to the leading edge
void plotObstArcOrigin(float curve, float rCent, float ang_dist,float ang_offset,int color){
	
	float rad=abs(1/curve); 

	const Vector2f center(0, rSign*rCent);
	float start_angle = atan2(-center.y(),-center.x());

	if(curve<0){
		DrawArc(center, rad ,start_angle - (ang_dist+ang_offset),start_angle - (ang_offset),color,local_viz_msg_) ;
	}
	else{
		DrawArc(center, rad ,start_angle+(ang_offset),start_angle+(ang_dist+ang_offset),color,local_viz_msg_) ;
	}  
}

void plotCar(int color1,int color2){
	
	//plot virtual components for obstacle detection
	const Vector2f bvl(actualLocation.x()-vw/2*sin(actualAngle)-(rearL+m)*cos(actualAngle),actualLocation.y()+vw/2*cos(actualAngle)-(rearL+m)*sin(actualAngle));
	const Vector2f bvr(actualLocation.x()+vw/2*sin(actualAngle)-(rearL+m)*cos(actualAngle),actualLocation.y()-vw/2*cos(actualAngle)-(rearL+m)*sin(actualAngle));
	const Vector2f fvl(actualLocation.x()-vw/2*sin(actualAngle)+vl*cos(actualAngle),actualLocation.y()+vw/2*cos(actualAngle)+vl*sin(actualAngle));
	const Vector2f fvr(actualLocation.x()+vw/2*sin(actualAngle)+vl*cos(actualAngle),actualLocation.y()-vw/2*cos(actualAngle)+vl*sin(actualAngle));
	DrawLine(bvl,bvr, color1,local_viz_msg_);
	DrawLine(bvl,fvl, color1,local_viz_msg_);
	DrawLine(bvr,fvr, color1,local_viz_msg_);
	DrawLine(fvl,fvr, color1,local_viz_msg_);
	DrawCross(actualLocation, .1, color2,local_viz_msg_);
	//plot actual vehicle dimensions
	const Vector2f bl(actualLocation.x()-w/2*sin(actualAngle)-rearL*cos(actualAngle),actualLocation.y()+w/2*cos(actualAngle)-rearL*sin(actualAngle));
	const Vector2f br(actualLocation.x()+w/2*sin(actualAngle)-rearL*cos(actualAngle),actualLocation.y()-w/2*cos(actualAngle)-rearL*sin(actualAngle));
	const Vector2f fl(actualLocation.x()-w/2*sin(actualAngle)+frontL*cos(actualAngle),actualLocation.y()+w/2*cos(actualAngle)+frontL*sin(actualAngle));
	const Vector2f fr(actualLocation.x()+w/2*sin(actualAngle)+frontL*cos(actualAngle),actualLocation.y()-w/2*cos(actualAngle)+frontL*sin(actualAngle));
	DrawLine(bl,br, color2,local_viz_msg_);
	DrawLine(bl,fl, color2,local_viz_msg_);
	DrawLine(br,fr, color2,local_viz_msg_);
	DrawLine(fl,fr, color2,local_viz_msg_); 

}


//TIME OPTIMAL CONTROL ---------------------------------------------------

void TOC(float dist, float velMag){
	
	//get current robot velocity from odometry
	
	//if at max speed
	if(velMag >= vmax){
		//first make sure enough room to decelerate, if not then stop.
		minStopDist = pow(velMag,2)/(2*maxd);
		if((dist - deltaT*velMag) <= minStopDist){
			drive_msg_.velocity = 0;
			//ROS_INFO("slowing down");
		}
		//otherwise remain at top speed
		else{
			drive_msg_.velocity = vmax;
			//ROS_INFO("remain at top speed");
		}
	} 
	else {
		//if not at max speed
		//assume accerating and ask if at next time step there is sufficient room to stop. If not then stop
		minStopDist = pow((velMag+deltaT*maxa),2)/(2*maxd);
		if((dist - deltaT*velMag-1/2*maxa*pow(deltaT,2)) <= minStopDist){
			drive_msg_.velocity = 0;
			//ROS_INFO("slowing down");
		}

		//otherwise accelerate
		else {
			drive_msg_.velocity = vmax;
			//ROS_INFO("speeding up");
		}
	}
}

//OBSTACLE DETECTION -------------------------------------------------------------

float * ShortestPathOD(float curve,bool plot, int color){
	
	//# of points in scan
	int cloudSize = points.size(); 
	
	//create return structure
	static float obArr[3];

	float minDist = 0;
	float minClearance = maxC;

	
	float min_obst_angle = 3*M_PI/2; //shortest path in the scan array in [rads]
	float r = 0;
	float endGoalDist = 0;
	
	//if traveling along a curve
	if(curve!=0){
		r = abs(1/curve);
		
		//used to compensate for negative curvature (right turns)
		if(curve <0 ){ rSign = -1;}else{rSign = 1;}
		
		//calculate inner and outer radii of turning using vehicle geometry
		float innerRad = r - vw/2;
		float outerRad = sqrt(pow(r+vw/2,2)+pow(vl,2));
		
		//reference vector to center of rotation
		const Vector2f centerRef(0,rSign*r); 

		// corner of car calcs
		const Vector2f k(vl,rSign*vw/2); //reference vector for the front inside corner of car
		float kRad = dist(centerRef,k);// radius of this corner
		float kAngle = getScanAngle(k,r);
		const Vector2f t(vl,rSign*(vw/2+maxC));
		//float tRad = dist(centerRef,t); // 
		float tAngle = getScanAngle(t,r);

		//vars
		float pointRad = 0; //radius that an obstacle point makes with centerRef
		float angle_offset = 0; //offset that depends on where obstacle intersects with car
		float raw_obst_angle = 0;
		bool isObst = false;
		
		//vars for holding shortest path properties
		float minPointRad = r; //the radius at which this shortest path occured
		float min_angle_offset = asin(vl/r);//offset for shortest path (for plotting)
		
		const Vector2f refGoal(goalLocation.x() - actualLocation.x(),goalLocation.y()-actualLocation.y());
		const Vector2f turnedRefGoal(refGoal.y()*sin(actualAngle)+refGoal.x()*cos(actualAngle),-refGoal.x()*sin(actualAngle)+refGoal.y()*cos(actualAngle));
		//float goalAngle = getScanAngle(turnedRefGoal,r);		

		//DrawLine(bl, turnedRefGoal, 0xd034d4, local_viz_msg_);
		
		for(int i =0; i<cloudSize; i++){
			
			//calculate radius of point from center of rotation
			pointRad = dist(points.at(i),centerRef);
			
			//if within the radii of rotation, point is within path of movement for car
			if((pointRad >= innerRad)&&(pointRad<= outerRad)){
				
				//find the angle between baselink and obstacle about center of rotation
				raw_obst_angle = getScanAngle(points.at(i),r);
				
				//depending on the obstacles radius, account for angle created between baselink and point of intersection with the vehicle
				if (pointRad <= kRad){
					
					angle_offset = acos((r-vw/2)/pointRad);
					
				}
				else{
					angle_offset = asin(vl/pointRad);
				}

				//subtract this offset
				float adj_obst_angle = raw_obst_angle - angle_offset;
				
				//record shortest path and its parameters
				if(adj_obst_angle < min_obst_angle){
					min_obst_angle = adj_obst_angle;
					minPointRad = pointRad;
					min_angle_offset = angle_offset;
					//const Vector2f relPoint(points.at(i).x()*cos(actualAngle)-points.at(i).y()*sin(actualAngle)+actualLocation.x(),points.at(i).y()*cos(actualAngle)+points.at(i).x()*sin(actualAngle)+actualLocation.y());
					isObst = true;
					
				}
				
				//graph relevant points 
				//const Vector2f relPoint(points.at(i).x()*cos(actualAngle)-points.at(i).y()*sin(actualAngle)+actualLocation.x(),points.at(i).y()*cos(actualAngle)+points.at(i).x()*sin(actualAngle)+actualLocation.y());
				//DrawPoint(relPoint, 0x6A0DAD, local_viz_msg_);
			
			}
			
			//make sure path doesn't continue in obstructed areas 
			if(pointRad <innerRad){
				
				//find the angle between baselink and obstacle about center of rotation
				raw_obst_angle = 2* atan2(abs(points.at(i).y()),points.at(i).x());
				angle_offset = kAngle;
				float adj_obst_angle = raw_obst_angle - angle_offset;
				
				//record shortest path and its parameters
				if(adj_obst_angle < min_obst_angle){
					min_obst_angle = adj_obst_angle;
					minPointRad = kRad;
					min_angle_offset = angle_offset;
					const Vector2f relPoint(points.at(i).x()*cos(actualAngle)-points.at(i).y()*sin(actualAngle)+actualLocation.x(),points.at(i).y()*cos(actualAngle)+points.at(i).x()*sin(actualAngle)+actualLocation.y());
					isObst = false;
				}

			}
			
		}
		
		//find closest point on path to goal
		/*comment out for removing navigation closest point
		if(min_obst_angle > goalAngle){
				min_obst_angle = goalAngle;
				isObst = false;
		}*/
		
		//if the obstacle angle is negative for some reason, set to zero
		if (min_obst_angle < 0){
			min_obst_angle = 0;
		}
		
		//different things of interest to plot
		//plotLinkArc(1/min_scan_rad*rSign,r, min_scan, 0x77fc03);
		//plotLinkArc(1/minPointRad*rSign,r, min_obst_angle, 0x77fc03);
		//plotLinkArc(1/outerRad, r, tAngle, 0xFFC0CB);
		//plotLinkArc(1/(kRad), r, kAngle, 0x964B00);
		//plotLinkArc(1/(innerRad), r, kAngle, 0x964B00);
		if(plot){
			plotObstArcOrigin(1/minPointRad*rSign, r, min_obst_angle, min_angle_offset,color);
		}
		
		
		//set minDist to the shortest path
		float *pointVects;
		pointVects = getScanVector(min_obst_angle,r);
		const Vector2f relEndPoint(*(pointVects),*(pointVects+1));
		const Vector2f bl(0,0);
		//DrawLine(bl,relEndPoint, 0xFFC8CC,local_viz_msg_);
		endGoalDist= dist(relEndPoint, turnedRefGoal);
		//float currGoalDist = dist(bl,turnedRefGoal);
		//if end path takes you further from goal, don't move
		/*also comment our for removing navigation closest point
		if(currGoalDist<endGoalDist){
			min_obst_angle = 0;
		}*/
		
		minDist= min_obst_angle * r;

		
		//CLEARANCE CALCULATION (CURVED) --------------------------------------------------------------
		
		//define clearance radii
		float innerClearRad = r - vw/2-maxC;
		float outerClearRad = sqrt(pow(r+vw/2+maxC,2)+pow(vl+maxC,2));
		
		//vars
		minClearance = maxC;
		float minClearRad = maxC;
		float min_clear_angle = min_obst_angle + tAngle;

		//float threshold = 5; //how many meters ahead we care about
		//float thresholdAngle = threshold / r;
		//if(thresholdAngle > min_obst_angle){
		//	thresholdAngle = min_obst_angle;
		//}
		
		if(isObst){
			minClearance = 0;
		} 
		else {
				
			//now find the minimum clearance along this path
			for(int i =0; i<cloudSize; i++){

				//calculate radius of point from center of rotation
				pointRad = dist(points.at(i),centerRef);


				if((pointRad > outerRad)&&(pointRad <=outerClearRad)){
					float phi = getScanAngle(points.at(i),r);
					if((min_obst_angle+tAngle>=phi)&&(pointRad - outerRad < minClearance)){

						minClearance = pointRad - outerRad;
						minClearRad = pointRad;
						min_clear_angle = phi;
						
					}
					//const Vector2f relPoint(points.at(i).x()*cos(actualAngle)-points.at(i).y()*sin(actualAngle)+actualLocation.x(),points.at(i).y()*cos(actualAngle)+points.at(i).x()*sin(actualAngle)+actualLocation.y());
					//DrawPoint(relPoint, 0x80020, local_viz_msg_);
				} 
				else if((pointRad < innerRad)&&(pointRad >= innerClearRad)){
					float phi = getScanAngle(points.at(i),r);
					if((min_obst_angle+kAngle>=phi)&&(innerRad - pointRad < minClearance)){
						
						minClearance = innerRad - pointRad;
						minClearRad = pointRad;
						min_clear_angle = phi;
					}
					//const Vector2f relPoint(points.at(i).x()*cos(actualAngle)-points.at(i).y()*sin(actualAngle)+actualLocation.x(),points.at(i).y()*cos(actualAngle)+points.at(i).x()*sin(actualAngle)+actualLocation.y());
					//DrawPoint(relPoint, 0x80020, local_viz_msg_);
				}
				//ROS_INFO("min_obst_angle: %f, minPointRad: %f, min_angle_offset: %f", min_obst_angle, minPointRad, min_angle_offset);
			}
		}

	if(plot &&(1==-1)){
		plotLinkArc(1/minClearRad*rSign,r, min_clear_angle, 0x77fc03);
	}
		
	
	}
	//if traveling in a straight line
	else{

		float min_dist = 10.0;
		float min_y = 0;
		float xCord = 0;
		float yCord = 0;
		
		for(int i =0; i<cloudSize; i++){
			
			xCord = points.at(i).x();
			yCord = points.at(i).y();
		
			if(abs(yCord) <= vw/2){
				//const Vector2f relPoint(xCord*cos(actualAngle)-yCord*sin(actualAngle)+actualLocation.x(),yCord*cos(actualAngle)+xCord*sin(actualAngle)+actualLocation.y());
				//DrawPoint(relPoint, 0x6A0DAD, local_viz_msg_);
				if(xCord<min_dist){
					min_dist = xCord;
					min_y = yCord;
				}
				
			}
			
		}
		if(plot){
			const Vector2f minPoint(min_dist*cos(actualAngle)-min_y*sin(actualAngle)+actualLocation.x(),min_y*cos(actualAngle)+min_dist*sin(actualAngle)+actualLocation.y());
			const Vector2f relEdge(actualLocation.x() + vl*cos(actualAngle)-min_y*sin(actualAngle),actualLocation.y() + vl* sin(actualAngle)+min_y*cos(actualAngle));
			DrawLine(minPoint,relEdge, 0x77fc03,local_viz_msg_);
		}
		minDist= min_dist-vl;
	
		//CLEARANCE CACULATION (STRAIGHT)
		float minClearDist= min_dist+vl;
		float minClearRad = 0;
		for(int i =0; i<cloudSize; i++){
			
			xCord = points.at(i).x();
			yCord = points.at(i).y();
			
			if(xCord <=0){
				continue;
			}
			if((abs(yCord) > vw/2 )&& (abs(yCord) <=vw/2 +maxC)){
				if(xCord < minClearDist){
					minClearDist = xCord;
					minClearRad = yCord;
				}
				//const Vector2f relPoint(xCord*cos(actualAngle)-yCord*sin(actualAngle)+actualLocation.x(),yCord*cos(actualAngle)+xCord*sin(actualAngle)+actualLocation.y());
				//DrawPoint(relPoint, 0x6A0DAD, local_viz_msg_);
			}
		}
		if(minClearDist > 5){
			minClearDist =0;
			minClearRad = maxC;
		}
		
		
		//const Vector2f blEdge(-minClearRad*sin(actualAngle)+actualLocation.x(),minClearRad*cos(actualAngle)+actualLocation.y());
		//const Vector2f clearObst(minClearDist*cos(actualAngle)-minClearRad*sin(actualAngle)+actualLocation.x(),minClearRad*cos(actualAngle)+minClearDist*sin(actualAngle)+actualLocation.y());
		//DrawLine(blEdge,clearObst, 0x77fc03,local_viz_msg_);
		minClearance = abs(minClearRad);
			
	}
	
	obArr[0] = minDist;
	obArr[1] = minClearance;
	obArr[2] = endGoalDist;

	
	return obArr;
}

//SCORING FUNCTION--------------------------------------------------------
float * score(){
	
	static float scoreArr[2];
	const int size = 60;
	static float curves[size];
	static float score[size];
	static float clearances[size];
	static float FPLs[size];
	static float goalDists[size];
	//static vector<Vector2f> FPL_cloud;
	//FPL_cloud.clear();
	
	float A = 0;
	float B = 4;
	float C = -5;
	
	int count = 0;
	float j = 1.05;
	float plotCurvature = 0;
	
	while(j < 1.9){
		if(count%2 == 0){
			plotCurvature = j - 1;
			j = j * 1.025;
		} else{
			plotCurvature = -(j-1);
		}
		if(plotCurvature == 0){
			continue;
		}
		//find longest free path for given curvature
		float *odVars;
		odVars = ShortestPathOD(plotCurvature,false,0);
		
		//find end path position, path length, goal distance from end path position
		//const Vector2f point(*(odVars+2),*(odVars+3));
		//FPL_cloud.push_back(point);
		float FPL = *(odVars);
		float clearance = *(odVars+1);
		float goalDist = *(odVars+2);
		
		clearances[count] = clearance ;
		FPLs[count] = FPL;
		goalDists[count] = goalDist;
		score[count] = A*FPL+B*clearance+C*goalDist;
		curves[count] = plotCurvature;
		count++;
	}
	
	int maxIndex = getMaxIndex(score,count);
	
	//ROS_INFO("goalLocation: %f x %f y",goalLocation.x(),goalLocation.y());
	//ROS_INFO("endPoint Loc: %f x %f y",FPL_cloud.at(maxIndex).x(),FPL_cloud.at(maxIndex).y());
	//ROS_INFO("endPoint Loc: %f x %f y",actualLocation.x(),actualLocation.y());

	ROS_INFO("score: %f", score[maxIndex]);
	ROS_INFO("A*FPL: %f B*clear: %f, C*gd: %f", FPLs[maxIndex]*A, clearances[maxIndex]*B, goalDists[maxIndex]*C);
	
	//for graphing
	//DrawPoint(FPL_cloud.at(maxIndex),0x000000, local_viz_msg_);
	float *odVars;
	odVars = ShortestPathOD(curves[maxIndex],true,0xFFC0CB);
	scoreArr[0] = *(odVars);
	scoreArr[1] = curves[maxIndex];
	return scoreArr;
}
	
float * refine( float roughCurve){
	static float scoreArr1[2];
	const int size = 100;
	static float curves1[size];
	static float score1[size];
	static float clearances1[size];
	static float FPLs1[size];
	static float goalDists1[size];
	//static vector<Vector2f> FPL_cloud;
	//FPL_cloud.clear();
	float A = 0;
	float B = 1;
	float C = 0;
	//float D = 0;
	
	int count = 0;
	float j = 1.005;
	float plotCurvature = 0;
	float curveConvert = 0;
	float plotDist = 5;
	while(j < 1.1){
		if(count%2 == 0){
			curveConvert = j - 1;
			j = j * 1.0025;
			plotCurvature = roughCurve + curveConvert;
		} else{
			curveConvert = -(j-1);
			plotCurvature = roughCurve + curveConvert;

		}
		if(plotCurvature == 0){
			continue;
		}
		
			plotLinkArc(plotCurvature,abs(1/plotCurvature), abs(plotDist*plotCurvature), 0x77fc03);
		
		//find longest free path for given curvature
		float *odVars1;
		odVars1 = ShortestPathOD(plotCurvature,false, 0);
		
		//find end path position, path length, goal distance from end path position
		//const Vector2f point(*(odVars1+2),*(odVars1+3));
		//FPL_cloud.push_back(point);
		float FPL = *(odVars1);
		float clearance = *(odVars1+1);
		float goalDist = *(odVars1+2);
		
		clearances1[count] = clearance ;
		FPLs1[count] = FPL;
		goalDists1[count] = goalDist;
		//score1[count] = A*FPL+B*clearance*FPL+C*clearance + D*goalDist;
		score1[count] = A*FPL+B*clearance+C*goalDist;

		curves1[count] = plotCurvature;
		
		count++;
	}
	int maxIndex = getMaxIndex(score1,size);
	
	//ROS_INFO("goalLocation: %f x %f y",goalLocation.x(),goalLocation.y());
	//ROS_INFO("endPoint Loc: %f x %f y",FPL_cloud.at(maxIndex).x(),FPL_cloud.at(maxIndex).y());
	//ROS_INFO("endPoint Loc: %f x %f y",actualLocation.x(),actualLocation.y());
	ROS_INFO("clearance = %f",clearances1[maxIndex]);
	if(-1==1){
	ROS_INFO("refined score: %f", score1[maxIndex]);
	ROS_INFO("refined: A*FPL: %f B*clear: %f, C*gd: %f", A*FPLs1[maxIndex], clearances1[maxIndex]*B, goalDists1[maxIndex]*C);
	}
	//for graphing
	//DrawPoint(FPL_cloud.at(maxIndex),0x000000, local_viz_msg_);
	float *odVars1;
	odVars1 = ShortestPathOD(curves1[maxIndex],true,0x00AB22);
	scoreArr1[0] = *(odVars1);
	scoreArr1[1] = curves1[maxIndex];
	return scoreArr1;
}

//MAIN -------------------------------------------------------------------------------------------------
void Navigation::Run() {
		
	//gather speed information
	//float velMag = mag(robot_vel_);

	//initialize 
    if (runCount < 8){
		startOdomY = robot_loc_.y();
		startOdomX = robot_loc_.x();
		runCount++;
		//const Vector2f startLocation(actualLocation.x(),actualLocation.y());
		ROS_INFO("initializing starting odometry");
		//ClearVisualizationMsg(local_viz_msg_);
    } 
	drive_msg_.curvature = 0;
	drive_msg_.velocity = 0;
	
	
     //find longest free path for given curvature
	 /*
	float *odVars1;
	odVars1 = ShortestPathOD(drive_msg_.curvature,true, 0);
	TOC(*(odVars1),velMag);
	*/
	
	//if((goalLocation.x() != 0)&&(goalLocation.y() != 0)){
		
		/*
		float *scoreVars;
		float *scoreVars2;
		scoreVars = score();
		scoreVars2 = refine(*(scoreVars+1));
		drive_msg_.curvature = *(scoreVars2+1);
		TOC(*(scoreVars2),velMag);*/
		//green is refined path
		
		
		/*
		drive_msg_.curvature = .2;
		float *odDrive;
		odDrive = ShortestPathOD(drive_msg_.curvature,true,0x00AB22);
		float FPL = *(odDrive);
		float clearance = *(odDrive + 1);
		TOC(*(odDrive),velMag);
		ROS_INFO("FPL: %f, clearance: %f",FPL, clearance);
		*/
	//}
	


	//visualization of possible paths
	/*
	float i = 1.05;
	float plotDist = 5;
	while(i < 3){
		float plotCurvature1 = i/2 - 1.05/2;
		float plotCurvature2 = -plotCurvature1;
		i = i * 1.05;
		plotLinkArc(plotCurvature1,1/plotCurvature1, abs(plotDist*plotCurvature1), 0x77fc03);
		plotLinkArc(plotCurvature2,1/plotCurvature2, abs(plotDist*plotCurvature2), 0x77fc03);
	}
	const Vector2f bl(actualLocation.x(),actualLocation.y());
	const Vector2f end(actualLocation.x() + plotDist*cos(actualAngle),actualLocation.y() + plotDist*sin(actualAngle));
	DrawLine(bl,end, 0x77fc03,local_viz_msg_);
	 */
    
	

	//plot the bot
	
	//plotCar(0x0000ff,0xff0000);
	//DrawCross(nav_goal_loc_, .1, 0xFF00FF,local_viz_msg_);
	viz_pub_.publish(local_viz_msg_);
	drive_pub_.publish(drive_msg_);
	//ClearVisualizationMsg(local_viz_msg_);

}  // namespace navigation

}
