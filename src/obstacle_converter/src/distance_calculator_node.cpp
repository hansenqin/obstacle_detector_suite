#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <obstacle_detector/Obstacles.h>
#include <math.h>

std::vector<geometry_msgs::Point> first_points;
std::vector<geometry_msgs::Point> last_points;
std::vector<double> detector_distance;
std_msgs::Header header_all;
double mocap_distance;

double shortestDistance(const geometry_msgs::Point first_point, const geometry_msgs::Point last_point) {
    double A = -(first_point.x);
    double B = -(first_point.y);
    double C = (last_point.x)-(first_point.x);
    double D = (last_point.y)-(first_point.y);
    
    double dot = (A*C) + (B*D);
    double len_sq = (C*C) + (D*D);
    int param = -1;
    if (len_sq != 0) {
    	param = dot / len_sq;
    }
    double xx, yy;
    if(param < 0) {
    	xx = first_point.x;
        yy = first_point.y;
    }else if(param > 1) {
	xx = last_point.x;
        yy = last_point.y;
    }else {
        xx = first_point.x + param * C;
        yy = first_point.y + param * D;
    }
    double dist = sqrt((xx*xx)+(yy*yy));
    return dist;
}

void obstaclesCallback(const obstacle_detector::Obstacles& obs) {
    first_points.clear();
    last_points.clear();    
    header_all = obs.header;
    detector_distance.clear();
    int seg_size = obs.segments.size();
    for(int i = 0; i<seg_size; i++) {
      if((obs.segments[i].first_point.y > 0)&&(obs.segments[i].last_point.y > 0)) {
        if((obs.segments[i].first_point.y < 1)&&(obs.segments[i].last_point.y < 1)) {
	  if((obs.segments[i].first_point.x < 1)&&(obs.segments[i].first_point.x > -1)) {
	    if((obs.segments[i].last_point.x < 1)&&(obs.segments[i].last_point.x > -1)) {
      	      first_points.push_back(obs.segments[i].first_point);
      	      last_points.push_back(obs.segments[i].last_point);
            }
          }
	}
      }
    }
} // obstaclesCallback

/*
void mocapCallback(const geometry_msgs::PoseStamped& obj) {
    double x = 2000 - (obj.pose.position.x+170);
    mocap_distance = x;
}
*/

void mocapCallback(const geometry_msgs::PoseStamped& obj) {
    if((obj.pose.position.x+170)<1000) {
	mocap_distance = sqrt(pow(obj.pose.position.x+170-1000,2)+pow(obj.pose.position.z+340,2));      }else if((obj.pose.position.x+170)>1320) {
	mocap_distance = sqrt(pow(obj.pose.position.x+170-1320,2)+pow(obj.pose.position.z+340,2));
    }else {
        mocap_distance = obj.pose.position.z+340;
    }
}
int main(int argc, char** argv) {

    ros::init(argc, argv, "distance_calculator");
    ros::NodeHandle nh_sub;
    ros::Subscriber sub1 = nh_sub.subscribe("/obstacles", 1000, obstaclesCallback);
    ros::Subscriber sub2 = nh_sub.subscribe("/mocap", 1000, mocapCallback);
    ros::Publisher pub = nh_sub.advertise<geometry_msgs::Point>("/distance", 1000);
    ros::Rate loop_rate(50);
    std::ofstream myfile;
    myfile.open("/home/nvidia/mocap_detector_test_record/test2.txt");
    while(ros::ok()) {
      if (!(first_points.empty())) {
        for(int i = 0; i<first_points.size(); i++) {
      	  double dist;
          dist = shortestDistance(first_points[i],last_points[i]);
          detector_distance.push_back(dist);
        }
      
        double dist_min = *std::min_element(detector_distance.begin(),detector_distance.end());
        geometry_msgs::Point a;
        a.x = (dist_min*1000)-mocap_distance;
        myfile << a.x << std::endl;
        pub.publish(a);
      }
      ros::spinOnce();
      loop_rate.sleep(); 
    }
    myfile.close();
    return 0;
}





