#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Header.h>
#include <obstacle_detector/Obstacles.h>
#include <obstacle_converter/Zonotopes.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <math.h>

std::vector<geometry_msgs::Point> first_points;
std::vector<geometry_msgs::Point> last_points;
std::vector<geometry_msgs::Point> centers;
std::vector<double> radius;
std_msgs::Header header_all;
jsk_recognition_msgs::PolygonArray pol;
obstacle_converter::Zonotopes zono;

void obstaclesCallback(const obstacle_detector::Obstacles& obs) {

    first_points.clear();
    last_points.clear();    
    radius.clear();
    centers.clear();
    header_all = obs.header;
    pol.polygons.clear();
    zono.zonotopes.clear(); 
    int seg_size = obs.segments.size();
    //std::cout<<seg_size<<std::endl;
    for(int i = 0; i<seg_size; i++) {
      first_points.push_back(obs.segments[i].first_point);
      last_points.push_back(obs.segments[i].last_point);
    }
    
    
    
    int cir_size = obs.circles.size();
    for(int i = 0; i<cir_size; i++) {
      centers.push_back(obs.circles[i].center);
      radius.push_back(obs.circles[i].radius);
    }
} // obstaclesCallback

int main(int argc, char** argv) {

    ros::init(argc, argv, "obstacle_converter");
    ros::NodeHandle nh_pub;
    ros::Publisher pub2 = nh_pub.advertise<obstacle_converter::Zonotopes>("/zonotope", 1000);
    ros::Publisher pub = nh_pub.advertise<jsk_recognition_msgs::PolygonArray>("/zonotope_visualization", 1000);
    ros::Subscriber sub = nh_pub.subscribe("/obstacles", 1000, obstaclesCallback);
    ros::Rate loop_rate(50);
    while(ros::ok()) {
      if (!(first_points.empty())) {
        //std::cout<<first_points.size()<<std::endl;
        for(int i = 0; i<first_points.size(); i++) {
	  geometry_msgs::Point c;
          geometry_msgs::Point generator_1;
          geometry_msgs::Point generator_2;
          obstacle_converter::Zonotope z;
          c.x = (first_points[i].x + last_points[i].x)/2;
          c.y = (first_points[i].y + first_points[i].y)/2;
          generator_1.x = 0.5*(first_points[i].x - last_points[i].x);
          generator_1.y = 0.5*(first_points[i].y - last_points[i].y);
          float len_1 = sqrt(pow(generator_1.x,2.0) + pow(generator_1.y,2.0));
          generator_2.x = generator_1.y*0.08;
          generator_2.y = -generator_1.x*0.08;
          z.points.push_back(c);
          z.points.push_back(generator_1);
          z.points.push_back(generator_2);
          zono.zonotopes.push_back(z);
        }
      }
      std::cout<<centers.empty()<<std::endl;
      if (!(centers.empty())) {
        for(int j = 0; j<centers.size(); j++) {
          geometry_msgs::PolygonStamped g;
          geometry_msgs::Point c;
          geometry_msgs::Point g1;
          geometry_msgs::Point g2;
          geometry_msgs::Point g3;
          geometry_msgs::Point32 p1;
          geometry_msgs::Point32 p2;
          geometry_msgs::Point32 p3;
          geometry_msgs::Point32 p4;
          geometry_msgs::Point32 p5;
          geometry_msgs::Point32 p6;
          obstacle_converter::Zonotope z;
          double r;
          r = radius[j]; 
          c = centers[j];
          p1.x = c.x - r;
          p1.y = c.y + r/sqrt(3);
          p2.x = c.x;
          p2.y = c.y + (2*r)/sqrt(3);
          p3.x = c.x + r;
          p3.y = c.y + r/sqrt(3);
          p4.x = c.x + r;
          p4.y = c.y - r/sqrt(3);
          p5.x = c.x;
          p5.y = c.y - (2*r)/sqrt(3);
          p6.x = c.x - r;
          p6.y = c.y - r/sqrt(3);
          g.polygon.points.push_back(p1);
          g.polygon.points.push_back(p2); 
          g.polygon.points.push_back(p3);
          g.polygon.points.push_back(p4);
          g.polygon.points.push_back(p5);
          g.polygon.points.push_back(p6);
          g.header = header_all;
          pol.polygons.push_back(g);
          std::cout<<pol.polygons.size()<<std::endl;
          g1.x = r;
          g1.y = r/sqrt(3);
          g2.x = r;
          g2.y = -r/sqrt(3);
          g3.x = 0;
          g3.y = (2*r)/sqrt(3);
          z.points.push_back(c);
          z.points.push_back(g1);
          z.points.push_back(g2);
          z.points.push_back(g3);
          zono.zonotopes.push_back(z);
        }
      }
      zono.header = header_all;
      pub2.publish(zono);
      pol.header = header_all;
      pub.publish(pol);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}





