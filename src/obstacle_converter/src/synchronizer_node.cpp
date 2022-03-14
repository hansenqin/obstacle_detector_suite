#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <obstacle_detector/Obstacles.h>
#include <sensor_msgs/LaserScan.h>

using namespace message_filters;
ros::Publisher pub_scan;
ros::Publisher pub_obstacle;


void callback(const sensor_msgs::LaserScanConstPtr& scan_data, const obstacle_detector::ObstaclesConstPtr& detector_data){
	pub_scan.publish(scan_data);
	pub_obstacle.publish(detector_data);	
}
 
int main(int argc, char** argv){
	ros::init(argc, argv, "synchronizer_node");
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1);
	message_filters::Subscriber<obstacle_detector::Obstacles> obstacles_sub(nh, "/obstacles", 1);
	pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_aligned", 1000);
	pub_obstacle = nh.advertise<obstacle_detector::Obstacles>("/obstacles_aligned", 1000);
	typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, obstacle_detector::Obstacles> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), scan_sub, obstacles_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	ros::spin();
	return 0;
}
