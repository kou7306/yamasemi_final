#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <map>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>

// state
enum State {STOP, RUN, FINISH, AVOIDANCE} state;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // scan data
    std::vector<float> scan_data;
    scan_data = msg->ranges;
    // scan data size
    int scan_data_size = scan_data.size();
    // scan data angle
    float scan_data_angle = msg->angle_min;
    // scan data angle increment
    float scan_data_angle_increment = msg->angle_increment;
    // scan data range
    float scan_data_range = msg->range_min;
    // scan data range max
    float scan_data_range_max = msg->range_max;
    // scan data range size
    int scan_data_range_size = (scan_data_range_max - scan_data_range) / msg->range_increment;
    
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "state");
	ros::NodeHandle nh;
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 100);
    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odomCallback);
    ros::Subscriber scan_sub = nh.subscribe("scan", 100, scanCallback);
    state = STOP;


	while (ros::ok())
	{
        switch (state)
        {
            case STOP:
                
                break;

            case RUN:
                
                break;

            case AVOIDANCE:
                    
                break;
            
            case FINISH:
                    
                break;
            
            default:
                state = STOP;
                break;
        }
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}