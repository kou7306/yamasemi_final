#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <map>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>


// state
enum State {STOP, RUN1, Run2, Run3, Run4, Run5, FINISH, AVOIDANCE} state;

sensor_msgs::LaserScan scan_data; 
bool is_obstacle = false;
double current_x = 0.0;
double current_y = 0.0;
double move_distance = 0.0;
state current_state = STOP;

void obstacleDetect(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // scan data
    
    scan.range = msg->ranges;
    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.angle_increment = msg->angle_increment;
    scan.time_increment = msg->time_increment;
    scan.scan_time = msg->scan_time;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;

    double obstacle_distance = 1.0; // 障害物と判定する距離


    // 正面のレーザーを取得し、障害物があるか判定
    for (int i = 0; i < scan.range.size(); i++)
    {
        angle=std::fabs(scan.angle_min + scan.angle_increment * i);
        if(3.0 > 1.0 * sin(angle))
        {
            if(scan.range[i] < obstacle_distance)
            {
                is_obstacle = true;
                break;
            }
        }

    }
}


void moveWaypoint()
{

    // 速度指令を送る
    geometry_msgs::Twist twist;
    twist.linear.x = 0.1;
    twist.angular.z = 0.0;
    twist_pub.publish(twist);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "state");
	ros::NodeHandle nh;
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 100); //速度命令を送るPublisher
    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odomCallback); //オドメトリを受け取るSubscriber 
    ros::Subscriber scan_sub = nh.subscribe("scan", 100, obstacleDetect); //スキャンデータを受け取るSubscriber
    state = STOP;

    geometry_msgs::Twist move_cmd;
    ros::Rate rate(10);



    //scanデータの分割



	while (ros::ok())
	{
        switch (state)
        {
            case STOP:
                if(is_obstacle){
                    state = AVOIDANCE;
                }
                else{
                    state = RUN;
                }            
                break;

            case RUN1:
                // 初期位置を保存
                double start_x = current_x;
                double start_y = current_y;
                double target_distance = 4.5; // 目標距離
                move_distance = 0.0; // 移動距離
                move_cmd.linear.x = 0.5;
                move_cmd.angular.z = 0.0;
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        state = AVOIDANCE;
                        break;
                    } else {
                        twist_pub.publish(move_cmd);
                        rate.sleep();
                        move_distance = std::sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));
                    }
                }

                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = 0.0;
                twist_pub.publish(move_cmd);
                ros::Duration(1.0).sleep();

                state = RUN2;
                    
                
                break;
            

            case RUN2:
                // 初期位置を保存
                double start_x = current_x;
                double start_y = current_y;
                double target_distance = 1.0; // 目標距離
                move_distance = 0.0; // 移動距離
                move_cmd.linear.y = 0.5;
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        state = AVOIDANCE;
                        break;
                    } else {
                        twist_pub.publish(move_cmd);
                        rate.sleep();
                        move_distance = std::sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));
                    }
                }

                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = 0.0;
                
                twist_pub.publish(move_cmd);
                ros::Duration(1.0).sleep();

                state = RUN3;
                    
                
                break;


    

            case RUN3:
                // 初期位置を保存
                double start_x = current_x;
                double start_y = current_y;
                double target_distance = 2.5; // 目標距離
                move_distance = 0.0; // 移動距離
                //現在のロボットの姿勢を基準にしている
                move_cmd.linear.x = -0.4;
                move_cmd.linear.y = 0.3;
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        current_state = Run3
                        state = AVOIDANCE;
                        break;
                    } else {
                        twist_pub.publish(move_cmd);
                        rate.sleep();
                        move_distance = std::sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));
                    }
                }

                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = 0.0;
                twist_pub.publish(move_cmd);
                ros::Duration(1.0).sleep();

                state = RUN4;
                    
                
                break;

            case RUN4:
                // 初期位置を保存
                double start_x = current_x;
                double start_y = current_y;
                double target_distance =  4 * std::sqrt(2); // 目標距離
                move_distance = 0.0; // 移動距離
                move_cmd.linear.x = 0.3;
                move_cmd.linear.y = 0.3;
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        state = AVOIDANCE;
                        break;
                    } else {
                        twist_pub.publish(move_cmd);
                        rate.sleep();
                        move_distance = std::sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));
                    }
                }

                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = 0.0;
                twist_pub.publish(move_cmd);
                ros::Duration(1.0).sleep();

                state = RUN5;
                    
                
                break;

            case RUN5:
                // 初期位置を保存
                double start_x = current_x;
                double start_y = current_y;
                double target_distance = 4.5; // 目標距離
                move_distance = 0.0; // 移動距離
                move_cmd.linear.x = 0.5;
                move_cmd.angular.z = 0.0;
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        state = AVOIDANCE;
                        break;
                    } else {
                        twist_pub.publish(move_cmd);
                        rate.sleep();
                        move_distance = std::sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));
                    }
                }

                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = 0.0;
                twist_pub.publish(move_cmd);
                ros::Duration(1.0).sleep();

                state = STOP;
                    
                
                break;

            case AVOIDANCE:
                double radius = 1.0;  // 円の半径
                double angular_speed = 0.2;  // 角速度（回転速度）
                double arc_length = M_PI * radius;  // 円周の長さ

                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = angular_speed;

                move_distance = 0.0;

                ros::Rate rate(10);  // ループ周波数

                double start_time = ros::Time::now().toSec();

                while (move_distance < arc_length) {
                    double current_time = ros::Time::now().toSec();
                    double elapsed_time = current_time - start_time;

                    // 円周上を移動
                    double angle = fmod(angular_speed * elapsed_time, 2.0 * M_PI);
                    move_cmd.linear.x = radius * cos(angle);
                    move_cmd.linear.y = radius * sin(angle);
                    move_distance = angle * radius;

                    twist_pub.publish(move_cmd);

                    rate.sleep();
                }

                state = current_state;         

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