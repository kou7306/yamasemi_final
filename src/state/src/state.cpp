#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h> 
#include <map>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>


// state
enum State {STOP, RUN1, RUN2, RUN3, RUN4, RUN5, AVOIDANCE, FINISH};

sensor_msgs::LaserScan scan;
bool is_obstacle = false;
double current_x = 0.0;
double current_y = 0.0;
double move_distance = 0.0;
double start_x;
double start_y;
double target_distance; 
double current_time;
double elapsed_time;
double angle;
double start_time;
double radius;  // 円の半径
double angular_speed;  // 角速度（回転速度）
double arc_length;  // 円周の長さ
State state = STOP;
State current_state = STOP;
// グローバル変数でtf::TransformListenerを保持
tf::TransformListener* tf_listener;
geometry_msgs::Twist move_cmd;
geometry_msgs::Twist cmd_vel_odom;


void obstacleDetect(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // scan data
    
    scan = *msg;

    double obstacle_distance = 1.0; // 障害物と判定する距離


    // 正面のレーザーを取得し、障害物があるか判定
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        double angle=std::fabs(scan.angle_min + scan.angle_increment * i);
        if(3.0 > 1.0 * sin(angle))
        {
            if(scan.ranges[i] < obstacle_distance)
            {
                is_obstacle = true;
                break;
            }
        }

    }
}



// odom座標系からbase_link座標系への速度命令の変換関数
geometry_msgs::Twist transformTwistToBaseLink(const geometry_msgs::Twist& twist_odom)
{
    geometry_msgs::Twist twist_base_link;

    tf::StampedTransform transform;
    try
    {
        tf_listener->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return twist_base_link;
    }

    tf::Vector3 linear_base_link = transform * tf::Vector3(twist_odom.linear.x, twist_odom.linear.y, 0.0);
    twist_base_link.linear.x = linear_base_link.x();
    twist_base_link.linear.y = linear_base_link.y();
    twist_base_link.angular.z = twist_odom.angular.z;

    return twist_base_link;
}






int main(int argc, char **argv)
{
	ros::init(argc, argv, "state");
	ros::NodeHandle nh;
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 100); //速度命令を送るPublisher
    // ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odomCallback); //オドメトリを受け取るSubscriber 
    ros::Subscriber scan_sub = nh.subscribe("scan", 100, obstacleDetect); //スキャンデータを受け取るSubscriber
    state = STOP;

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
                    state = RUN1;
                }            
                break;

            case RUN1:
                {
                    // 初期位置を保存
                    start_x = current_x;
                    start_y = current_y;
                    target_distance = 4.5; // 目標距離
                    move_distance = 0.0; // 移動距離
                    cmd_vel_odom.linear.x = 0.5; // 例: 0.5m/s の直進速度
                    cmd_vel_odom.angular.z = 0.0; // 例: 回転速度なし
                    // ロボット座標に変換
                    move_cmd = transformTwistToBaseLink(cmd_vel_odom);
                    while (move_distance < target_distance) {
                        if (is_obstacle) {
                            current_state = RUN1;
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
                }
                break;


            case RUN2:
                // 初期位置を保存
                start_x = current_x;
                start_y = current_y;
                target_distance = 1.0; // 目標距離
                move_distance = 0.0; // 移動距離
                cmd_vel_odom.linear.y = 0.5;

                move_cmd = transformTwistToBaseLink(cmd_vel_odom);
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        current_state = RUN2;
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
                start_x = current_x;
                start_y = current_y;
                target_distance = 2.5; // 目標距離
                move_distance = 0.0; // 移動距離
                cmd_vel_odom.linear.x= -0.3;
                cmd_vel_odom.linear.y = -0.4;

                move_cmd = transformTwistToBaseLink(cmd_vel_odom);
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        current_state = RUN3;
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
                start_x = current_x;
                start_y = current_y;
                target_distance = 4 * std::sqrt(2); // 目標距離
                move_distance = 0.0; // 移動距離
                cmd_vel_odom.linear.x= -0.25;
                cmd_vel_odom.linear.y = 0.25;

                move_cmd = transformTwistToBaseLink(cmd_vel_odom);
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        current_state = RUN4;
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
                start_x = current_x;
                start_y = current_y;
                target_distance = 1.0; // 目標距離
                move_distance = 0.0; // 移動距離
                cmd_vel_odom.linear.x= -0.5;

                move_cmd = transformTwistToBaseLink(cmd_vel_odom);
                while(move_distance < target_distance) {
                    if(is_obstacle){
                        current_state = RUN5;
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


                state = FINISH;
                    
                
                break;

            case AVOIDANCE:
                radius = 1.0;  // 円の半径
                angular_speed = 0.2;  // 角速度（回転速度）
                arc_length = M_PI * radius;  // 円周の長さ

                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = angular_speed;

                move_distance = 0.0;

                start_time = ros::Time::now().toSec();

                while (move_distance < arc_length) {
                    current_time = ros::Time::now().toSec();
                    elapsed_time = current_time - start_time;

                    // 円周上を移動
                    angle = fmod(angular_speed * elapsed_time, 2.0 * M_PI);
                    move_cmd.linear.x = radius * cos(angle);
                    move_cmd.linear.y = radius * sin(angle);
                    move_distance = angle * radius;

                    twist_pub.publish(move_cmd);

                    rate.sleep();
                }

                state = current_state;         

                break;
            
            case FINISH:
                move_cmd.linear.x = 0.0;
                move_cmd.linear.y = 0.0;
                move_cmd.angular.z = 0.0;
                twist_pub.publish(move_cmd);
                    
                break;
            
            default:
                break;
        }
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}