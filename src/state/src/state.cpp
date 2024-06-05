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
double move_distance = 0.0;
bool is_obstacle = false;
double start_x;
double start_y;
double target_distance; 
double current_time;
double elapsed_time;
double angle;
double start_time;
double speed;
bool flag = true;
State state = STOP;
State current_state = RUN1;
// グローバル変数でtf::TransformListenerを保持
tf::TransformListener* tf_listener;
geometry_msgs::Twist move_cmd;
geometry_msgs::Twist cmd_vel_odom;
geometry_msgs::Point target_point;
geometry_msgs::Point current_pose;
geometry_msgs::Point target;
geometry_msgs::Point current_point;
double roll, pitch, yaw;

// グローバル変数
geometry_msgs::TransformStamped global_transform;
tf::StampedTransform transform_tf;

//現在位置の取得
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose = msg.pose.pose
    // クォータニオンからオイラー角（ロール、ピッチ、ヨー）を取得
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // ヨー角を表示
    ROS_INFO("Yaw: %f", yaw);
}



//　障害物検知
void obstacleDetect(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // scan data
    scan = *msg;

    double obstacle_distance = 1.0; // 障害物と判定する距離


    // 正面のレーザーを取得し、障害物があるか判定
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        double angle=std::fabs(scan.angle_min + scan.angle_increment * i);
        if(i > scan.ranges.size()/2 -15 && i < scan.ranges.size()/2 + 15)
        {
            // nan値を除外し、範囲内かつ障害物距離よりも小さい場合に障害物を検知
            if (!std::isnan(scan.ranges[i]) && scan.ranges[i] < obstacle_distance && scan.ranges[i] > 0.2) {
                ROS_INFO("distance: %f", scan.ranges[i]);
                ROS_INFO("index: %d", i);
                ROS_INFO("obstacle detected");
                is_obstacle = true;
                break;
            }
        }

    }
}



//通常移動（座標で指定）

void moveToTarget(const geometry_msgs::Point& target_point, ros::Publisher& twist_pub, const geometry_msgs::Pose& current_pose)
{
    // 現在のロボットの位置を取得
    geometry_msgs::Point current_point;
    current_point.x = current_pose.position.x;
    current_point.y = current_pose.position.y;
    current_point.z = current_pose.position.z; 

    // ターゲット座標をベースリンク座標系に変換（この関数があることを仮定）
    // geometry_msgs::Point target_base_link = transformPointToBaseLink(target_point);

    ROS_INFO("odom_point: (%f, %f)", current_point.x, current_point.y);

    double angle_to_target = std::atan2(target_point.y - current_point.y, target_point.x - current_point.x);
    if(target_point.x > current_point.x && target_point.y > current_point.y){
        angle_to_target = M_PI / 2 - angle_to_target + yaw;
    }else if(target_point.x > current_point.x && target_point.y < current_point.y){
        angle_to_target = -(M_PI / 2 - angle_to_target + yaw);
    }else if(target_point.x < current_point.x && target_point.y < current_point.y){
        angle_to_target = -(M_PI / 2 + angle_to_target + yaw);
    }else if(target_point.x < current_point.x && target_point.y > current_point.y){
        angle_to_target = M_PI / 2 + angle_to_target + yaw;
    }
    double angler_speed = 0.5;

    double time_to_rotate = std::abs(angle_to_target) / angler_speed; 

    ros::Time start_time = ros::Time::now();
    

    while(ros::ok() && (ros::Time::now() - start_time).toSec() < time_to_rotate)
    {
        ros::spinOnce();  
        move_cmd.angular.z = angler_speed;
        move_cmd.linear.x = 0.0;
        move_cmd.linear.y = 0.0;
        twist_pub.publish(move_cmd);
    }

    double distance_to_target = std::sqrt(std::pow(target_point.x - current_point.x, 2) + 
                                          std::pow(target_point.y - current_point.y, 2));

    double speed = 0.3;

    double time_to_move = distance_to_target / speed;

    start_time = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - start_time).toSec() < time_to_move)
    {
        ros::spinOnce();
        move_cmd.angular.z = 0.0;
        move_cmd.linear.x = speed;
        move_cmd.linear.y = 0.0;
        twist_pub.publish(move_cmd);
    }
    
    ros::Duration(1.0).sleep();
}


// 障害物回避
void moveAvoidance(ros::Publisher& twist_pub,const double distance,const bool form)
{
    
    // 目標角度を計算
    if(form == true){
        double target_angle = M_PI /2;
    }else{
        double target_angle = -M_PI /2;
    }



    double angular_speed = 0.5;  // 回転速度 0.5rad/s
    // 角速度から回転にかかる時間を計算
    double time_to_rotation = std::abs(target_angle) / angular_speed;

    ros::Time start_time = ros::Time::now();

    while(ros::ok() && (ros::Time::now() - start_time).toSec() < time_to_rotate){
        ros::spinOnce();
        // 回転を開始する
        move_cmd.linear.x = 0.0;
        move_cmd.linear.y = 0.0;
        move_cmd.angular.z = angular_speed;
        twist_pub.publish(move_cmd);
    }

    double speed = 0.2;
    double time_to_move = distance / speed;
    ros::Time start_time = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - start_time).toSec() < time_to_move){
        ros::spinOnce();
        // 移動速度を設定
        move_cmd.angular.z = 0.0;  // 回転速度は0に戻す
        move_cmd.linear.x = speed;
        move_cmd.lineer.y = 0.0;

        // メッセージを配信して移動を行う
        twist_pub.publish(move_cmd);
    }


    // 移動を停止
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    twist_pub.publish(stop_cmd);

    ros::Duration(1.0).sleep();
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "state");
	ros::NodeHandle nh;
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100); //速度命令を送るPublisher
    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback); //オドメトリを受け取るSubscriber 
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, obstacleDetect); //スキャンデータを受け取るSubscriber
    ros::Subscriber tf_sub = nh.subscribe("tf", 10, tfCallback); //tfデータを受け取るSubscriber
    tf_listener = new tf::TransformListener();

    state = STOP;

    ros::Rate rate(10);
    ros::spinOnce();


	while (ros::ok())
	{
        ros::spinOnce();
        switch (state)
        {
            case STOP:
                    move_cmd.linear.x = 0.0;
                    move_cmd.linear.y = 0.0;
                    move_cmd.angular.z = 0.0;
                    twist_pub.publish(move_cmd);
            
                if(is_obstacle){
                    state = AVOIDANCE;
                }
                else{
                    state = current_state;
                }            
                break;

            case RUN1:
                {
                    
                    ROS_INFO("RUN1");
                    
                    target_point.x = 4.5;  // ターゲットのx座標
                    target_point.y = 0.0;  // ターゲットのy座標
                    target_point.z = 0.0;  // ターゲットのz座標
                    
                    moveToTarget(target_point, twist_pub, *tf_listener);

                    if(is_obstacle){
                        state = AVOIDANCE;
                    }
                    else{
                        state = RUN2;
                    }
                }
                break;


            case RUN2:
                current_state = RUN2;
                ROS_INFO("RUN2");
                
                target_point.x = 4.5;  // ターゲットのx座標
                target_point.y = 1.0;  // ターゲットのy座標
                target_point.z = 0.0;  // ターゲットのz座標
                
                moveToTarget(target_point, twist_pub, *tf_listener);
                if(is_obstacle){
                    state = AVOIDANCE;
                }
                else{
                    state = RUN3;
                }
            
                break;


    

            case RUN3:
                current_state = RUN3;
                ROS_INFO("RUN3");
                
                target_point.x = 3.0;  // ターゲットのx座標
                target_point.y = -1.5;  // ターゲットのy座標
                target_point.z = 0.0;  // ターゲットのz座標
                
                moveToTarget(target_point, twist_pub, *tf_listener);

                if(is_obstacle){
                    state = AVOIDANCE;
                }
                else{
                    state = RUN4;
                }
                    
                
                break;

            case RUN4:
                current_state = RUN4;
                ROS_INFO("RUN4");
                
                target_point.x = 1.0;  // ターゲットのx座標
                target_point.y = 1.5;  // ターゲットのy座標
                target_point.z = 0.0;  // ターゲットのz座標
                
                moveToTarget(target_point, twist_pub, *tf_listener);
                if(is_obstacle){
                    state = AVOIDANCE;
                }
                else{
                    state = RUN5;
                }
                
                
                break;

            case RUN5:
                current_state = RUN5;
                ROS_INFO("RUN5");

                target_point.x = 0.0;  // ターゲットのx座標
                target_point.y = 1.5;  // ターゲットのy座標
                target_point.z = 0.0;  // ターゲットのz座標
                
                moveToTarget(target_point, twist_pub, *tf_listener);
                state = FINISH;
                    
                
                break;

            case AVOIDANCE:
                ROS_INFO("AVOIDANCE");
                ROS_INFO("avoidance1");
                double distance1 = 1.0;
                moveAvoidance(twist_pub, distance1, true);

                ROS_INFO("avoidance2");
                double distance2 = 2.0;
                moveAvoidance(twist_pub, distance2, false);

                ROS_INFO("avoidance3");
                double distance3 = 1.0;
                moveAvoidance(twist_pub, distance3, false);

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
		rate.sleep();
	}

	return 0;
}
