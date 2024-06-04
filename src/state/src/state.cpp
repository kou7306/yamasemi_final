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
geometry_msgs::Point odom_point;
geometry_msgs::Point target;
geometry_msgs::Point current_point;

// グローバル変数
geometry_msgs::TransformStamped global_transform;
tf::StampedTransform transform_tf;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_point.x = msg->pose.pose.position.x;
    odom_point.y = msg->pose.pose.position.y;
}


// tfメッセージのコールバック関数
void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    // メッセージからtransformを抽出
    for (const auto& transform: msg->transforms)
    {
        if (transform.child_frame_id == "base_link" && transform.header.frame_id == "odom")
        {
            // odomからbase_linkへの変換情報を見つけたら、グローバル変数に格納
            global_transform = transform;
            break;
        }
    }
}


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



// odom座標系からbase_link座標系への変換関数
geometry_msgs::Point transformPointToBaseLink(const geometry_msgs::Point& point_odom)
{
    geometry_msgs::Point point_base_link;

    tf::StampedTransform transform;
    // try
    // {
    //     tf_listener->lookupTransform("odom", "base_link", ros::Time(0), transform);
    // }
    // catch (tf::TransformException &ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    //     return point_base_link;
    // }

    // ROS_INFO("point_odom: (%f, %f)", point_odom.x, point_odom.y);
    // ROS_INFO("global_transform: (%f, %f, %f)", global_transform.transform.translation.x, global_transform.transform.translation.y, global_transform.transform.translation.z);
    // if(global_transform.transform.translation.x == 0.0 && global_transform.transform.translation.y == 0.0 && global_transform.transform.translation.z == 0.0)
    // {
    //     return point_odom;
    // }

    // tf::Vector3 point_odom_tf(point_odom.x, point_odom.y, point_odom.z);
    // tf::transformStampedMsgToTF(global_transform, transform_tf);  // global_transformを使って変換

    // // ここで変換を行います
    // tf::Vector3 point_base_link_tf = transform_tf * point_odom_tf;

    point_base_link.x = point_odom.x-odom_point.x;
    point_base_link.y = point_odom.y-odom_point.y;


    return point_base_link;
}



void moveToTarget(const geometry_msgs::Point& target_point, ros::Publisher& twist_pub, tf::TransformListener& tf_listener)
{
    // 現在のロボットの位置を取得
    geometry_msgs::Point current_point;
    // ここで現在の位置を取得する処理が必要
    current_point.x = 0.0;
    current_point.y = 0.0;
    current_point.z = 0.0; 

    // ターゲット座標をベースリンク座標系に変換
    geometry_msgs::Point target_base_link = transformPointToBaseLink(target_point);

    ROS_INFO("target_base_link: (%f, %f)", target_base_link.x, target_base_link.y);
    ROS_INFO("odom_point: (%f, %f)", odom_point.x, odom_point.y);

    start_x = odom_point.x;
    start_y = odom_point.y;

    // 移動速度を設定
    speed= 0.3;  // 移動速度 0.5m/s

    // ロボットを目標座標に向かって移動させる
    while (ros::ok() && std::sqrt(pow(target_base_link.x - current_point.x, 2) + pow(target_base_link.y - current_point.y, 2)) > 0.1) {  // 0.1m未満の距離になるまで移動を続ける
        ros::spinOnce();
        if(is_obstacle){
            ROS_INFO("change state");
            state = AVOIDANCE;
            break;
        }

        geometry_msgs::Twist cmd_vel;
        // 目標角度を計算
            // 角度差を-PIからPIの範囲に収める
        if (target_angle > M_PI) {
            double target_angle -= 2 * M_PI;
        } else if (target_angle < -M_PI) {
            double target_angle += 2 * M_PI;
        }
        if(flag==true){


            double angular_speed = 0.5;  // 回転速度 0.5rad/s
            // 角速度から回転にかかる時間を計算
            double rotation_time = std::abs(target_angle) / angular_speed;

            // 回転を開始する
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_speed;
            twist_pub.publish(cmd_vel);
            ros::Duration(rotation_time).sleep();
            flag=false;
            ROS_INFO("flag: %d", flag);
        }

        // 移動速度を設定
        cmd_vel.angular.z = 0.0;  // 回転速度は0に戻す
        cmd_vel.linear.x = speed;

        // メッセージを配信して移動を行う
        twist_pub.publish(cmd_vel);
        ros::Duration(0.1).sleep();

        // 位置を更新
        current_point.x = odom_point.x - start_x;
        current_point.y = odom_point.y - start_y;
    }
    flag=true;
    // 移動を停止
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    twist_pub.publish(stop_cmd);

    ros::Duration(1.0).sleep();
}


void moveAvoidance(ros::Publisher& twist_pub,const geometry_msgs::Point& target,const bool form)
{
    // 移動速度を設定
    speed= 0.2;  // 移動速度 0.5m/s
    current_point.x = 0.0;
    current_point.y = 0.0;
    current_point.z = 0.0; 

    // ロボットを目標座標に向かって移動させる
    while (ros::ok() && std::abs(target.y)-current_point.y > 0.1) {  // 0.1m未満の距離になるまで移動を続ける
        geometry_msgs::Twist cmd_vel;
        // 目標角度を計算
        if(form == true){
            double target_angle = M_PI /2;
        }else{
            double target_angle = -M_PI /2;
        }
            // 角度差を-PIからPIの範囲に収める
        // if (target_angle > M_PI) {
        //     target_angle -= 2 * M_PI;
        // } else if (target_angle < -M_PI) {
        //     target_angle += 2 * M_PI;
        // }
        if(flag==true){


            double angular_speed = 0.5;  // 回転速度 0.5rad/s
            // 角速度から回転にかかる時間を計算
            double rotation_time = std::abs(target_angle) / angular_speed;

            // 回転を開始する
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_speed;
            twist_pub.publish(cmd_vel);
            ros::Duration(rotation_time).sleep();
            flag=false;
            ROS_INFO("flag: %d", flag);
        }

        // 移動速度を設定
        cmd_vel.angular.z = 0.0;  // 回転速度は0に戻す
        cmd_vel.linear.x = speed;

        // メッセージを配信して移動を行う
        twist_pub.publish(cmd_vel);
        ros::Duration(0.1).sleep();


        current_point.y = current_point.y + 0.1 * speed;
    }
    flag=true;



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
                target.x = 0.0;  // ターゲットのx座標
                target.y = 1.0 ;  // ターゲットのy座標
                moveAvoidance(twist_pub,target,true);

                ROS_INFO("avoidance2");
                target.x = 0.0;  // ターゲットのx座標
                target.y = -2.0;  // ターゲットのy座標

                moveAvoidance(twist_pub,target,false);
                ROS_INFO("avoidance3");
                target.x = 0.0;  // ターゲットのx座標
                target.y = -1.0;  // ターゲットのy座標   
                moveAvoidance(twist_pub,target,false);
                is_obstacle=false;

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
