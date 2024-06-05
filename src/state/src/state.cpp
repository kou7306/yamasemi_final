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
double speed;
bool flag = true;
double distance1;
double distance2;
double distance3;
State state = STOP;
State current_state = RUN1;
// グローバル変数でtf::TransformListenerを保持
tf::TransformListener* tf_listener;
geometry_msgs::Twist move_cmd;
geometry_msgs::Twist cmd_vel_odom;
geometry_msgs::Point current_pose;
geometry_msgs::Point target;
geometry_msgs::Point current_point;
ros::Time start_time;
double robot_x, robot_y;
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;

geometry_msgs::Twist twist; // 指令する速度、角速度

std::map<std::string, double> params_; // パラメータをここに格納
geometry_msgs::PoseStamped goal; // 目標地点

// グローバル変数
geometry_msgs::TransformStamped global_transform;
tf::StampedTransform transform_tf;




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

// オドメトリのコールバック
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_x = msg->pose.pose.position.x;
	robot_y = msg->pose.pose.position.y;
	robot_r = msg->pose.pose.orientation;
}

// クォータニオンをオイラーに変換                                               
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

//　goalで指定した位置に近いかの判定を行う
int near_position(geometry_msgs::PoseStamped goal)
{
	double difx = robot_x - goal.pose.position.x;
	double dify = robot_y - goal.pose.position.y;
	return (sqrt(difx * difx + dify * dify) < 0.4);
}


void go_position(geometry_msgs::PoseStamped goal)
{
    double k_v = 0.1; // 速度の係数
    double k_w = 1.6; // 角速度の係数
	
	// 指令する速度と角速度
	double v = 0.0;
	double w = 0.0;

	//　角速度の計算
	double theta = atan2(goal.pose.position.y - robot_y, goal.pose.position.x - robot_x);
	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	while (yaw <= -M_PI || M_PI <= yaw)
	{
		if (yaw <= -M_PI)
			yaw = yaw + 2 * M_PI;
		else
			yaw = yaw - 2 * M_PI;
	}

	theta = theta - yaw; //thetaに目標とする点に向くために必要となる角度を格納

	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	w = k_w * theta;

	// 速度の計算(追従する点が自分より前か後ろかで計算を変更)
	if (theta <= M_PI / 2 && theta >= -M_PI / 2)
		v = k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
	else
		v = -k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
	
	// publishする値の格納
	twist.linear.x = v;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = w;

	std::cout << "v: " << v << ", w: " << w << std::endl;

}





// 障害物回避
void moveAvoidance(ros::Publisher& twist_pub,const double distance,const bool form)
{
    double target_angle;
    // 目標角度を計算
    if(form == true){
        target_angle = M_PI /2;
    }else{
        target_angle = -M_PI /2;
    }



    double angular_speed = 0.5;  // 回転速度 0.5rad/s
    // 角速度から回転にかかる時間を計算
    double time_to_rotation = std::abs(target_angle) / angular_speed;

    start_time = ros::Time::now();

    while(ros::ok() && (ros::Time::now() - start_time).toSec() < time_to_rotation){
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
        move_cmd.linear.y = 0.0;

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
    tf_listener = new tf::TransformListener();

    state = STOP;

	ros::Rate loop_rate(100);

	// odometryの値の初期化
	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;
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
                    
                    goal.pose.position.x = 4.5;  // ターゲットのx座標
                    goal.pose.position.y = 0.0;

                    while (ros::ok())
                    {
                        ros::spinOnce();
                        if(is_obstacle){
                            state = AVOIDANCE;
                            break;
                        }
                        

                        // 比例制御で近づき、近づき終えたら停止
                        go_position(goal);
                        if (near_position(goal))
                        {
                            twist.linear.x = 0.0;
                            twist.angular.z = 0.0;
                        }
                        twist_pub.publish(twist);

                        loop_rate.sleep();
                    }

                    state = RUN2;


                }
                break;


            case RUN2:
                current_state = RUN2;
                ROS_INFO("RUN2");
                
                goal.pose.position.x = 4.5;  // ターゲットのx座標
                goal.pose.position.y = 1.0;

                while (ros::ok())
                {
                    ros::spinOnce();
                    if(is_obstacle){
                        state = AVOIDANCE;
                        break;
                    }
                    

                    // 比例制御で近づき、近づき終えたら停止
                    go_position(goal);
                    if (near_position(goal))
                    {
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                    }
                    twist_pub.publish(twist);

                    loop_rate.sleep();
                }

                state = RUN3;
            
                break;


    

            case RUN3:
                current_state = RUN3;
                ROS_INFO("RUN3");
                
                goal.pose.position.x = 3.0;  // ターゲットのx座標
                goal.pose.position.y = -1.5;

                while (ros::ok())
                {
                    ros::spinOnce();
                    if(is_obstacle){
                        state = AVOIDANCE;
                        break;
                    }
                    

                    // 比例制御で近づき、近づき終えたら停止
                    go_position(goal);
                    if (near_position(goal))
                    {
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                    }
                    twist_pub.publish(twist);

                    loop_rate.sleep();
                }

                state = RUN4;
                
                break;

            case RUN4:
                current_state = RUN4;
                ROS_INFO("RUN4");
                
                goal.pose.position.x = 1.0;  // ターゲットのx座標
                goal.pose.position.y = 1.5;

                while (ros::ok())
                {
                    ros::spinOnce();
                    if(is_obstacle){
                        state = AVOIDANCE;
                        break;
                    }
                    

                    // 比例制御で近づき、近づき終えたら停止
                    go_position(goal);
                    if (near_position(goal))
                    {
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                    }
                    twist_pub.publish(twist);

                    loop_rate.sleep();
                }

                state = RUN5;
                
                
                break;

            case RUN5:
                current_state = RUN5;
                ROS_INFO("RUN5");
                goal.pose.position.x = 0.0;  // ターゲットのx座標
                goal.pose.position.y = 0.0;

                while (ros::ok())
                {
                    ros::spinOnce();
                    if(is_obstacle){
                        state = AVOIDANCE;
                        break;
                    }
                    

                    // 比例制御で近づき、近づき終えたら停止
                    go_position(goal);
                    if (near_position(goal))
                    {
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                    }
                    twist_pub.publish(twist);

                    loop_rate.sleep();
                }
                state = FINISH;
                    
                
                break;

            case AVOIDANCE:
                ROS_INFO("AVOIDANCE");
                ROS_INFO("avoidance1");
                distance1 = 1.0;
                moveAvoidance(twist_pub, distance1, true);

                ROS_INFO("avoidance2");
                distance2 = 2.0;
                moveAvoidance(twist_pub, distance2, false);

                ROS_INFO("avoidance3");
                distance3 = 1.0;
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
	}

	return 0;
}
