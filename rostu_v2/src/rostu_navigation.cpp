#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <rostu_v2/BallCoor.h>
#include <rostu_v2/Dribling.h>
#include <rostu_v2/Kicker.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>

#define PI 3.14159265358979323846

using namespace std;

string path = ros::package::getPath("rostu_v2");
YAML::Node team_config = YAML::LoadFile(path + "/cfg/rostu/team.yaml");
YAML::Node speed_mul = YAML::LoadFile(path + "/cfg/rostu/speed_mul.yaml");
YAML::Node odom_mul = YAML::LoadFile(path + "/cfg/rostu/odom_mul.yaml");

double vx;
double vy;
double vth;

double vx_speed = float(speed_mul["x_speed_mul"].as<int>()) / float(100);
double vy_speed = float(speed_mul["y_speed_mul"].as<int>()) / float(100);
double vth_speed = float(speed_mul["w_speed_mul"].as<int>()) / float(100);

double odom_x;
double odom_y;
double odom_th;

double odom_vx;
double odom_vy;
double odom_vth;

double vx_odom = float(odom_mul["x_odom_mul"].as<int>()) / float(100);
double vy_odom = float(odom_mul["y_odom_mul"].as<int>()) / float(100);
double vth_odom = float(odom_mul["w_odom_mul"].as<int>()) / float(100);

double dt;
double delta_x;
double delta_y;
double delta_th;

bool step_back = false;
bool find_ball = false;
bool ball_detect, got_ball;
double ball_distance_from_frame = 200;
double ball_angle_from_frame;
double kicker_volt;
string referee_command = "S";
string robot_team = team_config["team"].as<string>();
string robotName = "";
bool startGame = false;

geometry_msgs::Twist rostu_cmdvel;
geometry_msgs::Twist cmdvel;
rostu_v2::Dribling dribling;
rostu_v2::Kicker kicker_msg;
geometry_msgs::Quaternion robot_orientation;
geometry_msgs::PoseStamped goal_publish_msg;
actionlib_msgs::GoalID cancel_goal_msg;;

ros::Publisher cancel_goal_pub;
ros::Publisher publish_goal;

double lastGoalPosX, lastGoalPosY, lastGoalPosZ, lastGoalPosW;
bool waitDelayCondition = false;
double waitDelaySec = 0.0;
bool robot_kickoff = false, robot_freekick = false, robot_goalkick = false, robot_throw_in = false, robot_corner = false, robot_penalty = false;
bool enemy_kickoff = false, enemy_freekick = false, enemy_goalkick = false, enemy_throw_in = false, enemy_corner = false, enemy_penalty = false;

ros::Time current_time, last_time, counter, waitDelay, delayUniversal;

double kp = 0.05, kd = 0.03;
double setpoint = 0.0;
double error;
double P_Value;
double D_Value;
double PD_Value;
double lastError;

double robot_center_pose_x, robot_center_pose_y, robot_angle;
double robot_end_pose_x, robot_end_pose_y;
double robot_angle_from_goal;
int robot_status = 3, robot_find_ball_pose = 0;
double gawang_point_x, gawang_point_y;

double PD_Controller(double th) {
  error = th;
  P_Value = kp * error;
  D_Value = (lastError - error) * kd;
  PD_Value = P_Value + D_Value;
  lastError = error;

  return PD_Value;
}

void odom_mul_callback(const std_msgs::Int16MultiArray& msg) {
  vx_odom = float(msg.data[0]) / float(100);
  vy_odom = float(msg.data[1]) / float(100);
  vth_odom = float(msg.data[2]) / float(100);
}

void speed_mul_callback(const std_msgs::Int16MultiArray& msg) {
  vx_speed = float(msg.data[0]) / float(100);
  vy_speed = float(msg.data[1]) / float(100);
  vth_speed = float(msg.data[2]) / float(100);
}

void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  odom_vx = msg.linear.x * vx_odom;
  odom_vy = msg.linear.y * vy_odom;
  odom_vth = msg.angular.z * vth_odom;

  delta_x = (odom_vx * cos(odom_th) - odom_vy * sin(odom_th)) * dt;
  delta_y = (odom_vx * sin(odom_th) + odom_vy * cos(odom_th)) * dt;
  delta_th = odom_vth * dt;

  odom_x += delta_x;
  odom_y += delta_y;
  odom_th += delta_th;

  delta_x = 0;
  delta_y = 0;
  delta_th = 0;

  vx = msg.linear.x * vx_speed;
  vy = msg.linear.y * vy_speed;
  vth = msg.angular.z * vth_speed;

  rostu_cmdvel.linear.x = vx;
  rostu_cmdvel.linear.y = vy;
  rostu_cmdvel.angular.z = vth;
}

void ball_coor_callback(const rostu_v2::BallCoor& msg) {
  ball_detect = msg.ball_detect;
  ball_distance_from_frame = msg.distance;
  ball_angle_from_frame = msg.angle;

  if (ball_angle_from_frame < 5 && ball_angle_from_frame > -5) {
    ball_angle_from_frame = 0;
  }
  if (ball_detect) {
    if (ball_distance_from_frame < 50) {
      dribling.d1pwm1 = 200;
      dribling.d1pwm2 = 0;
      dribling.d2pwm1 = 200;
      dribling.d2pwm2 = 0;
    }
  }
  else {
    if (!step_back) {
      dribling.d1pwm1 = 0;
      dribling.d1pwm2 = 0;
      dribling.d2pwm1 = 0;
      dribling.d2pwm2 = 0;
    }
  }
}

void dribling_callback(const rostu_v2::Dribling& msg) {
  got_ball = msg.got_ball;
}

void kicker_callback(const rostu_v2::Kicker& msg) {
  kicker_volt = msg.cap_volt;
  kicker_msg.cap_volt = kicker_volt;
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  robot_center_pose_x = msg.pose.pose.position.x;
  robot_center_pose_y = msg.pose.pose.position.y;
  robot_orientation = msg.pose.pose.orientation;
  robot_angle = tf::getYaw(robot_orientation);
  robot_end_pose_x = robot_center_pose_x - sin(robot_angle) * 0.25;
  robot_end_pose_y = robot_center_pose_y - cos(robot_angle) * 0.25;
  double angle = atan2(robot_center_pose_y - robot_end_pose_y, robot_center_pose_x - robot_end_pose_x) - atan2(gawang_point_x - robot_center_pose_x, gawang_point_y - robot_center_pose_y);
  robot_angle_from_goal = angle * 360 / (2 * PI);
  if (robot_angle_from_goal < 10 && robot_angle_from_goal > -10) {
    robot_angle_from_goal = 0;
  }
}

void move_base_status_callback(const actionlib_msgs::GoalStatusArray& msg) {
  if (msg.status_list.size() > 0) {
    int msg_size = msg.status_list.size();
    robot_status = msg.status_list[msg_size - 1].status;
  }

  if (startGame) {
    if (!ball_detect) {
      if (robot_status == 3 || robot_status == 2) {
        find_ball = true;
        if ((current_time - counter).toSec() > 1.5) {
          goal_publish_msg.header.frame_id = "map";
          if (robot_find_ball_pose == 0) {
            goal_publish_msg.pose.position.x = 3;
            goal_publish_msg.pose.position.y = 3.75;
            goal_publish_msg.pose.orientation.w = 1.0;
          }
          else if (robot_find_ball_pose == 1) {
            goal_publish_msg.pose.position.x = 5.25;
            goal_publish_msg.pose.position.y = 6.125;
            goal_publish_msg.pose.orientation.w = 1.0;
          }
          else if (robot_find_ball_pose == 2) {
            goal_publish_msg.pose.position.x = 7.5;
            goal_publish_msg.pose.position.y = 3.75;
            goal_publish_msg.pose.orientation.w = 1.0;
          }
          else if (robot_find_ball_pose == 3) {
            goal_publish_msg.pose.position.x = 5.25;
            goal_publish_msg.pose.position.y = 1.375;
            goal_publish_msg.pose.orientation.w = 1.0;
          }
          robot_find_ball_pose++;
          publish_goal.publish(goal_publish_msg);
        }
      }
      else {
        counter = ros::Time::now();
      }
    }
    else if (ball_detect && robot_status == 1 && find_ball) {
      find_ball = false;
      cancel_goal_pub.publish(cancel_goal_msg);
    }

    if (robot_find_ball_pose > 3) {
      robot_find_ball_pose = 0;
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc == 4) {
    robotName = argv[1];
  }
  
  ros::init(argc, argv, "rostu_navigation");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("cmd_vel", 1, cmd_vel_callback);
  ros::Subscriber sub2 = nh.subscribe("rostu/ball_coor", 1, ball_coor_callback);
  ros::Subscriber sub3 = nh.subscribe("rostu/dribling", 1, dribling_callback);
  ros::Subscriber sub4 = nh.subscribe("rostu/kicker", 1, kicker_callback);
  ros::Subscriber sub5 = nh.subscribe("amcl_pose", 1, amcl_pose_callback);
  ros::Subscriber sub6 = nh.subscribe("move_base/status", 1, move_base_status_callback);
  ros::Subscriber sub7 = nh.subscribe("vel_mul", 1, speed_mul_callback);
  ros::Subscriber sub8 = nh.subscribe("odom_mul", 1, odom_mul_callback);
  ros::Publisher nav_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher rostu_vel = nh.advertise<geometry_msgs::Twist>("rostu/cmd_vel", 1);
  ros::Publisher dribling_pub = nh.advertise<rostu_v2::Dribling>("rostu/dribling", 1);
  ros::Publisher kicker_pub = nh.advertise<rostu_v2::Kicker>("rostu/kicker", 1);
  cancel_goal_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
  publish_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

  ros::Rate r(30);

  tf::TransformBroadcaster bll_broadcaster;
  tf::TransformBroadcaster bl_broadcaster;
  tf::TransformBroadcaster bf_broadcaster;

  odom_x = 0.0;
  odom_y = 0.0;
  odom_th = 0.0;

  ros::Time kickoff_pass_time;

  current_time = ros::Time::now();
  last_time = ros::Time::now();
  counter = ros::Time::now();
  waitDelay = ros::Time::now();
  kickoff_pass_time = ros::Time::now();

  double angular_z_vel;

  if( robot_team == "M") {
    gawang_point_x = 1.25;
    gawang_point_y = 3.875;
  }
  else if( robot_team == "C") {
    gawang_point_x = 9.75;
    gawang_point_y = 3.875;
  }

  while (ros::ok()) {
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    if(referee_command == "s") { //Robot Start
      if (startGame) {
        if (got_ball && robot_status != 1) {
          if (robot_kickoff) {
            dribling.d1pwm1 = 0;
            dribling.d1pwm2 = 255;
            dribling.d2pwm1 = 0;
            dribling.d2pwm2 = 255;
            dribling_pub.publish(dribling);
          }
          else if (robot_angle_from_goal != 0) {
            double output = PD_Controller(robot_angle_from_goal);
            if (output > 2) {
              output = 2;
            }

            if (output < -2) {
              output = -2;
            }

            if (output > 0 ) {
              cmdvel.angular.z = output;
            }
            else if (output < 0) {
              cmdvel.angular.z = output;
            }
            else {
              cmdvel.angular.z = 0;
            }
            dribling.d1pwm1 = 200;
            dribling.d1pwm2 = 0;
            dribling.d2pwm1 = 200;
            dribling.d2pwm2 = 0;
          }
          else {
            if (kicker_volt > 250) {
              kicker_msg.kick_pwm = 255;
            }
            cmdvel.angular.z = 0;
            dribling.d1pwm1 = 0;
            dribling.d1pwm2 = 0;
            dribling.d2pwm1 = 0;
            dribling.d2pwm2 = 0;
          }
          cmdvel.linear.x = 0.0;
          cmd_vel.publish(cmdvel);
        }
        else if (ball_detect && robot_status != 1) {
          lastGoalPosX = 0; lastGoalPosY = 0; lastGoalPosZ = 0; lastGoalPosW = 0;
          double output = PD_Controller(ball_angle_from_frame);

          if (output > 0 ) {
            angular_z_vel = PD_Value * -1;
          }
          else if (output < 0) {
            angular_z_vel = PD_Value * -1;
          }
          else {
            angular_z_vel = 0;
          }

          if (angular_z_vel > 3) {
            angular_z_vel = 3;
          }

          if (angular_z_vel < -3) {
            angular_z_vel = -3;
          }

          if (ball_angle_from_frame < 45 && ball_angle_from_frame > -45) {
            if (ball_distance_from_frame > 100) {
              cmdvel.linear.x = 1.5;
            }
            else if (ball_distance_from_frame <= 100 && ball_distance_from_frame > 25) {
              cmdvel.linear.x = 1.25;
            }
            else if (ball_distance_from_frame <= 25) {
              if (ball_angle_from_frame < 20 && ball_angle_from_frame > -20) {
                cmdvel.linear.x = 0.5;
              }
            }
          }
          else {
    	       cmdvel.linear.x = 0;
          }

          cmdvel.angular.z = angular_z_vel;
          cmd_vel.publish(cmdvel);
        }
        else if (!step_back) {
          dribling.d1pwm1 = 0;
          dribling.d1pwm2 = 0;
          dribling.d2pwm1 = 0;
          dribling.d2pwm2 = 0;
        }
      }
    }

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th);

    geometry_msgs::TransformStamped bll_trans;
    bll_trans.header.stamp = current_time;
    bll_trans.header.frame_id = robotName + "/base_link";
    bll_trans.child_frame_id = robotName + "/base_laser_link";

    bll_trans.transform.translation.x = 0;
    bll_trans.transform.translation.y = 0;
    bll_trans.transform.translation.z = 0.2;
    bll_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    bll_broadcaster.sendTransform(bll_trans);

    geometry_msgs::TransformStamped bl_trans;
    bl_trans.header.stamp = current_time;
    bl_trans.header.frame_id = robotName + "/base_footprint";
    bl_trans.child_frame_id = robotName + "/base_link";

    bl_trans.transform.translation.x = 0;
    bl_trans.transform.translation.y = 0;
    bl_trans.transform.translation.z = 0.0;
    bl_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    bl_broadcaster.sendTransform(bl_trans);

    geometry_msgs::TransformStamped bf_trans;
    bf_trans.header.stamp = current_time;
    bf_trans.header.frame_id = robotName + "/odom";
    bf_trans.child_frame_id = robotName + "/base_footprint";

    bf_trans.transform.translation.x = odom_x;
    bf_trans.transform.translation.y = odom_y;
    bf_trans.transform.translation.z = 0.0;
    bf_trans.transform.rotation = odom_quat;
    bf_broadcaster.sendTransform(bf_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = robotName + "/odom";

    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "";
    odom.twist.twist.linear.x = odom_vx;
    odom.twist.twist.linear.y = odom_vy;
    odom.twist.twist.angular.z = odom_vth;

    rostu_vel.publish(rostu_cmdvel);
    dribling_pub.publish(dribling);
    kicker_pub.publish(kicker_msg);

    kicker_msg.kick_pwm = 0;
    rostu_cmdvel.linear.x = 0;
    rostu_cmdvel.linear.y = 0;
    rostu_cmdvel.linear.z = 0;
    rostu_cmdvel.angular.x = 0;
    rostu_cmdvel.angular.y = 0;
    rostu_cmdvel.angular.z = 0;

    last_time = current_time;
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


