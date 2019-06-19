#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Int16MultiArray.h>

using namespace std;
using namespace cv;

string path = ros::package::getPath("rostu_v2");
YAML::Node speed_mul = YAML::LoadFile(path + "/cfg/rostu/speed_mul.yaml");
YAML::Node odom_mul = YAML::LoadFile(path + "/cfg/rostu/odom_mul.yaml");

int speed_x_slider_value = speed_mul["x_speed_mul"].as<int>();
int speed_y_slider_value = speed_mul["y_speed_mul"].as<int>();
int speed_w_slider_value = speed_mul["w_speed_mul"].as<int>();
int speed_x_slider_max = 500, speed_y_slider_max = 500, speed_w_slider_max = 250;

int odom_x_slider_value = odom_mul["x_odom_mul"].as<int>();
int odom_y_slider_value = odom_mul["y_odom_mul"].as<int>();
int odom_w_slider_value = odom_mul["w_odom_mul"].as<int>();
int odom_x_slider_max = 500, odom_y_slider_max = 500, odom_w_slider_max = 250;

void on_speed_x_trackbar( int, void* ) {
  if (speed_x_slider_value < 1) {
    speed_x_slider_value = 1;
  }

  speed_mul["x_speed_mul"] = speed_x_slider_value;
  ofstream speed_mul_fout(path + "/cfg/rostu/speed_mul.yaml");
  speed_mul_fout << speed_mul;
  speed_mul_fout.close();
}

void on_speed_y_trackbar( int, void* ) {
  if (speed_y_slider_value < 1) {
    speed_y_slider_value = 1;
  }

  speed_mul["y_speed_mul"] = speed_y_slider_value;
  ofstream speed_mul_fout(path + "/cfg/rostu/speed_mul.yaml");
  speed_mul_fout << speed_mul;
  speed_mul_fout.close();
}

void on_speed_w_trackbar( int, void* ) {
  if (speed_w_slider_value < 1) {
    speed_w_slider_value = 1;
  }

  speed_mul["w_speed_mul"] = speed_w_slider_value;
  ofstream speed_mul_fout(path + "/cfg/rostu/speed_mul.yaml");
  speed_mul_fout << speed_mul;
  speed_mul_fout.close();
}


void on_odom_x_trackbar( int, void* ) {
  if (odom_x_slider_value < 1) {
    odom_x_slider_value = 1;
  }

  odom_mul["x_odom_mul"] = odom_x_slider_value;
  ofstream odom_mul_fout(path + "/cfg/rostu/odom_mul.yaml");
  odom_mul_fout << odom_mul;
  odom_mul_fout.close();
}

void on_odom_y_trackbar( int, void* ) {
  if (odom_y_slider_value < 1) {
    odom_y_slider_value = 1;
  }

  odom_mul["y_odom_mul"] = odom_y_slider_value;
  ofstream odom_mul_fout(path + "/cfg/rostu/odom_mul.yaml");
  odom_mul_fout << odom_mul;
  odom_mul_fout.close();
}

void on_odom_w_trackbar( int, void* ) {
  if (odom_w_slider_value < 1) {
    odom_w_slider_value = 1;
  }

  odom_mul["w_odom_mul"] = odom_w_slider_value;
  ofstream odom_mul_fout(path + "/cfg/rostu/odom_mul.yaml");
  odom_mul_fout << odom_mul;
  odom_mul_fout.close();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_vel_mul");
  ros::NodeHandle nh;
  ros::Publisher vel_mul = nh.advertise<std_msgs::Int16MultiArray>("vel_mul", 1);
  ros::Publisher odom_mul = nh.advertise<std_msgs::Int16MultiArray>("odom_mul", 1);

  ros::Rate r(30);

  namedWindow("vel", 1);

  createTrackbar("X Speed", "vel", &speed_x_slider_value, speed_x_slider_max, on_speed_x_trackbar);
  createTrackbar("Y Speed", "vel", &speed_y_slider_value, speed_y_slider_max, on_speed_y_trackbar);
  createTrackbar("W Speed", "vel", &speed_w_slider_value, speed_w_slider_max, on_speed_w_trackbar);

  createTrackbar("Y Odom", "vel", &odom_y_slider_value, odom_y_slider_max, on_odom_y_trackbar);
  createTrackbar("X Odom", "vel", &odom_x_slider_value, odom_x_slider_max, on_odom_x_trackbar);
  createTrackbar("W Odom", "vel", &odom_w_slider_value, odom_w_slider_max, on_odom_w_trackbar);

  std_msgs::Int16MultiArray v_mul;
  v_mul.data.resize(3);

  std_msgs::Int16MultiArray o_mul;
  o_mul.data.resize(3);

  while (ros::ok()) {
    v_mul.data[0] = speed_x_slider_value;
    v_mul.data[1] = speed_y_slider_value;
    v_mul.data[2] = speed_w_slider_value;

    v_mul.data[0] = odom_x_slider_value;
    v_mul.data[1] = odom_y_slider_value;
    v_mul.data[2] = odom_w_slider_value;

    vel_mul.publish(v_mul);
    odom_mul.publish(o_mul);

    if ((char)27 == (char)waitKey(1)) ros::shutdown();

    ros::spinOnce();
    r.sleep();
  }

  destroyAllWindows();
  return 0;
}
