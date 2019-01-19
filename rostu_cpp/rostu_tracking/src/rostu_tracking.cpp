#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

struct passwd *pw1 = getpwuid(getuid());
string homeDir = pw1->pw_dir;

string calibrationFile[3];

Mat kernel = Mat::ones(9, 9, CV_8UC1);
Mat frame, depth_image, mask[3], hsv;

double lower[3][3], upper[3][3];

bool viewer = false;

class SubscriberAndPublisher {
private:
  bool findDepth;
  float distance, lastDistance;
  Point2f center;
  ros::Publisher ballCoor_pub;
  ros::Publisher ballDepth_pub;
  image_transport::Subscriber sub;
  image_transport::Subscriber sub2;
  geometry_msgs::Pose2D ball_coor;
  std_msgs::Float32 ball_depth;

public:
  SubscriberAndPublisher(ros::NodeHandle &nh, image_transport::ImageTransport &it) {
    sub = it.subscribe("kinect2/qhd/image_color", 1, &SubscriberAndPublisher::imageCallback, this);
    sub2 = it.subscribe("kinect2/qhd/image_depth_rect", 100, &SubscriberAndPublisher::depthCallback, this);
    ballCoor_pub = nh.advertise<geometry_msgs::Pose2D>("ballCoor", 1);
    ballDepth_pub = nh.advertise<std_msgs::Float32>("ballDepth", 1);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      frame = cv_bridge::toCvShare(msg, "8UC3")->image;

      //Thresholding For Convex Hull
      cvtColor(frame, hsv, COLOR_BGR2HSV);


      inRange(hsv, Scalar(0,177,151), Scalar(179,255,255), mask[0]);
      erode(mask[0], mask[0], 0, Point(-1,-1), 2);
      dilate(mask[0], mask[0], 0, Point(-1, -1), 2);
      morphologyEx(mask[0] ,mask[0], MORPH_CLOSE, kernel);

      for (int i = 1; i < 2; i++) {
        inRange(hsv, Scalar(lower[i][0],lower[i][1],lower[i][2]), Scalar(upper[i][0],upper[i][1],upper[i][2]), mask[i]);
        erode(mask[i], mask[i], 0, Point(-1,-1), 2);
        dilate(mask[i], mask[i], 0, Point(-1, -1), 2);
        morphologyEx(mask[i] ,mask[i], MORPH_CLOSE, kernel);
      }

      //Find Contours For Convex Hull
      Mat maskMerge = mask[0] + mask[1];
      vector<vector<Point> > hullCnts;
      vector<vector<Point> > cnts;
      vector<vector<Point> > largestHull(1);
      findContours(maskMerge.clone(), hullCnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      //Drawing Convex Hull
      Mat cropped_img, hsv_cropped_img;

      if (hullCnts.size() > 0) {
        vector<vector<Point> > hull(hullCnts.size());

        int largest_area = 0;
        int largest_contour_index = 0;

        for (int i = 0; i < hullCnts.size(); i++) {
          convexHull(Mat(hullCnts[i]), hull[i], false);
        }

        for (int i = 0; i < hull.size(); i++) {
          double a = contourArea(hull[i], false);
          if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;
          }
        }

        largestHull[0] = hull[largest_contour_index];
        drawContours(frame, largestHull, 0, Scalar(0, 0, 255), 2, 8);

        //Find Contours For Ball Tracking
        Mat mask_for_crop = Mat::zeros(frame.size(), CV_8UC3);
        fillPoly(mask_for_crop, largestHull, Scalar(255, 255, 255));
        bitwise_and(mask_for_crop, frame, cropped_img);

        //Thresholding For Ball Tracking
        cvtColor(cropped_img, hsv_cropped_img, COLOR_BGR2HSV);

        inRange(hsv_cropped_img, Scalar(0,177,151), Scalar(179,255,255), mask[0]);
        erode(mask[0], mask[0], 0, Point(-1,-1), 2);
        dilate(mask[0], mask[0], 0, Point(-1, -1), 2);
        morphologyEx(mask[0] ,mask[0], MORPH_CLOSE, kernel);

        findContours(mask[0].clone(), cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
      }

      //Drawing
      if (cnts.size() > 0) {
        Moments M;
        float radius;
        int largest_area = 0;
        int largest_contour_index = 0;

        for (int i = 0; i < cnts.size(); i++) {
          double a = contourArea(cnts[i], false);
          if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;
          }
        }

        minEnclosingCircle(cnts[largest_contour_index], center, radius);
        M = moments(cnts[largest_contour_index]);

        if (radius > 0) {
          circle(frame, center, int(radius), Scalar(0, 255, 0), 2);
        }
        ball_coor.x = center.x;
        ball_coor.y = center.y;
        findDepth = true;
      }
      else {
        findDepth = false;
        center.x = 0;
        center.y = 0;
        ball_coor.x = 0;
        ball_coor.y = 0;
      }

      ballCoor_pub.publish(ball_coor);
      if (viewer) {
        // imshow("mask", mask[0]);
        imshow("frame", frame);
      }

      if ((char)27 == (char)waitKey(1)) ros::shutdown();
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to '8UC3'.", msg->encoding.c_str());
    }
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      if (findDepth) {
        depth_image = cv_bridge::toCvShare(msg, "16UC1")->image;
        depth_image.convertTo(depth_image, CV_32F);

        distance = depth_image.at<float>(center);

        if (distance == 0) {
          distance = lastDistance;
        }

        ball_depth.data = distance;
        ROS_INFO("Depth: %f", distance);
        lastDistance = distance;
      }
      else {
        distance = 0;
        ball_depth.data = distance;
        ROS_INFO("Depth: %f", distance);
        lastDistance = distance;
      }

      ballDepth_pub.publish(ball_depth);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s to '16UC1'.", msg->encoding.c_str());
    }
  }
};

void readData() {
  calibrationFile[0] = homeDir + "/catkin_ws/src/rostu_cpp/data/ballValue";
  calibrationFile[1] = homeDir + "/catkin_ws/src/rostu_cpp/data/fieldValue";
  calibrationFile[2] = homeDir + "/catkin_ws/src/rostu_cpp/data/lineValue";

  ifstream inputFile[3];
  string calibrationData[3];

  for (int i = 0; i < 3; i++) {
    inputFile[i].open(calibrationFile[i].c_str());
    getline(inputFile[i], calibrationData[i]);
    inputFile[i].close();

    int it = 0;
    string dataTemp[6] = {"", "", "", "", "", ""};
    for (int j = 0; j < calibrationData[i].length(); j++) {
      if (calibrationData[i][j] == ',') {
        it = it + 1;
        dataTemp[it] = "";
      }
      else if (calibrationData[i][j] == ';') {
        break;
      }
      else {
        dataTemp[it] = dataTemp[it] + calibrationData[i][j];
      }
    }

    for (int k = 0; k < 3; k++) {
      lower[i][k] = atoi(dataTemp[k].c_str());
      upper[i][k] = atoi(dataTemp[k+3].c_str());
    }

    // cout << lower[i][0] << "," << lower[i][1] << "," << lower[i][2] << endl;
    // cout << upper[i][0] << "," << upper[i][1] << "," << upper[i][2] << endl << endl;
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rostu_tracking");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  SubscriberAndPublisher SAPObject(nh, it);

  readData();

  if (argc == 2) {
    if (strcmp(argv[1], "viewer") == 0) {
      viewer = true;
    }
  }

  ros::spin();
  destroyAllWindows();
  return 0;
}
