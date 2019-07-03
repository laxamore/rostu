#include <iostream>
#include <fstream>
#include <chrono>

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

# define PI 3.14159265358979323846

using namespace std;
using namespace cv;

string path = ros::package::getPath("opencv_convexhull");
YAML::Node calibration_data = YAML::LoadFile(path + "/cfg/calib_value.yaml");
string calib = "";

Mat kernel = Mat::ones(3, 3, CV_8UC1);

int Lower[3][3] = {
  {calibration_data["field_h_low"].as<int>(), calibration_data["field_s_low"].as<int>(), calibration_data["field_v_low"].as<int>()},
  {calibration_data["line_h_low"].as<int>(), calibration_data["line_s_low"].as<int>(), calibration_data["line_v_low"].as<int>()},
  {calibration_data["ball_h_low"].as<int>(), calibration_data["ball_s_low"].as<int>(), calibration_data["ball_v_low"].as<int>()}
};
int Upper[3][3] = {
  {calibration_data["field_h_up"].as<int>(), calibration_data["field_s_up"].as<int>(), calibration_data["field_v_up"].as<int>()},
  {calibration_data["line_h_up"].as<int>(), calibration_data["line_s_up"].as<int>(), calibration_data["line_v_up"].as<int>()},
  {calibration_data["ball_h_up"].as<int>(), calibration_data["ball_s_up"].as<int>(), calibration_data["ball_v_up"].as<int>()}
};

int x_start = 0;
int y_start = 0;
int x_end = 0;
int y_end = 0;

bool sampling = false;
bool getROI = true;
bool saveROI = false;

double lower[3], upper[3];
int intLow[3], intUp[3];

static void leftClick(int event, int x, int y, int, void*) {
  if (event == EVENT_LBUTTONDOWN) {
    x_start = x;
    y_start = y;
    x_end = x;
    y_end = y;
    sampling = true;
    saveROI = false;
    getROI = false;
  }
  else if (event == EVENT_MOUSEMOVE) {
    if (sampling) {
      x_end = x;
      y_end = y;
    }
  }
  else if (event == EVENT_LBUTTONUP) {
    x_end = x;
    y_end = y;
    if (x_start > x_end) {
      int x_temp = x_start;
      x_start = x_end;
      x_end = x_temp;
    }
    if (y_start > y_end) {
      int y_temp = y_start;
      y_start = y_end;
      y_end = y_temp;
    }

    if (x_start == x_end || y_start == y_end) saveROI = false;
    else saveROI = true;
    sampling = false;
  }
}

void on_trackbar(int, void*) {
  ofstream fo(path + "/cfg/calib_value.yaml");
  if (!fo.is_open()) {
      cout << "unable to save calibration data to " << path << endl;
  }
  else {
    if (calib == "ball") {
        calibration_data["ball_h_low"] = intLow[0]; calibration_data["ball_h_up"] = intUp[0];
        calibration_data["ball_s_low"] = intLow[1]; calibration_data["ball_s_up"] = intUp[1];
        calibration_data["ball_v_low"] = intLow[2]; calibration_data["ball_v_up"] = intUp[2];
    }
    else if (calib == "field") {
        calibration_data["field_h_low"] = intLow[0]; calibration_data["field_h_up"] = intUp[0];
        calibration_data["field_s_low"] = intLow[1]; calibration_data["field_s_up"] = intUp[1];
        calibration_data["field_v_low"] = intLow[2]; calibration_data["field_v_up"] = intUp[2];
    }
    else if (calib == "line") {
        calibration_data["line_h_low"] = intLow[0]; calibration_data["line_h_up"] = intUp[0];
        calibration_data["line_s_low"] = intLow[1]; calibration_data["line_s_up"] = intUp[1];
        calibration_data["line_v_low"] = intLow[2]; calibration_data["line_v_up"] = intUp[2];
    }

    fo << calibration_data;
    fo.close();
  }
}

void calibration(Mat frame) {
    Mat mask, hsv;

    if (!getROI) {
      if (!sampling && !saveROI) {
        imshow("frame", frame);
      }
      else if (sampling && !saveROI) {
        rectangle(frame, Point(x_start, y_start), Point(x_end, y_end), Scalar(0, 255, 0), 2);
        imshow("frame", frame);
      }
      else if (saveROI) {
        rectangle(frame, Point(x_start, y_start), Point(x_end, y_end), Scalar(0, 255, 0), 2);
        imshow("frame", frame);

        Mat roi = frame(Range(y_start, y_end), Range(x_start, x_end));
        Mat hsvRoi, splitHsv[3];
        cvtColor(roi, hsvRoi, COLOR_BGR2HSV);
        split(hsvRoi, splitHsv);
        minMaxLoc(splitHsv[0], &lower[0], &upper[0]);
        minMaxLoc(splitHsv[1], &lower[1], &upper[1]);
        minMaxLoc(splitHsv[2], &lower[2], &upper[2]);

        setTrackbarPos("H Low", "trackbar", (int)lower[0]);
        setTrackbarPos("S Low", "trackbar", (int)lower[1]);
        setTrackbarPos("V Low", "trackbar", (int)lower[2]);

        setTrackbarPos("H Up", "trackbar", (int)upper[0]);
        setTrackbarPos("S Up", "trackbar", (int)upper[1]);
        setTrackbarPos("V Up", "trackbar", (int)upper[2]);

        ofstream fo(path.c_str());
        if (!fo.is_open()) {
            cout << "unable to save calibration data to " << path << endl;
        }
        else {
          fo << lower[0] << "," << lower[1] << "," << lower[2] << ",";
          fo << upper[0] << "," << upper[1] << "," << upper[2] << ";";
          fo.close();
        }
        saveROI = false;
        getROI = true;
      }
    }
    else {
      for (int i = 0; i < 3; i++) {
          lower[i] = (double)intLow[i];
          upper[i] = (double)intUp[i];
      }

      cvtColor(frame, hsv, COLOR_BGR2HSV);
      inRange(hsv, Scalar(lower[0],lower[1],lower[2]), Scalar(upper[0],upper[1],upper[2]), mask);
      erode(mask, mask, 0, Point(-1,-1), 2);
      dilate(mask, mask, 0, Point(-1, -1), 2);
      morphologyEx(mask ,mask, MORPH_CLOSE, kernel);

      //Find Contours
      Mat maskColone = mask.clone();
      vector<vector<Point> > cnts;
      findContours(maskColone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      //Drawing
      if (cnts.size() > 0) {
        Moments M;
        Point2f center;
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
      }

      imshow("mask", mask);
      imshow("frame", frame);
    }
}

char config = 'c';
int framePause = 0;

int main(int argc, char* argv[]) {
    if (argc > 2) {
        cout << "Too much argument" << endl;
        return 0;
    }

    if (strcmp("c", argv[1]) != 0 &&
        strcmp("C", argv[1]) != 0 &&
        strcmp("n", argv[1]) != 0 &&
        strcmp("N", argv[1]) != 0) {
        cout << "Bad argument" << endl;
        return 0;
    }

    if (strcmp("c", argv[1]) == 0 || strcmp("C", argv[1]) == 0) {
        config = 'c';
    }
    else if (strcmp("n", argv[1]) == 0 || strcmp("N", argv[1]) == 0) {
        config = 'n';
    }

    auto printFps = std::chrono::high_resolution_clock::now();
    VideoCapture cap(path + "/data/video.mp4"); 
    
    if (!cap.isOpened()){
        return -1;
    }

    Mat frame, hsv, mask[3], cropped_img;
    
    while(1) {
        auto start = std::chrono::high_resolution_clock::now();

        cap >> frame;
    
        if (frame.empty()) {
            break;
        }
    
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        if (config == 'c') {
            for (int i = 0; i < 2; i++) {
                inRange(hsv, Scalar(Lower[i][0],Lower[i][1],Lower[i][2]), Scalar(Upper[i][0],Upper[i][1],Upper[i][2]), mask[i]);
                erode(mask[i], mask[i], 0, Point(-1,-1), 2);
                dilate(mask[i], mask[i], 0, Point(-1, -1), 2);
                morphologyEx(mask[i] ,mask[i], MORPH_CLOSE, kernel);
            }

            Mat maskMerge = mask[0];
            Mat im_floodfill = maskMerge.clone();
            floodFill(im_floodfill, Point(0,0), Scalar(255));
            Mat im_floodfill_inv;
            bitwise_not(im_floodfill, im_floodfill_inv);
            maskMerge = (maskMerge | im_floodfill_inv);

            vector<vector<Point> > hullCnts;
            vector<vector<Point> > cnts;
            vector<vector<Point> > largestHull(1);
            findContours(maskMerge.clone(), hullCnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            
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
                Mat mask_for_crop = Mat::zeros(frame.size(), CV_8UC3);
                fillPoly(mask_for_crop, largestHull, Scalar(255, 255, 255));
                bitwise_and(mask_for_crop, frame, cropped_img);
                
                Mat hsv_cropped_img;
                cvtColor(cropped_img, hsv_cropped_img, COLOR_BGR2HSV);
                inRange(hsv_cropped_img, Scalar(Lower[2][0],Lower[2][1],Lower[2][2]), Scalar(Upper[2][0],Upper[2][1],Upper[2][2]), mask[2]);
                erode(mask[2], mask[2], 0, Point(-1,-1), 2);
                dilate(mask[2], mask[2], 0, Point(-1, -1), 2);
                morphologyEx(mask[2] ,mask[2], MORPH_CLOSE, kernel);

                Mat maskClone = mask[2].clone();
                vector<vector<Point> > cnts;
                findContours(maskClone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

                if (cnts.size() > 0) {
                    Moments M;
                    Point2f center;
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
                        circle(cropped_img, center, int(radius), Scalar(0, 255, 0), 2);
                    }

                    // double length = sqrt(pow(center.x - 300, 2) + pow(center.y - 300, 2));
                    // double angle = (atan2(300 - 0, 300 - 300) - atan2(300 - center.y, 300 - center.x)) * 360 / (2 * PI) * -1;
                    // line(frame, Point(300, 300), center, Scalar(0, 0, 255), 1);
                    // putText(frame, to_string(length), Point(center.x - 25, center.y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2);
                    // putText(frame, to_string(angle), Point(center.x - 25, center.y + 25), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2);
                }
            }
            
            imshow( "cropped_img", cropped_img );
            imshow( "frame", frame );
        }
        else if (config == 'n') {
            inRange(hsv, Scalar(Lower[2][0],Lower[2][1],Lower[2][2]), Scalar(Upper[2][0],Upper[2][1],Upper[2][2]), mask[2]);
            erode(mask[2], mask[2], 0, Point(-1,-1), 2);
            dilate(mask[2], mask[2], 0, Point(-1, -1), 2);
            morphologyEx(mask[2] ,mask[2], MORPH_CLOSE, kernel);

            Mat maskClone = mask[2].clone();
            vector<vector<Point> > cnts;
            findContours(maskClone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            if (cnts.size() > 0) {
                Moments M;
                Point2f center;
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

                // double length = sqrt(pow(center.x - 300, 2) + pow(center.y - 300, 2));
                // double angle = (atan2(300 - 0, 300 - 300) - atan2(300 - center.y, 300 - center.x)) * 360 / (2 * PI) * -1;
                // line(frame, Point(300, 300), center, Scalar(0, 0, 255), 1);
                // putText(frame, to_string(length), Point(center.x - 25, center.y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2);
                // putText(frame, to_string(angle), Point(center.x - 25, center.y + 25), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2);
            }
            
            imshow( "frame", frame );
        }
        
        // imshow( "mask", maskMerge );

        char c = (char)waitKey(25);
        if (c == 32 || framePause == 182) {
            // destroyWindow("cropped_img");
            
            x_start = 0;
            y_start = 0;
            x_end = 0;
            y_end = 0;

            sampling = false;
            getROI = true;
            saveROI = false;

            while(1) {
                c = (char)waitKey(25);
                if (c == 102) {
                    namedWindow("trackbar", CV_WINDOW_FREERATIO);
                    setMouseCallback("frame", leftClick);

                    intLow[0] = Lower[0][0]; lower[0] = double(intLow[0]); intUp[0] = Upper[0][0]; upper[0] = double(intUp[0]);
                    intLow[1] = Lower[0][1]; lower[1] = double(intLow[1]); intUp[1] = Upper[0][1]; upper[1] = double(intUp[1]);
                    intLow[2] = Lower[0][2]; lower[2] = double(intLow[2]); intUp[2] = Upper[0][2]; upper[2] = double(intUp[2]);

                    createTrackbar("H Low", "trackbar", &intLow[0], 255, on_trackbar);
                    createTrackbar("S Low", "trackbar", &intLow[1], 255, on_trackbar);
                    createTrackbar("V Low", "trackbar", &intLow[2], 255, on_trackbar);

                    createTrackbar("H Up", "trackbar", &intUp[0], 255, on_trackbar);
                    createTrackbar("S Up", "trackbar", &intUp[1], 255, on_trackbar);
                    createTrackbar("V Up", "trackbar", &intUp[2], 255, on_trackbar);

                    while(1) {
                        calib = "field";
                        calibration(frame.clone());
                        
                        c = (char)waitKey(25);
                        if (c == 27) {
                            break;
                        }
                    }
                    destroyWindow("trackbar");
                }
                else if (c == 108) {
                    namedWindow("trackbar", CV_WINDOW_FREERATIO);
                    setMouseCallback("frame", leftClick);

                    intLow[0] = Lower[1][0]; lower[0] = double(intLow[0]); intUp[0] = Upper[1][0]; upper[0] = double(intUp[0]);
                    intLow[1] = Lower[1][1]; lower[1] = double(intLow[1]); intUp[1] = Upper[1][1]; upper[1] = double(intUp[1]);
                    intLow[2] = Lower[1][2]; lower[2] = double(intLow[2]); intUp[2] = Upper[1][2]; upper[2] = double(intUp[2]);

                    createTrackbar("H Low", "trackbar", &intLow[0], 255, on_trackbar);
                    createTrackbar("S Low", "trackbar", &intLow[1], 255, on_trackbar);
                    createTrackbar("V Low", "trackbar", &intLow[2], 255, on_trackbar);

                    createTrackbar("H Up", "trackbar", &intUp[0], 255, on_trackbar);
                    createTrackbar("S Up", "trackbar", &intUp[1], 255, on_trackbar);
                    createTrackbar("V Up", "trackbar", &intUp[2], 255, on_trackbar);

                    while(1) {
                        calib = "line";
                        calibration(frame.clone());
                        
                        c = (char)waitKey(25);
                        if (c == 27) {
                            break;
                        }
                    }
                    destroyWindow("trackbar");
                }
                else if (c == 98) {
                    namedWindow("trackbar", CV_WINDOW_FREERATIO);
                    setMouseCallback("frame", leftClick);

                    intLow[0] = Lower[2][0]; lower[0] = double(intLow[0]); intUp[0] = Upper[2][0]; upper[0] = double(intUp[0]);
                    intLow[1] = Lower[2][1]; lower[1] = double(intLow[1]); intUp[1] = Upper[2][1]; upper[1] = double(intUp[1]);
                    intLow[2] = Lower[2][2]; lower[2] = double(intLow[2]); intUp[2] = Upper[2][2]; upper[2] = double(intUp[2]);

                    createTrackbar("H Low", "trackbar", &intLow[0], 255, on_trackbar);
                    createTrackbar("S Low", "trackbar", &intLow[1], 255, on_trackbar);
                    createTrackbar("V Low", "trackbar", &intLow[2], 255, on_trackbar);

                    createTrackbar("H Up", "trackbar", &intUp[0], 255, on_trackbar);
                    createTrackbar("S Up", "trackbar", &intUp[1], 255, on_trackbar);
                    createTrackbar("V Up", "trackbar", &intUp[2], 255, on_trackbar);

                    while(1) {
                        calib = "ball";
                        calibration(frame.clone());
                        
                        c = (char)waitKey(25);
                        if (c == 27) {
                            break;
                        }
                    }
                    destroyWindow("trackbar");
                }
                else if (c == 32) {
                    break;
                }
            }
        }
        else if (c == 27) {
            break;
        }
        
        
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        std::chrono::duration<double> printTime = std::chrono::high_resolution_clock::now() - printFps;
        if (printTime.count() >= 1) {
            cout << "FPS : " << 1 / elapsed.count() << endl;
            printFps = std::chrono::high_resolution_clock::now();
        }

        framePause++;
    }
    
    cap.release();
    destroyAllWindows();
        
    return 0;
}