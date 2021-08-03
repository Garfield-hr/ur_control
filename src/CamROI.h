//
// Created by hairui on 6/30/21.
//

#ifndef MY_UR_CONTROL_CAMROI_H
#define MY_UR_CONTROL_CAMROI_H
#include<iostream>
#include<string>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;


class CamROI {
public:
    vector<Point2i> markers;
    int times = 0;
    string markers_win_name = "Please select four markers in order";
    int roi_size = 10;
    bool pouring_started = false;
    vector<int> upper_line_vec, lower_line_vec;
    vector<double> t_read, t_process;
    CamROI();
    virtual bool read_img(Mat& img);
    virtual void parallel_dis_img(Mat& img);
    void display_video();
    bool set_marker_position();
    friend void OnMouseAction(int event,int x,int y,int flags,void* cam_roi);
    bool read_img_roi(Mat& img_roi, Point2i& offset);
    bool if_pouring_started(Mat& img_bin);
    bool get_beer_ratio(double& ratio, double& liquid_level, bool debug = false, double time = 0);
    bool get_marker_position(vector<Point> &markers_position);

};
void OnMouseAction(int event,int x,int y,int flags,void* cam_roi);

void image_seg_two_lines(Mat& img, int& upper_line, int& lower_line);

int get_median(vector<int> arr);

int get_x_by_two_points_and_y(int y, Point p1, Point p2);


#endif //MY_UR_CONTROL_CAMROI_H
