//
// Created by hairui on 6/29/21.
//
#include "CamROI.h"
#include <time.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace cv;

class ParaDis{
private:
    Mat img;
    bool updated = false;
    boost::mutex mt;
public:
    ParaDis(){}
    bool display = true;
    void update_img(Mat& img_input){
        mt.lock();
        this->img = img_input;
        updated = true;
        mt.unlock();
    }
    friend void display_img(ParaDis* p);

};
void display_img(ParaDis* p){
    string win_name = "parallel dis";
    Mat img_show;
    namedWindow(win_name, WINDOW_AUTOSIZE);
    while(p->display){
        p->mt.lock();
        if (p->updated){
            img_show = p->img;
            p->updated = false;
            p->mt.unlock();
            imshow(win_name, img_show);
            waitKey(33);
        }
        else{
            p->mt.unlock();
        }
    }
}

class CamRoiVc:public CamROI{
private:
    VideoCapture* vc;
    ParaDis* pd;

public:
    explicit CamRoiVc(VideoCapture* vc, ParaDis* pd){
        this->vc = vc;
        this->pd = pd;
        set_marker_position();
    }
    bool read_img(Mat& img) override{
        bool ret = vc->read(img);
        return ret;
    }
    void parallel_dis_img(Mat& img) override{
        pd->update_img(img);
    }
};

int main(){
    string pic_dir = "/home/hairui/Pictures/experiment/";
    string pic_name = "bad_cup.jpeg";
    string video_dir = "/home/hairui/Videos/experiments/";
    string video_name = "618-1.avi";
//    Mat img = imread(pic_dir+pic_name);
//    Mat img_bin;
//    threshold(img, img_bin, 160, 255, THRESH_BINARY);
//    int ul, ll;
//    clock_t t1 = clock();
//    image_seg_two_lines(img_bin, ul, ll);
//    cout << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << endl;
//    line(img, Point(0, ul), Point(img.cols, ul), Scalar(0, 0, 255), 5, CV_AA);
//    line(img, Point(0, ll), Point(img.cols, ll), Scalar(0, 0, 255), 5, CV_AA);
//    imshow("img", img);
//    waitKey(0);
    VideoCapture vc;
    ParaDis pd;
    Mat frame;
    frame = vc.open(video_dir+video_name);
    if(!vc.isOpened()){
        cout<<"can't open file "<<video_dir+video_name<<endl;
        return -1;
    }
    CamRoiVc my_vc(&vc, &pd);
    boost::thread t1(boost::bind(display_img, &pd));
    my_vc.display_video();
    return 0;
}
