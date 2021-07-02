//
// Created by hairui on 6/29/21.
//
#include "CamROI.h"
#include <time.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <numeric>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "xiApiPlusOcv.hpp"


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
            if (waitKey(33) == 27) p->display = false;
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

class CamRoiXimea:public CamROI{
private:
    ParaDis* pd;
    xiAPIplusCameraOcv* cam;
public:
    explicit CamRoiXimea(xiAPIplusCameraOcv* cam, ParaDis* pd){
        this->pd = pd;
        this->cam = cam;
        set_marker_position();
    }
    bool read_img(Mat& img) override{
        img = cam->GetNextImageOcvMat();
        return true;
    }
    void parallel_dis_img(Mat& img) override{
        pd->update_img(img);
    }
};

int func1(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("BeerPouring", 1000);
    string pic_dir = "/home/hairui/Pictures/experiment/";
    string pic_name = "bad_cup.jpeg";
    string video_dir = "/home/liangxiao/Videos/";
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
    boost::thread th1(boost::bind(display_img, &pd));
    Mat img;
    clock_t t1 = clock();
    vector<double> t_vec;
    double liquid_level, beer_ratio;
    while (my_vc.get_beer_ratio(beer_ratio, liquid_level,true)){
        t_vec.emplace_back((clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000);
        t1 = clock();
        std_msgs::Float64MultiArray data;
        data.data.emplace_back(beer_ratio);
        data.data.emplace_back(liquid_level);
        data.data.emplace_back(t_vec.back());
        chatter_pub.publish(data);
    }
    cout<<"ave time "<<accumulate(t_vec.begin(), t_vec.end(), 0.0) / t_vec.size()<<endl;
    cout<<"ave read time "<<accumulate(my_vc.t_read.begin(), my_vc.t_read.end(), 0.0) / my_vc.t_read.size()<<endl;
    cout<<"ave thresh time "<<accumulate(my_vc.t_thresh.begin(), my_vc.t_thresh.end(), 0.0) / my_vc.t_thresh.size()<<endl;
    cout<<"ave seg time "<<accumulate(my_vc.t_seg.begin(), my_vc.t_seg.end(), 0.0) / my_vc.t_seg.size()<<endl;
    pd.display = false;
    return 0;
}
int func2(int argc, char** argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("BeerPouring", 1000);

    // Sample for XIMEA OpenCV
    xiAPIplusCameraOcv cam;

    // Retrieving a handle to the camera device
    cout << "Opening first camera..." << endl;
    cam.OpenFirst();

    // Set exposure
    cam.SetImageDataFormat(XI_RGB24);
    cam.SetExposureTime(792);
    cam.SetGain(10);
    cout << "Starting acquisition..." << endl;
    cam.StartAcquisition();
    // Note: The default parameters of each camera might be different in different API versions
    cout<<"Please adjust the camera for white balancing, esc to continue"<<endl;
    Mat img_wb;
    while (waitKey(33) != 27){
        img_wb = cam.GetNextImageOcvMat();
        imshow("white balance", img_wb);
    }
    cam.SetXIAPIParamInt(XI_PRM_MANUAL_WB, 1);
    cout<<"OK?"<<endl;
    while (waitKey(33) != 27){
        img_wb = cam.GetNextImageOcvMat();
        imshow("white balance", img_wb);
    }

    ParaDis pd;
    CamRoiXimea my_vc(&cam, &pd);

    boost::thread th1(boost::bind(display_img, &pd));
    Mat img;
    clock_t t1 = clock();
    vector<double> t_vec;
    double liquid_level, beer_ratio;
    while (pd.display){
        my_vc.get_beer_ratio(beer_ratio, liquid_level,true);
        t_vec.emplace_back((clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000);
        t1 = clock();
        std_msgs::Float64MultiArray data;
        data.data.emplace_back(beer_ratio);
        data.data.emplace_back(liquid_level);
        data.data.emplace_back(t_vec.back());
        chatter_pub.publish(data);

    }
    cout<<"ave time "<<accumulate(t_vec.begin(), t_vec.end(), 0.0) / t_vec.size()<<endl;
    cout<<"ave read time "<<accumulate(my_vc.t_read.begin(), my_vc.t_read.end(), 0.0) / my_vc.t_read.size()<<endl;
    cout<<"ave thresh time "<<accumulate(my_vc.t_thresh.begin(), my_vc.t_thresh.end(), 0.0) / my_vc.t_thresh.size()<<endl;
    cout<<"ave seg time "<<accumulate(my_vc.t_seg.begin(), my_vc.t_seg.end(), 0.0) / my_vc.t_seg.size()<<endl;
    return 0;

}


int main(int argc, char **argv){
    return func2(argc, argv);
}
