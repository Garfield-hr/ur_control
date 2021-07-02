//
// Created by hairui on 6/30/21.
//

#include "CamROI.h"
#include <numeric>
#include <time.h>


CamROI::CamROI() {

}

bool CamROI::read_img(Mat& img) {
    return false;
}

void CamROI::display_video() {
    Mat img;
    clock_t t1 = clock();
    vector<double> t_vec;
    double liquid_level, beer_ratio;
    while (get_beer_ratio(beer_ratio, liquid_level,true)){
        t_vec.emplace_back((clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000);
        t1 = clock();
    }
    cout<<"ave time "<<accumulate(t_vec.begin(), t_vec.end(), 0.0) / t_vec.size()<<endl;
    cout<<"ave read time "<<accumulate(t_read.begin(), t_read.end(), 0.0) / t_read.size()<<endl;
    cout<<"ave thresh time "<<accumulate(t_thresh.begin(), t_thresh.end(), 0.0) / t_thresh.size()<<endl;
    cout<<"ave seg time "<<accumulate(t_seg.begin(), t_seg.end(), 0.0) / t_seg.size()<<endl;

}

bool CamROI::set_marker_position() {
    Mat img;
    if(!read_img(img)) {
        cout << "can't read img" << endl;
        return false;
    }

    imshow(markers_win_name, img);
    setMouseCallback(markers_win_name, OnMouseAction, this);
    while (markers.size() < 4){
        waitKey(1000);
    }
    destroyAllWindows();
    return true;
}

void OnMouseAction(int event, int x, int y, int flags, void* cam_roi) {

    if (event == CV_EVENT_LBUTTONDOWN){
        auto* cam_roi_pointer = reinterpret_cast<CamROI*>(cam_roi);
        cam_roi_pointer->times++;
        cout<<"selected "<<cam_roi_pointer->times<<"points"<<endl;
        cam_roi_pointer->markers.emplace_back(x,y);
        Mat img;
        cam_roi_pointer->read_img(img);
        for(auto & marker : cam_roi_pointer->markers){
            circle(img, marker, 1, Scalar(0,0, 255), -1);
            string text_xy = to_string(marker.x) + "," + to_string(marker.y);
            putText(img, text_xy, marker, CV_FONT_HERSHEY_COMPLEX, 1.0,
                    Scalar(0,0, 255), 2, 8, 0);
            imshow("Please select four markers in order", img);
        }
    }

}

bool CamROI::read_img_roi(Mat &img_roi, Point2i& offset) {
    clock_t t1 = clock();
    Mat img_full;
    if (!read_img(img_full)){
        return false;
    }
    clock_t t2 = clock();
    t_read.emplace_back((t2 - t1) * 1.0 / CLOCKS_PER_SEC * 1000);
    for(int i=0; i<markers.size(); i++){
        int x = markers[i].x;
        int y = markers[i].y;
        Mat roi, roi_bin;
        img_full(Range(max(0, y - roi_size), min(img_full.rows, y + roi_size)),
            Range(max(0, x - roi_size), min(img_full.cols, x + roi_size))).copyTo(roi);
        if (roi.channels() == 3){
            cvtColor(roi, roi, COLOR_BGR2GRAY);
        }
        threshold(roi, roi_bin, 220, 255, THRESH_BINARY);
        Moments mom = moments(roi_bin);
        if (mom.m00 > 20){
            x = mom.m10 / mom.m00 + x - roi_size;
            y = mom.m01 / mom.m00 + y - roi_size;
        }
        markers[i].x = x;
        markers[i].y = y;
    }
    clock_t t3 = clock();
    t_thresh.emplace_back((t3 - t2) * 1.0 / CLOCKS_PER_SEC * 1000);
    int x1, x2, y1, y2;
    vector<int> x_arr, y_arr;
    for(auto & marker : markers){
        x_arr.emplace_back(marker.x);
        y_arr.emplace_back(marker.y);
    }
    x1 = *min_element(x_arr.begin(), x_arr.end());
    x2 = *max_element(x_arr.begin(), x_arr.end());
    y1 = *min_element(y_arr.begin(), y_arr.end());
    y2 = *max_element(y_arr.begin(), y_arr.end());
    offset.x = x1;
    offset.y = y1;
    img_full(Range(y1, y2), Range(x1, x2)).copyTo(img_roi);
    clock_t t4 = clock();
    t_seg.emplace_back((t4 - t3) * 1.0 / CLOCKS_PER_SEC * 1000);
    return true;
}

bool CamROI::if_pouring_started(Mat &img_bin) {
    int min_length = (markers[2].x - markers[1].x) * 6/10;
    Mat labels, stats, centroids;
    int num_comp = connectedComponentsWithStats(img_bin, labels, stats, centroids);
    bool this_img_started = false;
    for(int i = 1; i < num_comp; i++){
        if (stats.at<int>(i, CC_STAT_WIDTH) > min_length)
            this_img_started = true;
    }
    return this_img_started;
}

bool CamROI::get_beer_ratio(double& ratio, double& liquid_level, bool debug) {
    Mat img_roi, img_gray;
    int filter_size = 9;
    Point2i offset;
    bool ret = read_img_roi(img_roi, offset);

    if(!ret) return false;
    if (img_roi.channels() == 3){
        cvtColor(img_roi, img_gray, COLOR_BGR2GRAY);
    }
    else{
        img_gray = img_roi;
    }
    int bin_thresh1 = 100, bin_thresh2 = 120;
    if(!pouring_started){
        Mat img_bottom = img_gray(Range(img_gray.rows/2, img_gray.rows), Range(0, img_gray.cols));
        Mat img_bottom_bin;
        threshold(img_bottom, img_bottom_bin, bin_thresh1, 255, THRESH_BINARY);
        pouring_started = if_pouring_started(img_bottom_bin);
        if (debug){
            parallel_dis_img(img_bottom);
        }
        return true;
    }
    Mat img_bin;
    int upper_line, lower_line;
    threshold(img_gray, img_bin, bin_thresh2, 255, THRESH_BINARY);
    image_seg_two_lines(img_bin, upper_line, lower_line);

    // mid filter
    if (upper_line_vec.size() == filter_size){
        upper_line_vec.erase(upper_line_vec.begin());
        upper_line_vec.emplace_back(upper_line);
        upper_line = get_median(upper_line_vec);
        lower_line_vec.erase(lower_line_vec.begin());
        lower_line_vec.emplace_back(lower_line);
        lower_line = get_median(lower_line_vec);
    }
    else{
        if(upper_line_vec.size() < filter_size){
            upper_line_vec.emplace_back(upper_line);
            lower_line_vec.emplace_back(lower_line);
            return true;
        }
        else{
            cout<<"out of filter range"<<endl;
            return false;
        }
    }
    vector<Point2i> normalize_markers;
    for(const auto& marker : markers){
        Point2i normalize_marker;
        normalize_marker.x = marker.x - offset.x;
        normalize_marker.y = marker.y - offset.y;
        normalize_markers.emplace_back(normalize_marker);
    }
    Point2i upper_p, lower_left_p, lower_right_p;
    upper_p.y = upper_line;
    lower_left_p.y = lower_line;
    lower_right_p.y = lower_line;
    upper_p.x = get_x_by_two_points_and_y(upper_p.y, normalize_markers[0], normalize_markers[1]);
    lower_left_p.x = get_x_by_two_points_and_y(lower_left_p.y, normalize_markers[0], normalize_markers[1]);
    lower_right_p.x = get_x_by_two_points_and_y(lower_right_p.y, normalize_markers[2], normalize_markers[3]);
    double h1, h2, h3;
    h1 = norm(upper_p - lower_left_p);
    h2 = norm(lower_left_p - normalize_markers[1]);
    h3 = norm(lower_right_p - normalize_markers[2]);
    ratio = (h2 + h3) / (2*h1 + h2 +h3);
    liquid_level = (normalize_markers[2].y- upper_line) / (normalize_markers[2].y - normalize_markers[3].y);
    if(debug){
        line(img_roi, Point(0, upper_line), Point(img_roi.cols, upper_line), Scalar(255, 0, 0), 1, CV_AA);
        line(img_roi, Point(0, lower_line), Point(img_roi.cols, lower_line), Scalar(0, 0, 255), 1, CV_AA);
        circle(img_roi,upper_p, 3, Scalar(255,0 ,0));
        circle(img_roi,lower_left_p, 3, Scalar(0,255 ,0));
        circle(img_roi,lower_right_p, 3, Scalar(0,0 ,255));
        parallel_dis_img(img_roi);
    }

    return true;
}

void CamROI::parallel_dis_img(Mat &img) {
    imshow("img", img);
    waitKey(1);
}

void image_seg_two_lines(Mat& img, int& upper_line, int& lower_line){
    if(img.channels() == 3){
        cvtColor(img, img, COLOR_BGR2GRAY);
    }
    Mat img_row_summed;
    reduce(img, img_row_summed, 1, CV_REDUCE_SUM, CV_32S);
    vector<int> array;
    vector<int> diff;
    diff.resize(img_row_summed.rows);
    adjacent_difference(img_row_summed.begin<int>(), img_row_summed.end<int>(), diff.begin());
    upper_line = max_element(diff.begin(), diff.end()) - diff.begin();
    lower_line = min_element(diff.begin(), diff.end()) - diff.begin();
}

int get_median(vector<int> arr){
    int num = arr.size();
    nth_element(arr.begin(), arr.begin() + num/2, arr.end());
    return arr[num/2];
}

int get_x_by_two_points_and_y(int y, Point p1, Point p2){
    return p2.x + (y - p2.y)*(p2.x - p1.x)/(p2.y - p1.y);
}
