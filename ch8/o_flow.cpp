/*
 * @Author: Jack
 * @Date: 2022-07-27 23:45:36
 * @LastEditTime: 2022-07-28 21:51:31
 * @LastEditors: your name
 * @FilePath: /ch8/o_flow.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<opencv2/opencv.hpp>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<chrono>
#include<string>

using namespace std;

string LK_path1 = "/home/nj/DATA/learn_slambook/ch8/LK1.png";
string LK_path2 = "/home/nj/DATA/learn_slambook/ch8/LK2.png";
class OpticalFlowTracker
{
    public:
        OpticalFlowTracker(
            const cv::Mat &img1_, 
            const cv::Mat &img2_,
            const vector<cv::KeyPoint> &kp1_,
            vector<cv::KeyPoint> &kp2_,
            vector<bool> &success_,
            bool inverse = true,
            bool has_initial = false):
            img1(img1_), img2(img2_), kp1(kp1_),kp2(kp2_),success(success_){}
        void calculateOpticalFlow(const cv::Range &range);
            
    private:
        const cv::Mat &img1;
        const cv::Mat &img2;
        const vector<cv::KeyPoint> &kp1;
        vector<cv::KeyPoint> &kp2;
        vector<bool> &success;
        bool inverse = true;
        bool has_initial = false;
};
/**
 * @brief 
 * 
 * @param img1 
 * @param img2 
 * @param kp1 
 * @param kp2 
 * @param success  success true if a keypoint is tracked successfully
 * @param inverse  inverse set true to enable inverse formulation
 * @param has_initial 
 */
void OpticalFlowSingle(const cv::Mat &img1,
                       const cv::Mat &img2,
                       const vector<cv::KeyPoint> &kp1,
                       vector<cv::KeyPoint> &kp2,
                       vector<bool> &success,
                       bool inverse = false, 
                       bool has_initial = false);

void OpticalFlowMulti(const cv::Mat &img1,
                      const cv::Mat &img2,
                      const vector<cv::KeyPoint> &kp1,
                      vector<cv::KeyPoint> &kp2,
                      vector<bool> &success,
                      bool inverse = false);
//获取像素点的线性插值
inline float GetPixelValue(const cv::Mat &img, float x, float y){
    if(x < 0) x = 0;
    if(y < 0) y = 0;
    if(x >= img.cols-1)
        x = img.cols - 2;
    if (y >= img.rows - 1)
        y = img.rows - 2;
    float xx = x - floor(x); //floor向下取整函数　xx表示小数部分
    float yy = y - floor(y);
    int x_a1 = min(img.cols - 1, int(x) + 1);
    int y_a1 = min(img.cols - 1, int(y) + 1);

     //双线性插值　（x, y）(x+1, y) (x, y+1) (x+1, y+1)
    float x = floor(x);
    float y = floor(y);

    return (1 - xx) * (1 - yy) * img.at<uchar>(y, x) 
    + xx * (1 - yy) * img.at<uchar>(y, x_a1) 
    + (1 - xx) * yy * img.at<uchar>(y_a1, x) 
    + xx * yy * img.at<uchar>(y_a1, x_a1);
}

int main(int argc, char **argv){
    cv::Mat img1 = cv::imread(LK_path1, cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(LK_path2, cv::IMREAD_GRAYSCALE);

    vector<cv::KeyPoint> kp1;
    //使用harris角点作为LK光流的跟踪输入点　角点定义为在x,y方向上均有较大梯度变化的小区域
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20);
    detector->detect(img1, kp1);
    //跟踪上第二张图上这些点　在验证图中使用单层LK
    vector<cv::KeyPoint> kp2_single;
    vector<bool> success_single;
    OpticalFlowSingle(img1, img2, kp1, kp2_single, success_single);

    //多层LK
    vector<cv::KeyPoint> kp2_multi;
    vector<bool> success_multi;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    OpticalFlowMulti(img1, img2, kp1, kp2_multi, success_multi, true);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "optical flow by gauss-newton" << time_used.count() << "seconds" << endl;
    //利用opencv光流验证
    vector<cv::Point2f> pt1, pt2;
    for(auto &kp : kp1)
        pt1.push_back(kp.pt);

    vector<uchar> status;
    vector<float> error;  
    t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by opencv" << time_used.count() << "seconds" << endl;

    //画出单层、多层以及opencv实现的区别
    cv::Mat img2_single;
    cv::cvtColor(img2, img2_single,cv::COLOR_GRAY2BGR);
    for (int i = 0; i < kp2_single.size(); i++){
        if(success_single[i]){
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0), 2);
        }
    }

    cv::Mat img2_multi;
    cv::cvtColor(img2, img2_multi,cv::COLOR_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++){
        if(success_multi[i]){
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0), 2);
        }
    }

    cv::Mat img2_cv;
    cv::cvtColor(img2, img2_cv,cv::COLOR_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++){
        if(success_multi[i]){
            cv::circle(img2_multi, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, pt1[i], pt2[i], cv::Scalar(0, 250, 0), 2);
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_cv);
    cv::waitKey(0);
    return 0;
}

void OpticalFlowSingle(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const vector<cv::KeyPoint> &kp1,
    vector<cv::KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false, 
    bool has_initial = false){
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
    //并行调用pticalFlowTracker::calculateOpticalFlow 
    cv::parallel_for_(cv::Range(0, kp1.size()), std::bind(&OpticalFlowTracker::calculateOpticalFlow, tracker, placeholders::_1));
}

void OpticalFlowMulti(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const vector<cv::KeyPoint> &kp1,
    vector<cv::KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false);