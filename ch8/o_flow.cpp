/*
 * @Author: Jack
 * @Date: 2022-07-27 23:45:36
 * @LastEditTime: 2022-07-28 01:17:50
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
            
    private:
        const cv::Mat &img1;
        const cv::Mat &img2;
        const vector<cv::KeyPoint> &kp1;
        vector<cv::KeyPoint> &kp2;
        vector<bool> &success;
        bool inverse = true;
        bool has_initial = false;
};

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
    if(y >= img.rows-1)
        y = img.rows - 2;
}