/*
 * @Author: Jack
 * @Date: 2022-07-19 00:53:10
 * @LastEditTime: 2022-07-19 01:09:45
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/orb_feature_cv.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<chrono>

using namespace std;

int main(int argc, char ** argv){
    string img1_path = "/home/nj/DATA/slambook2/ch7/1.png";
    string img2_path = "/home/nj/DATA/slambook2/ch7/2.png";
    //读取图像
    cv::Mat img1 = cv::imread(img1_path, CV_LOAD_IMAGE_COLOR);
    cv::Mat img2 = cv::imread(img2_path, CV_LOAD_IMAGE_COLOR);
    assert(img1.data != nullptr && img2.data != nullptr);

    //初始化
    vector<cv::KeyPoint> keypoints1, keypoints2;//关键点
    cv::Mat descriptors1, descriptors2;         //描述子
    //特征提取器
    cv::Ptr<cv::FeatureDetector> detetor = cv::ORB::create();
    //描述子提取器
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    //描述子匹配器
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::ORB::create();

    //首先检测FAST角点位置
    



    return 0;
}