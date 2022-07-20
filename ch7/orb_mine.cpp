/*
 * @Author: Jack
 * @Date: 2022-07-20 01:14:18
 * @LastEditTime: 2022-07-20 21:42:22
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/orb_mine.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<string>
#include<chrono>
#include<opencv2/opencv.hpp>
#include<nmmintrin.h>//和内存有关

using namespace std;

string img_path1 = "/home/nijun/Documents/learn_slambook/ch7/1.png";
string img_path2 = "/home/nijun/Documents/learn_slambook/ch7/2.png";

typedef vector<uint32_t> DescriptorType;

//计算orb特征点的描述子
/**
 * NOTE: if a keypoint goes outside the image boundary (8 pixels), descriptors will not be computed and will be left as
 * empty
 * 
 */
void ComputeORB(const cv::Mat &img, vector<cv::KeyPoint> &keypoints, vector<DescriptorType> &descriptors);

//两组描述子暴力匹配
int main(int argc, char **argv){

    cv::Mat img1 = cv::imread(img_path1, 0);
    cv::Mat img2 = cv::imread(img_path2, 0);

    assert(img1.data != nullptr && img2.data != nullptr);

    //检测fast关键点
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    vector<cv::KeyPoint> keypoints;
    
    return 0;
}