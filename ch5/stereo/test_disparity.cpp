/*
 * @Author: Jack
 * @Date: 2022-05-10 23:18:57
 * @LastEditTime: 2022-05-11 13:03:10
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch5/stereo/test_disparity.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Core>
#include<string>
#include<vector>

#include <unistd.h>
using namespace std;
using namespace Eigen;
using namespace cv;

string left_image = "/home/nj/Documents/66_Infrared.png";
string right_image = "/home/nj/Documents/55_Infrared.png";


int main(int argc, char **argv){
    //内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    //基线
    double baseline = 0.573;

    //读取图像
    Mat left = imread(left_image, 0);
    cout << left.channels() << endl;
    Mat right = imread(right_image, 0);
    //调用SGBM来计算视差
    //https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html#details
    //指向OPenCV中的sgbm（半全局立体匹配算法），其中最小视差为0，最大视差为96
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
    Mat disparity_sgdm, disparity;
    sgbm->compute(left, right, disparity_sgdm);
    disparity_sgdm.convertTo(disparity, CV_32F, 1.0 / 16.0f);
    imshow("disparity", disparity / 96.0);
    imwrite("/home/nj/Documents/disparity4.png", disparity);
    waitKey(0);
    return 0;
}