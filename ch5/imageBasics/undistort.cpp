/*
 * @Author: Jack
 * @Date: 2022-05-04 23:29:41
 * @LastEditTime: 2022-06-09 00:50:12
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch5/imageBasics/undistort.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<string>

using namespace std;
using namespace cv;

string distorted_image = "/home/nj/DATA/slambook2/ch5/imageBasics/distorted.png";
int main(int argc, char **argv)
{
    //去畸变代码
     // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    Mat image = imread(distorted_image, 0);//灰度图，单通道
    int rows = image.rows, cols = image.cols;
    Mat undistort_image = Mat(rows, cols, CV_8UC1);
    //计算去畸变后的图像内容
    for (int v = 0; v < rows; v++){
        for (int u = 0; u < cols; u++){
            //计算点(u,v)对应畸变图像中的坐标(u_distorted, v_distorted)
            double x = (u - cx) / fx, y = (v - cy) / fy;//畸变图像坐标(u,v)先作用到畸变的归一化平面坐标(x,y坐标表示)
            double r = sqrt(x * x + y * y);
            //计算归一化平面上的点的作用畸变系数来纠正，去畸变 
            double x_undistort = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_undistort = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            double u_undistort = fx * x_undistort + cx;
            double v_undistrot = fy * y_undistort + cy;

            //赋值
            if(u_undistort >= 0 && v_undistrot >= 0 && u_undistort < cols && v_undistrot < rows){
                undistort_image.at<uchar>(v, u) = image.at<uchar>((int)v_undistrot, (int)u_undistort);
            }
            else{
                undistort_image.at<uchar>(v, u) = 0;
            }
        }
    }
    //画出畸变图像
    imshow("distorted_image", image);
    //画出去畸变图像（作用畸变系数后)
    imshow("undistort_image", undistort_image);
    waitKey();
    return 0;
}