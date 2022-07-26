/*
 * @Author: Jack
 * @Date: 2022-07-23 13:14:24
 * @LastEditTime: 2022-07-26 20:20:43
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/pose_3d3d.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<Eigen/SVD>

#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<sophus/se3.hpp>
#include<chrono>

using namespace std;

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches);
void pose_3d3d(const vector<cv::Point3f> &pts1, const vector<cv::Point3f> &pts2, cv::Mat &R, cv::Mat &t);
cv::Point2d pixel2cam(const vector<cv::Point2d> &p, cv::Mat &K);
void BAG2O(const vector<cv::Point3d> &p_3d, const vector<cv::Point2d> &p_2d, cv::Mat &R, cv::Mat &t);

int main(int argc, char ** argv){
    string img_path1 = "/home/nj/DATA/learn_slambook/ch7/1.png"; 
    string img_path2 = "/home/nj/DATA/learn_slambook/ch7/2.png";
    cv::Mat img1 = cv::imread(img_path1, 1);
    cv::Mat img2 = cv::imread(img_path2, 1);
    string depth_path1 = "/home/nj/DATA/learn_slambook/ch7/1_depth.png";
    string depth_path2 = "/home/nj/DATA/learn_slambook/ch7/2_depth.png";
    cv::Mat depth1 = cv::imread(depth_path1, -1);
    cv::Mat depth2 = cv::imread(depth_path2, -1);
    vector<cv::KeyPoint> keypoints1, keypoints2;
    vector<cv::DMatch> matches;
    feature_match(img1, img2, keypoints1, keypoints2, matches);

    cout << "找到的匹配数量" << matches.size() << endl;

    //建立3D点
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<cv::Point3f> pts1, pts2;
    for(cv::DMatch &m : matches){
        ushort d1 = depth1.ptr<ushort>(keypoints1[m.queryIdx].pt.y)[keypoints1[m.queryIdx].pt.x];
        ushort d2 = depth2.ptr<ushort>(keypoints2[m.trainIdx].pt.y)[keypoints2[m.trainIdx].pt.x];
        if(d1 == 0 || d2 == 0)
            continue;
        
    }

    return 0;
}