/*
 * @Author: Jack
 * @Date: 2022-07-23 13:14:24
 * @LastEditTime: 2022-07-26 21:53:38
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
void BAG2O(const vector<cv::Point3d> &p_3d1, const vector<cv::Point3d> &p_3d2, cv::Mat &R, cv::Mat &t);

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
        cv::Point2d p1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
        cv::Point2d p2 = pixel2cam(keypoints2[m.trainIdx].pt, K);
        float dd1 = float(d1) / 5000.0;
        float dd2 = float(d2) / 5000.0;

        pts1.push_back(cv::Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(cv::Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    cout << "3d-3d 匹配点对" << pts1.size() << endl;

    cv::Mat R, t;
    pose_3d3d(pts1, pts2, R, t);
    cout << "ICP通过SVD分解结果" << endl;
    cout << "R = " << R << endl;
    cout << "t" << t << endl;
    cout << "R_inv" << R.t() << endl;
    cout << "t_inv" << -R.t() * t << endl;

    cout << "BA by G2O" << endl;
    BAG2O(pts1, pts2, R, t);
    // p1= R * p2 + t;　p2是相机坐标系，p1是世界坐标系
    for (int i = 0; i < 5; ++i){
        cout << "p1=" << pts1[i] << endl;
        cout << "p2=" << pts2[i] << endl;
        cout << "R*p2+t = " << R * (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t << endl;
        cout << endl;
    }

    return 0;
}
void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches){
    cv::Ptr<cv::FeatureDetector> detetor = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    detetor->detect(img1, kp1);
    detetor->detect(img2, kp2);

    cv::Mat descriptors1, descriptors2;
    descriptor->compute(img1, kp1, descriptors1);
    descriptor->compute(img2, kp2, descriptors2);

    vector<cv::DMatch> match;
    matcher->match(descriptors1, descriptors2, match);

    double min_distance = 10000, max_distance = 0;

    for (int i = 0; i < descriptors1.rows; i++){
        double dist = match[i].distance;
        if(dist<min_distance)
            min_distance = dist;
        if(dist>max_distance)
            max_distance = dist;
        
    }
    cout << "min_distance" << min_distance << endl;
    cout << "max_distance" << max_distance << endl;

    for (int i = 0; i < descriptors1.rows;++i){
        if(match[i].distance<=max(2*min_distance,30.0)){
            matches.push_back(match[i]);
        }
    }
}