/*
 * @Author: Jack
 * @Date: 2022-07-23 13:13:46
 * @LastEditTime: 2022-07-24 22:19:37
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/pose_3d2d.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<chrono>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/sparse_optimizer.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/solver.h>
#include<g2o/core/optimization_algorithm.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<Eigen/Core>
#include<sophus/se3.hpp>


using namespace std;

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches);

//像素坐标转成相机归一化坐标
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);
//BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Vec2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vec3d;
void BAG2O(const Vec3d &points_3d, const Vec2d &points_2d, const cv::Mat &K, Sophus::SE3d &poses);
//BA by Gauss-Newton
void BAGaussNewton(const Vec3d &points_3d, const Vec2d &points_2d, const cv::Mat &K, Sophus::SE3d &poses);

int main(int argc, char** argv){
    
    string img_path1 = "/home/nijun/Documents/learn_slambook/ch7/1.png"; 
    string img_path2 = "/home/nijun/Documents/learn_slambook/ch7/2.png";
    string depth_path1 = "/home/nijun/Documents/learn_slambook/ch7/1_depth.png";
    string depth_path2 = "/home/nijun/Documents/learn_slambook/ch7/2_depth.png";
    cv::Mat img1 = cv::imread(img_path1, 1);
    cv::Mat img2 = cv::imread(img_path2, 1);
    cv::Mat depth1 = cv::imread(depth_path1, -1);
    cv::Mat depth2 = cv::imread(depth_path2, -1);
    assert(img1.data != nullptr && img2.data!=nullptr);

    vector<cv::KeyPoint> kp1, kp2;
    vector<cv::DMatch> matches;
    feature_match(img1, img2, kp1, kp2, matches);

    //建立3D点
    cv::Mat K = (cv::Mat_<double>(3,3)<<  520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<cv::Point3f> pts_3d;
    vector<cv::Point2f> pts_2d;
    for(cv::DMatch &m : matches){
        ushort d = depth1.ptr<unsigned short>(int(kp1[m.queryIdx].pt.y))[int(kp1[m.queryIdx].pt.x)];
        if(d == 0)
            continue;
        float dd = d / 5000.0;//根据数据集的说明
        //计算归一化坐标
        cv::Point2d p1 = pixel2cam(kp1[m.queryIdx].pt, K);
        //计算空间上的3d坐标
        pts_3d.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
        //另一张图上的匹配的2d坐标
        pts_2d.push_back(kp2[m.trainIdx].pt);
    }
    cout << "3d-2d pairs: " << pts_3d.size() << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    cv::Mat r, t;//r为旋转向量
    cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false); //第四个是相机畸变参数
    //将旋转向量转换成旋转矩阵
    cv::Mat R;
    cv::Rodrigues(r, R);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;
    
    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;

    Vec3d pts_3d_eigen;
    Vec2d pts_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); ++i){
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }

    cout << "calling bundle adjustment by gauss newton" << endl;

    Sophus::SE3d pose_gn;
    t1 = chrono::steady_clock::now();
    BAGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << endl;

    cout << "calling bundle adjustment by g2o" << endl;
    Sophus::SE3d pose_g2o;
    t1 = chrono::steady_clock::now();
    BAG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;

    return 0;
}

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches){
    cv::Ptr<cv::FeatureDetector> detetor = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    detetor->detect(img1, keypoints1);
    detetor->detect(img2, keypoints2);

    cv::Mat descriptors1, descriptors2;
    descriptor->compute(img1, keypoints1, descriptors1);
    descriptor->compute(img2, keypoints2, descriptors2);
}