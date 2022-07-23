/*
 * @Author: Jack
 * @Date: 2022-07-23 13:13:46
 * @LastEditTime: 2022-07-23 14:47:16
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

    return 0;
}