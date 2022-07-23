/*
 * @Author: Jack
 * @Date: 2022-07-23 14:51:22
 * @LastEditTime: 2022-07-23 20:24:44
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/triangulation_self.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches);
void pose_2d2d(const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches, cv::Mat &R, cv::Mat &t);
void triangulation(const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches, cv::Mat &R, cv::Mat &t, vector, cv::Point3d > &points);
cv::Point2f pixel2cam(const cv::Point2d &p, cv::Mat &K);

//作图取色
inline cv::Scalar get_color(float depth){
    float up = 50, low = 10;
    float range = up - low; 
    if(depth > up)
        depth = up;
    if(depth < low)
        depth = low;
    return cv::Scalar(255 * depth / range, 0, 255 * (1 - depth / range));
}

int main(int argc, char **argv){
    cv::Mat img1 = cv::imread("./1.png", cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread("./2.png", cv::IMREAD_COLOR);
    vector<cv::KeyPoint> keypoints1, keypoints2;
    vector<cv::DMatch> matches;
    feature_match(img1, img2, keypoints1, keypoints2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    //估计两张图像运动
    cv::Mat R, t;
    pose_2d2d(keypoints1, keypoints2, matches, R, t);

    //三角化
    vector<cv::Point3d> points_3d;
    triangulation(keypoints1, keypoints2, matches, R, t, points_3d);

    //验证三角化的点和特征点重投影关系
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    cv::Mat img1_copy = img1.clone();
    cv::Mat img2_copy = img2.clone();
    for (int i = 0; i < matches.size(); ++i){
        //第一张图
        float depth1 = points_3d[i].z;
        cout << "depth = " << depth1 << endl;
        cv::Point2d pt1_cam = pixel2cam(keypoints1[matches[i].queryIdx].pt, K);
        cv::circle(img1_copy, keypoints1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

        //第二张图
        cv::Mat p_trans = R * (cv::Mat_<double>(3, 1) << points_3d[i].x, points_3d[i].y, points_3d[i].z) + t;
        float depth2 = p_trans.at<double>(2, 0);
        cv::circle(img2_copy, keypoints2[matches[i].trainIdx].pt, 2, get_color(depth2), 2)

    }
    cv::imshow("img1", img1_copy);
    cv::imshow("img2", img2_copy);
    cv::waitKey(0);

    return 0;
}

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches){
    
}