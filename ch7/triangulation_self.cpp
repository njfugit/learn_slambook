/*
 * @Author: Jack
 * @Date: 2022-07-23 14:51:22
 * @LastEditTime: 2022-07-24 01:23:45
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
void triangulation(const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches, cv::Mat &R, cv::Mat &t, vector<cv::Point3d>  &points);
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
        cv::circle(img2_copy, keypoints2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
    }
    cv::imshow("img1", img1_copy);
    cv::imshow("img2", img2_copy);
    cv::waitKey(0);

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

    vector<cv::DMatch> match;
    matcher->match(descriptors1, descriptors2, match);

    //匹配点对筛选
    double min_distance = 10000, max_diatance = 0;
    //找出所有匹配之间的最小距离和最大距离
    for (int i = 0; i < descriptors1.rows; ++i){
        double dist = match[i].distance;
        if(dist < min_distance)
            min_distance = dist;
        if(dist > max_diatance)
            max_diatance = dist;

    }
    printf("-- Max dist : %f \n", max_diatance);
    printf("-- Min dist : %f \n", min_distance);
    cout << "min_distance" << min_distance << endl;
    cout << "max_diatance" << max_diatance << endl;

    for (int i = 0; i < descriptors1.rows; ++i){
        if(match[i].distance <= max(2*min_distance, 30)){
            matches.push_back(match[i]);
        }
    }
}

void pose_2d2d(const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches, cv::Mat &R, cv::Mat &t){
    // 相机内参,TUM Freiburg2
  cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  //-- 把匹配点转换为vector<Point2f>的形式
  vector<cv::Point2f> points1;
  vector<cv::Point2f> points2;

  for (int i = 0; i < (int) matches.size(); i++) {
    points1.push_back(kp1[matches[i].queryIdx].pt);
    points2.push_back(kp2[matches[i].trainIdx].pt);
  }

  //-- 计算本质矩阵
  cv::Point2d principal_point(325.1, 249.7);        //相机主点, TUM dataset标定值
  int focal_length = 521;            //相机焦距, TUM dataset标定值
  cv::Mat essential_matrix;
  essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);

  //-- 从本质矩阵中恢复旋转和平移信息.
  cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);

}
void triangulation(const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches, cv::Mat &R, cv::Mat &t, vector<cv::Point3d>  &points){
    cv::Mat T1 = (cv::Mat_<double>(3, 4) << 
                 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<double>(3, 4) << 
                 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0);
}