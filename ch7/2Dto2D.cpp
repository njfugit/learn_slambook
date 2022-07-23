/*
 * @Author: Jack
 * @Date: 2022-07-22 15:34:32
 * @LastEditTime: 2022-07-22 22:25:15
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/2Dto2D.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches);
void pose_eatimation(vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches, cv::Mat &R, cv::Mat &t);
//将像素坐标转化为相机归一化坐标
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

int main(int argc, char **argv)
{
    cv::Mat img1 = cv::imread("./1.png", 1);
    cv::Mat img2 = cv::imread("./2.png", 1);
    assert(img1.data != nullptr && img2.data != nullptr);

    vector<cv::KeyPoint> kp1, kp2;
    vector<cv::DMatch> matches;
    feature_match(img1, img2, kp1, kp2, matches);
    cout << "找到" << matches.size() << " 组匹配点" << endl;

    //估计两张图像的相对运动
    cv::Mat R, t;
    pose_eatimation(kp1, kp2, matches, R, t);
    //验证本质矩阵　E=t^R*s s表示尺度因子
    //反对称矩阵
    cv::Mat_<double> t_x = (cv::Mat_<double>(3, 3) << 
                            0, -t.at<double>(2, 0), t.at<double>(1, 0),
                            t.at<double>(2,0), 0, -t.at<double>(0, 0),
                            -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    cout << "t^R=" << endl
         << t_x * R << endl;

    //验证对极约束
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for(cv::DMatch &m : matches){
        cv::Point2d p1 = pixel2cam(kp1[m.queryIdx].pt, K);//关键点投影到归一化平面
        cv::Vec3d d1(p1.x, p1.y, 1);    //转化为齐次坐标
        cv::Mat y1 = (cv::Mat_<double>(3, 1) << p1.x, p1.y, 1);

        cv::Point2d p2 = pixel2cam(kp2[m.trainIdx].pt, K);
        cv::Vec3d d2(p2.x, p2.y, 1);
        cv::Mat y2 = (cv::Mat_<double>(3, 1) << p2.x, p2.y, 1);

        cv::Mat d = d2.t() * t_x * R * d1;
        cout << "对极约束:" << d << endl;
    }

    return 0;
}
void feature_match(const cv::Mat &img1, 
                   const cv::Mat &img2, 
                   vector<cv::KeyPoint> &keypoints1, 
                   vector<cv::KeyPoint> &keypoints2, 
                   vector<cv::DMatch> &matches){
    //初始化
    cv::Mat descriptors1, descriptors2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    //检测
    detector->detect(img1, keypoints1);
    detector->detect(img2, keypoints2);
    //计算描述子
    descriptor->compute(img1, keypoints1, descriptors1);
    descriptor->compute(img2, keypoints2, descriptors2);
    //描述子匹配
    vector<cv::DMatch> match;
    matcher->match(descriptors1, descriptors2, match);
    //匹配点对筛选
    double min_distance = 10000, max_diatance = 0;
    //找出所有匹配之间的最小距离和最大距离
    for (int i = 0; i < descriptors1.rows; ++i){
        double d = match[i].distance;
        if(d < min_distance)
            min_distance = d;
        if (d > max_diatance)
            max_diatance = d;
        
    }
    printf("max_distance : %f \n", max_diatance);
    printf("min_distance : %f \n", min_distance);


    //当描述子之间的距离大于两倍的最小距离时，认为匹配有误　下限阈值为30
    for (int i = 0; i < descriptors1.rows; ++i){
        if(match[i].distance <= max(2*min_distance, 30.0)){
            matches.push_back(match[i]);
        }
    }
}

void pose_eatimation(vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches, cv::Mat &R, cv::Mat &t){
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<cv::Point2f> points1; //分别存放匹配的点对的两个点
    vector<cv::Point2f> points2; 

    for (int i = 0; i < matches.size(); ++i){
        points1.push_back(keypoints1[matches[i].queryIdx].pt);
        points1.push_back(keypoints2[matches[i].trainIdx].pt);
    }
    //计算基础矩阵
    cv::Mat f_matrix;
    f_matrix = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
    cout << "fundamental_matrix is " << endl << f_matrix << endl;

    //计算本质矩阵
    cv::Point2d principal_point(325.1, 249.7);//相机光心
    double focal_length = 521;//焦距
    cv::Mat e_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << e_matrix << endl;
    
    //计算单应性矩阵
    cv::Mat h_matrix = cv::findHomography(points1, points2, cv::RANSAC, 3);
    cout << "homography_matrix is " << endl << h_matrix << endl;

    //从本质矩阵恢复Ｒ，t
    cv::recoverPose(e_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K){
    double px = (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0);
    double py = (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1);
    return cv::Point2d(px, py);
}