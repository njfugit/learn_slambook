/*
 * @Author: Jack
 * @Date: 2022-07-19 00:53:10
 * @LastEditTime: 2022-07-19 22:20:49
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/orb_feature_cv.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<chrono>

using namespace std;

int main(int argc, char ** argv){
    string img1_path = "/home/nj/DATA/slambook2/ch7/1.png";
    string img2_path = "/home/nj/DATA/slambook2/ch7/2.png";
    //读取图像
    //cv::Mat img1 = cv::imread(img1_path, CV_LOAD_IMAGE_COLOR);
    //cv::Mat img2 = cv::imread(img2_path, CV_LOAD_IMAGE_COLOR);
    cv::Mat img1 = cv::imread(img1_path, 1);
    cv::Mat img2 = cv::imread(img2_path, 1);
    assert(img1.data != nullptr && img2.data != nullptr);

    //初始化
    vector<cv::KeyPoint> keypoints1, keypoints2;//关键点
    cv::Mat descriptors1, descriptors2;         //描述子
    //特征提取器
    cv::Ptr<cv::FeatureDetector> detetor = cv::ORB::create();
    //描述子提取器
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    //描述子匹配器
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::ORB::create();

    //一、首先检测FAST角点位置
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    detetor->detect(img1, keypoints1);
    detetor->detect(img2, keypoints2);
    
    //二、根据角点计算brief描述子
    descriptor->compute(img1, keypoints1, descriptors1);
    descriptor->compute(img2, keypoints2, descriptors2);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "提取特征点和计算描述子的时间" << time_used.count() << "seconds" << endl;

    //显示提取的特征点
    cv::Mat outimg1;
    cv::drawKeypoints(img1, keypoints1, outimg1);
    cv::imshow("img1 ORB features", outimg1);

    //三、描述子匹配　使用Hamming距离
    vector<cv::DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors1, descriptors2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "匹配描述子的时间" << time_used.count() << "seconds" << endl;
    
    //四、匹配点的筛选　　计算最大距离和最小距离
    auto min_max = minmax_element(matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch &m2)
                                  { return m1.distance < m2.distance; });
    double min_distance = min_max.first->distance;
    double max_distance = min_max.second->distance;

    printf("Min distance : %f \n", min_distance);
    printf("Max distance : %f \n", max_distance);

    //当描述子之间的距离大于两倍的最小距离时，即认为匹配有误，　设置30作为距离下限
    vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptors1.rows; i++){
        if(matches[i].distance <= max(2*min_distance, 30.0))
            good_matches.push_back(matches[i]);
    }
    //五、绘制结果
    cv::Mat img_match;
    cv::Mat img_goodmatch;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_match);
    cv::drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_goodmatch);

    cv::imshow("all matches", img_match);
    cv::imshow("good matches", img_goodmatch);
    cv::waitKey(0);
    

    return 0;
}