/*
 * @Author: Jack
 * @Date: 2022-05-05 23:08:57
 * @LastEditTime: 2022-06-09 01:16:19
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch5/stereo/stereo.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Core>
#include<string>
#include<vector>
#include<pangolin/pangolin.h>
#include <unistd.h>
using namespace std;
using namespace Eigen;
using namespace cv;

string left_image = "/home/nj/DATA/slambook2/ch5/stereo/left.png";
string right_image = "/home/nj/DATA/slambook2/ch5/stereo/right.png";

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);


int main(int argc, char **argv){
    //内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    //基线
    double baseline = 0.573;

    //读取图像
    Mat left = imread(left_image, 0);
    Mat right = imread(right_image, 0);
    //调用SGBM来计算视差
    //https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html#details
    //指向OPenCV中的sgbm（半全局立体匹配算法），其中最小视差为0，最大视差为96
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
    Mat disparity_sgdm, disparity;
    sgbm->compute(left, right, disparity_sgdm);
    disparity_sgdm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    //只利用了左图
    for (int v = 0; v < left.rows; v++){
        for (int u = 0; u < left.cols; u++){
            if(disparity.at<float>(v,u) <= 0.0 || disparity.at<float>(v,u) >= 96.0)continue;
            //根据双目模型计算point位置         
            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); //前三位表示xyz,第四位表示颜色（灰度值）
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * baseline / (disparity.at<float>(v, u));
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }
    }
    imshow("disparity", disparity / 96.0);
    waitKey(0);
    showPointCloud(pointcloud);
    return 0;
}
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud){

    if(pointcloud.empty()){
        cerr << "empty" << endl;
        return;
    }
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));
    while(pangolin::ShouldQuit() ==false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glPointSize(2);
        glBegin(GL_POINTS);
        for(auto &p: pointcloud){
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);//5ms
    }
    return;
}