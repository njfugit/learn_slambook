/*
 * @Author: Jack
 * @Date: 2022-05-07 22:58:03
 * @LastEditTime: 2022-05-09 23:12:38
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch5/rgbd/rgbdMap.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
#include<pangolin/pangolin.h>
#include<sophus/se3.hpp>
#include<string>
#include<boost/format.hpp>

using namespace std;
using namespace cv;

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

string trajectory_file = "/home/nj/DATA/slambook2/ch5/rgbd/pose.txt";

int main(int argc,char **argv){
    vector<Mat> color_images, depth_images;
    TrajectoryType poses;//相机位姿

    ifstream fin(trajectory_file);
    if(!fin){
        cerr << "cannot find trajectory file at" << trajectory_file << endl;
        return 1;
    }
    for (int i = 0; i < 5; i++){
        boost::format fmt("/home/nj/DATA/slambook2/ch5/rgbd/%s/%d.%s");
        color_images.push_back(imread((fmt %"color" %(i+1) %"png").str()));
        depth_images.push_back(imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));//标志位-1：即以不改变图片的方式打开，图片是彩色就是彩色，图片是灰度图像就是灰度图像

        double data[7] = {0};
        for(auto &d : data){
            fin >> d;//把trajectory_file文件内容依次输出到data中
        }
        //定义李群
        Sophus::SE3d pose(Eigen::Quaterniond(data[6],data[3],data[4],data[5]),
                          Eigen::Vector3d(data[0],data[1],data[2]));
        poses.push_back(pose);
    }

    //计算点云拼接
    //相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);

    for (int i = 0; i < 5; i++){
        cout << "转换图像：" << i + 1 << endl;
        Mat color = color_images[i];
        Mat depth = depth_images[i];
        Sophus::SE3d T = poses[i];
        for (int v = 0; v < color.rows; v++){
            for(int u = 0; u < color.cols; u++){
                unsigned int d = depth.ptr<unsigned short>(v)[u];//用行指针取深度值
                if(d == 0) continue;// 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;

                Eigen::Vector3d pointWorld = T * point;
                Vector6d p;
                p.head<3>() = pointWorld;//取p的前三维
                //成员函数step是返回该Mat对象一行所占的数据字节数。
                //因此，对于三通道的图片color，color.step返回了图片宽度的三倍，
                //也就是说图片中一个像素占据三个字节的空间大小，分别存储B，G，R的值，该图片是3通道8位无符号数
                //成员函数data uchar类型的指针，指向Mat数据矩阵的首地址
                //可以用color.at<cv::Vec3b>(v,u)[0] color.at<cv::Vec3b>(v,u)[1], color.at<cv::Vec3b>(v,u)[2]
                p[3] = color.data[v * color.step + u * color.channels() + 2]; // R 其值是0~255
                p[4] = color.data[v * color.step + u * color.channels() + 1]; // G
                p[5] = color.data[v * color.step + u * color.channels()];     // B
                pointcloud.push_back(p);    
            }
        }
    }

    cout << "点云共有" << pointcloud.size() << "个点" << endl;
    showPointCloud(pointcloud);
    return 0;
}


void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud){
    if(pointcloud.empty()){
        cerr << "point cloud is empty" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("pointcloud viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                                      pangolin::ModelViewLookAt(0.0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
    pangolin::View &d_cam = pangolin::CreateDisplay()
                            .SetBounds(0.0, 1.0, 0, 1.0, -1024.0f / 768.0f)
                            .SetHandler(new pangolin::Handler3D(s_cam));

    while(pangolin::ShouldQuit() == false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glPointSize(2);
        glBegin(GL_POINTS);
        for(auto &p : pointcloud){
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
            
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);//5ms
    }
    return;
}