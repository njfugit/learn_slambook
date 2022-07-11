/*
 * @Author: Jack
 * @Date: 2022-04-26 00:21:33
 * @LastEditTime: 2022-04-27 23:43:46
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch4/example/trajectoryerror.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<fstream>
#include<pangolin/pangolin.h>
#include<sophus/se3.hpp>
#include<unistd.h>

using namespace std;
using namespace Sophus;

string gt_file = "/home/nj/DATA/slambook2/ch4/example/groundtruth.txt";
string estimated_file = "/home/nj/DATA/slambook2/ch4/example/estimated.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);
TrajectoryType ReadTrajectory(const string &path);

int main(){
    TrajectoryType groundtruth = ReadTrajectory(gt_file);
    TrajectoryType estimated = ReadTrajectory(estimated_file);

    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    //计算rmse
    double rmse = 0;
    for (size_t i = 0; i < estimated.size(); i++){
        Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
        double error = (p2.inverse() * p1).log().norm();//log()取李代数  norm()求二范数
        rmse += error * error;
    }
    rmse = rmse / double(estimated.size());//均方根误差 ||log(Tgt(-1)*Testi))取李代数||2范数  再取根号
    rmse = sqrt(rmse);
    cout << "RMSE = " << rmse << endl;

    DrawTrajectory(groundtruth, estimated);

    return 0;
}

TrajectoryType ReadTrajectory(const string &path){
    ifstream fin;
    fin.open(path, ios::in);
    TrajectoryType trajectory;
    if(!fin.is_open()){
        cerr << "trajectory" << path << "not found." << endl;
        return trajectory;
    }
    
    while(!fin.eof()){
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        trajectory.push_back(p1);
    }

    return trajectory;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti){
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)//pangolin::Attach::Pix(175)
                                .SetHandler(new pangolin::Handler3D(s_cam));
    
    while(pangolin::ShouldQuit() == false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt.size() - 1; i++){
            glColor3f(0.0f, 0.0f, 1.0f);//蓝色
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < esti.size() - 1; i++){
            glColor3f(1.0f, 0.0f, 0.0f);//红色
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
}