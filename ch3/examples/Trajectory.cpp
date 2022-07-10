/*
 * @Author: Jack
 * @Date: 2022-04-17 10:11:28
 * @LastEditTime: 2022-04-17 16:31:04
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch3/examples/Trajectory.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<pangolin/pangolin.h>
#include<Eigen/Core>
#include<fstream>
#include<unistd.h>

using namespace std;
using namespace Eigen;

void drawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main()
{
    string trajectory_file = "/home/nijun/slambook2/ch3/examples/trajectory.txt";
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    ifstream fin;
    fin.open(trajectory_file, ios::in);
    if(!fin.is_open()){
        cout << "trajectory.txt文件打开失败" << endl;
        return -1;
    }
    while(!fin.eof()){
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Vector3d(tx, ty, tz));
        poses.push_back(Twr);
        
    }
    cout << "read trajectory_file total" << poses.size() << endl;
    drawTrajectory(poses);

    return 0;
}
void drawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses){
    pangolin::CreateWindowAndBind("Trajectory", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay().
    SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f).
    SetHandler(new pangolin::Handler3D(s_cam));
    
    while(pangolin::ShouldQuit() == false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);

        for (size_t i = 0; i < poses.size(); i++){
                //画每个位姿的三个坐标轴
                Vector3d Ow = poses[i].translation();
                Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
                Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
                Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));

                //三个坐标轴画线
                glBegin(GL_LINES);
                glColor3f(1.0, 0.0, 0.0);
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Xw[0], Xw[1], Xw[2]);

                glColor3f(0.0, 1.0, 0.0);
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Yw[0], Yw[1], Yw[2]);

                glColor3f(0.0, 0.0, 1.0);
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Zw[0], Zw[1], Zw[2]);

                glEnd();
            }
            for (size_t i = 0; i < poses.size(); i++){
                glColor3f(0.0, 0.0, 0.0);
                glBegin(GL_LINES);
                auto p1 = poses[i], p2 = poses[i + 1];
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                glEnd();
            }
            pangolin::FinishFrame();
            usleep(5000);//usleep返回的时间单位是微秒
        }
}
