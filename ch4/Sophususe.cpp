/*
 * @Author: Jack
 * @Date: 2022-04-23 00:12:33
 * @LastEditTime: 2022-04-23 01:09:55
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch4/Sophususe.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<cmath>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int *argc, char **argv){

    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    //四元数
    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);  // Sophus::SO3d可以直接从旋转矩阵构造
    Sophus::SO3d SO3_q(q);  //也可以通过四元数构造

    cout << "SO(3) from matrix:\n"
         << SO3_R.matrix() << endl;

    cout << "SO(3) from quaternion:\n"
         << SO3_q.matrix() << endl;

    cout << "they are equal" << endl;

    //使用对数映射获得李代数
    Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    //hat为向量到反对称矩阵
    cout << "so3 hat =\n"
         << Sophus::SO3d::hat(so3) << endl;
    //vee为反对称到向量
    cout << "so3 hat vee="
         << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;
    
    //增量扰动模型的更新
    Vector3d update_so3(1e-4, 0, 0);//假设更新量这么多
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n"
         << SO3_updated.matrix() << endl;
    cout << "************************************" << endl;

    //对SE(3)操作

    Vector3d t(1, 0, 0);
    Sophus::SE3d SE3_Rt(R, t);//通过（R, t）构造SE(3)
    Sophus::SE3d SE3_qt(q, t);//通过（q, t）构造SE(3)
    cout << "SE(3) from matrix:\n"
         << SE3_Rt.matrix() << endl;

    cout << "SE(3) from quaternion:\n"
         << SE3_qt.matrix() << endl;

    cout << "they are equal" << endl;

    //李代数se(3)是一个六维向量
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    //会发现在Sophus中，se(3)的平移在前，旋转在后

    cout << "se3 hat = \n"
         << Sophus::SE3d::hat(se3) << endl;
    cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;


    //更新
    Vector6d update_se3;
    update_se3.setZero();

    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = \n"
         << SE3_updated.matrix() << endl;

    return 0;
}