/*
 * @Author: Jack
 * @Date: 2022-03-19 15:55:59
 * @LastEditTime: 2022-03-24 23:04:41
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch3/useGeometry/geometryuse.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<cmath>

#include<Eigen/Core>
#include<Eigen/Geometry>
// Eigen/Geometry 模块提供了各种旋转和平移的表示方法
using namespace std;
using namespace Eigen;

int main()
{
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Matrix3d rotation_matrix = Matrix3d::Identity();//初始化一个单位矩阵
    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
    cout.precision(3);
    //用matrix()将旋转向量转换成矩阵
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;   
    //直接转成矩阵形式
    rotation_matrix = rotation_vector.toRotationMatrix();

    //通过旋转向量进行坐标变换
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl; 

    //通过旋转矩阵进行坐标变换
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation (by rotation_matrix) = " << v_rotated.transpose() << endl; 
    

    //欧拉角：旋转矩阵可以直接转换成欧拉角 以vector的形式存储
    Vector3d euler_angles = rotation_vector.matrix().eulerAngles(2, 1, 0);//等价于euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    //其中（2，1，0）表示ZYX顺序，即rpy角 yaw-pitch-roll
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    //欧式变换矩阵 Eigen::Isometry
    Isometry3d T = Isometry3d::Identity();  //虽然称为3d，实质上是4＊4的矩阵
    T.rotate(rotation_vector);
    T.pretranslate(Vector3d(1, 2, 3)); //相当于左乘 平移量
    cout << "Transform matrix = \n" << T.matrix() << endl; //先平移后旋转

    // 用变换矩阵进行坐标变换
    Vector3d v_transformed = T * v;                              // 相当于R*v+t
    cout << "v tranformed = " << v_transformed.transpose() << endl;

    // Vector3d vt_rotated = v_rotated + Vector3d(1, 2, 3);
    // cout << "vt tranformed = " << vt_rotated.transpose() << endl;


    // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可

    //四元数的使用
    Quaterniond q = Quaterniond(rotation_vector);//使用旋转向量生成四元数
    cout << "quaternion from rotation vector = " << q.coeffs().transpose()
    << endl; // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部

    q = Quaterniond(rotation_matrix);//使用旋转矩阵生成四元数
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose()
    << endl; // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    
    //使用四元数去旋转一个向量
    v_rotated = q * v;//数学上是qvq{-1} 操作符重载四元数和向量的乘法
    cout << "v rotated from quaternion = " << v.transpose() << endl;

    //如果采用常规四元数乘法为
    cout << "v rotated from quaternion = " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

    Quaterniond q1(0.55, 0.3, 0.2, 0.2), q2(-0.1, 0.3, -0.7, 0.2);
    q1.normalize();
    q2.normalize();
    Vector3d t1(0.7, 1.1, 0.2), t2(-0.1,0.4,0.8), p1(0.5,-0.1,0.2);

    Isometry3d T1w(q1), T2w(q2);//从四元数构造欧式变换矩阵
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    Vector3d p2 = T2w * T1w.inverse() * p1;
    cout.precision(6);
    cout << "p2 = " << p2.transpose() << endl;

    return 0;
}