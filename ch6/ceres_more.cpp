/*
 * @Author: Jack
 * @Date: 2022-07-13 21:22:06
 * @LastEditTime: 2022-07-14 21:09:51
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch6/ceres_more.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
//
// Created by xiang on 18-11-19.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include<ceres/rotation.h>
using namespace std;

// 代价函数的计算模型
struct Reprojectionerror
{
  Reprojectionerror(double x, double y):ob_x(x),ob_y(y){}

  template <typename T>
  bool operator()(const T* const camera, const T* const point, T* residuals)const{
    T p[3];
     // camera[0,1,2] are the angle-axis rotation.
    ceres::AngleAxisRotatePoint(camera, point, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];


    //符号变化来自 Noah Snavely 的 Bundler 假设的相机模型，其中相机坐标系具有负 z 轴。
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];

    //camera[7] camera[8] 最后两维表示径向畸变参数
    const T &k1 = camera[7];
    const T &k2 = camera[8];
    T r2 = xp * xp + yp * yp;
    //径向畸变1 + k1 * r^2 + k2 * r^4 + k3 * r^6
    T distortion = 1.0 + k1 * r2 + k2 * r2 * r2;
    //camera[6]表示相机的焦距
    const T &focal = camera[6];

    //只计算到图像坐标
    T predicted_x = focal * xp * distortion;
    T predicted_y = focal * yp * distortion;

    residuals[0] = predicted_x - T(ob_x);
    residuals[1] = predicted_y - T(ob_y);

    return true;
  }

    static ceres::CostFunction * create(const double ob_x, const double ob_y){
      
      return (new ceres::AutoDiffCostFunction<Reprojectionerror, 2, 9, 3>(new Reprojectionerror(ob_x, ob_y)));
    }
  }

  private:
    const double ob_x;
    const double ob_y;
};

int main(int argc, char **argv) {
  // double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
  // double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
  // int N = 100;                                 // 数据点
  // double w_sigma = 1.0;                        // 噪声Sigma值
  // double inv_sigma = 1.0 / w_sigma;
  // cv::RNG rng;                                 // OpenCV随机数产生器

  // vector<double> x_data, y_data;      // 数据
  // for (int i = 0; i < N; i++) {
  //   double x = i / 100.0;
  //   x_data.push_back(x);
  //   y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  // }

  // double abc[3] = {ae, be, ce};

  // 构建最小二乘问题
  ceres::Problem problem;
  for (int i = 0; i <  bal_problem.num_observations(); i++) {
    problem.AddResidualBlock(Reprojectionerror::create(bal_problem.observations()[2 * i + 0], bal_problem.observations()[2 * i + 0]), 
                                                        nullptr, 
                                                        bal_problem.mutable_camera_for_observation(i),
                                                        bal_problem.mutable_point_for_observation(i))
  }

  // 配置求解器
  ceres::Solver::Options options;     // 这里有很多配置项可以填
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
  options.minimizer_progress_to_stdout = true;   // 输出到cout

  ceres::Solver::Summary summary;                // 优化信息





  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);  // 开始优化
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // 输出结果
  cout << summary.BriefReport() << endl;


  return 0;
}