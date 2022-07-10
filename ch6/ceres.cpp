/*
 * @Author: Jack
 * @Date: 2022-05-23 23:27:41
 * @LastEditTime: 2022-05-31 00:46:13
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch6/ceres.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<ceres/ceres.h>
#include<chrono>

using namespace std;

//代价函数计算模型
struct CURVE_FITTING_COST{
    CURVE_FITTING_COST(double x, double y): _x(x),_y(y){}

    //残差的计算(代价函数)
    template<typename T>
    bool operator()(const T *const abc, T *residual) const{
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);//y-exp(ax^2+bx+c)
        return true;
    }
    const double _x, _y;
};

int main(int argc, char **argv){
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;
    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;
    vector<double> x_data, y_data;
    for (int i = 0; i < N;i++){
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    double abc[3] = {ae, be, ce};

    /* ceres::Problem problem;
    for (int i = 0; i < N; i++){
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])), nullptr, abc);

    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;//增量方程如何求解:密集 Cholesky 分解 
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost =" << time_used.count() << "seconds. " << endl;

    cout << summary.FullReport() << endl; */

    //构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < N;i++){
        problem.AddResidualBlock( //添加误差项
            //使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要和前面的结构体struct CURVE_FITTING_COST中一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(//1，3分别表示残差块数量和参数块中的变量个数
                new CURVE_FITTING_COST(x_data[i], y_data[i])),
            nullptr,//核函数
            abc);//待估计参数
    }
    //配置求解器
    ceres::Solver::Options options; //这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;//增量方程如何求解
    options.minimizer_progress_to_stdout = true;    //输出到cout

    ceres::Solver::Summary summary; //优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);//开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << "seconds." << endl;

    //输出结果
    cout << summary.FullReport() << endl;
    cout << "estimated a,b,c = ";
    for(auto a:abc)
        cout << a << " ";
    cout << endl;

    return 0;
}
