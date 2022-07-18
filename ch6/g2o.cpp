/*
 * @Author: Jack
 * @Date: 2022-07-18 00:58:08
 * @LastEditTime: 2022-07-18 21:45:57
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch6/g2o.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<g2o/core/g2o_core_api.h>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/linear_solver.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_dogleg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<Eigen/Core>
#include<opencv2/opencv.hpp>
#include<cmath>
#include<chrono>
#include<vector>
using namespace std;

class myVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        virtual bool read(istream &in){}
        virtual bool write(ostream &out) const{}
        virtual void setToOriginImpl() override{
            _estimate << 0, 0, 0;
        }
        virtual void oplusImpl(const double* update) override{
            _estimate += Eigen::Vector3d(update);
        }
};

class myEdge : public g2o::BaseUnaryEdge<1, double, myVertex>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        //构造函数
        myEdge(double x) : BaseUnaryEdge(),  _x(x){}

        virtual bool read(istream &in){}
        virtual bool write(ostream &out) const{}
        virtual void computeError() override{
            const myVertex *v = static_cast<const myVertex *>(_vertices[0]);
            const Eigen::Vector3d abc = v->estimate();
            _error(0,0) = _measurement - exp(abc(0,0) * _x * _x + abc(1,0) * _x + abc(2,0));
        }
        virtual void linearizeOplus() override{
            const myVertex *v = static_cast<const myVertex *>(_vertices[0]);
            const Eigen::Vector3d abc = v->estimate();
            double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
            _jacobianOplusXi[0] = -_x * _x * y;
            _jacobianOplusXi[1] = -_x * y;
            _jacobianOplusXi[2] = -y;
            
        }

    public:
        double _x;   //ｙ值为＿measurement
};

int main()
{
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
    //构建图优化 先定义Block类型, 根据Block的定义linearSolver类型
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;//误差项优化维度是３，误差值维度为１
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    //定义LinearSolver指针
    auto linear = g2o::make_unique<LinearSolverType>();//线性求解器类型
    //定义BlockSolver指针,利用上面生成的ＬinearSolver指针
    auto BlockSolver = g2o::make_unique<BlockSolverType>(move(linear));
    //定义slover, 选择优化策略　梯度下降算法
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(move(BlockSolver));

    //g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    //定义稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true); //打开调试开关

    myVertex *v = new myVertex();
    //图中添加节点信息, 该节点表示待估计的参数
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    //图中增加边
    for (int i = 0; i < N; i++){
        myEdge *edge = new myEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v); //将超边上的第0个顶点设置为提供的指针
        //设置测量值
        edge->setMeasurement(y_data[i]);
        //设置信息矩阵（方差矩阵的逆）
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 * inv_sigma * inv_sigma);
        optimizer.addEdge(edge);
    }

    //执行优化流程
    cout << "start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    
    optimizer.initializeOptimization();
    optimizer.optimize(10);//设置迭代次数为10

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost =" << time_used.count() << "seconds." << endl;

    //输出优化的值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "eatimated value:" << abc_estimate.transpose() << endl;

    return 0;
}