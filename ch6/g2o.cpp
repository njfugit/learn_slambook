/*
 * @Author: Jack
 * @Date: 2022-07-18 00:58:08
 * @LastEditTime: 2022-07-18 01:28:58
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
    
};




int main()
{

    return 0;
}