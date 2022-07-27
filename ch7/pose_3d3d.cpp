/*
 * @Author: Jack
 * @Date: 2022-07-23 13:14:24
 * @LastEditTime: 2022-07-27 16:22:57
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/pose_3d3d.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<Eigen/SVD>

#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<sophus/se3.hpp>
#include<chrono>

using namespace std;

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches);
void pose_3d3d(const vector<cv::Point3f> &pts1, const vector<cv::Point3f> &pts2, cv::Mat &R, cv::Mat &t);
cv::Point2d pixel2cam(const vector<cv::Point2d> &p, cv::Mat &K);
void BAG2O(const vector<cv::Point3f> &p_3d1, const vector<cv::Point3f> &p_3d2, cv::Mat &R, cv::Mat &t);

int main(int argc, char ** argv){
    string img_path1 = "/home/nj/DATA/learn_slambook/ch7/1.png"; 
    string img_path2 = "/home/nj/DATA/learn_slambook/ch7/2.png";
    cv::Mat img1 = cv::imread(img_path1, 1);
    cv::Mat img2 = cv::imread(img_path2, 1);
    string depth_path1 = "/home/nj/DATA/learn_slambook/ch7/1_depth.png";
    string depth_path2 = "/home/nj/DATA/learn_slambook/ch7/2_depth.png";
    cv::Mat depth1 = cv::imread(depth_path1, -1);
    cv::Mat depth2 = cv::imread(depth_path2, -1);
    vector<cv::KeyPoint> keypoints1, keypoints2;
    vector<cv::DMatch> matches;
    feature_match(img1, img2, keypoints1, keypoints2, matches);

    cout << "找到的匹配数量" << matches.size() << endl;

    //建立3D点
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<cv::Point3f> pts1, pts2;
    for(cv::DMatch m : matches){
        ushort d1 = depth1.ptr<ushort>(int(keypoints1[m.queryIdx].pt.y))[int(keypoints1[m.queryIdx].pt.x)];
        ushort d2 = depth2.ptr<ushort>(int(keypoints2[m.trainIdx].pt.y))[int(keypoints2[m.trainIdx].pt.x)];
        if(d1 == 0 || d2 == 0)
            continue;
        cv::Point2d p1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
        cv::Point2d p2 = pixel2cam(keypoints2[m.trainIdx].pt, K);
        float dd1 = float(d1) / 5000.0;
        float dd2 = float(d2) / 5000.0;

        pts1.push_back(cv::Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(cv::Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    cout << "3d-3d 匹配点对" << pts1.size() << endl;

    cv::Mat R, t;
    pose_3d3d(pts1, pts2, R, t);
    cout << "ICP通过SVD分解结果" << endl;
    cout << "R = " << R << endl;
    cout << "t" << t << endl;
    cout << "R_inv" << R.t() << endl;
    cout << "t_inv" << -R.t() * t << endl;

    cout << "BA by G2O" << endl;
    BAG2O(pts1, pts2, R, t);
    // p1= R * p2 + t;　p2是相机坐标系，p1是世界坐标系
    for (int i = 0; i < 5; ++i){
        cout << "p1=" << pts1[i] << endl;
        cout << "p2=" << pts2[i] << endl;
        cout << "R*p2+t = " << R * (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t << endl;
        cout << endl;
    }

    return 0;
}
void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches){
    cv::Ptr<cv::FeatureDetector> detetor = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    detetor->detect(img1, kp1);
    detetor->detect(img2, kp2);

    cv::Mat descriptors1, descriptors2;
    descriptor->compute(img1, kp1, descriptors1);
    descriptor->compute(img2, kp2, descriptors2);

    vector<cv::DMatch> match;
    matcher->match(descriptors1, descriptors2, match);

    double min_distance = 10000, max_distance = 0;

    for (int i = 0; i < descriptors1.rows; i++){
        double dist = match[i].distance;
        if(dist<min_distance)
            min_distance = dist;
        if(dist>max_distance)
            max_distance = dist;
        
    }
    cout << "min_distance" << min_distance << endl;
    cout << "max_distance" << max_distance << endl;

    for (int i = 0; i < descriptors1.rows;++i){
        if(match[i].distance<=max(2*min_distance,30.0)){
            matches.push_back(match[i]);
        }
    }
}
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K){
    double px = (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0);
    double py = (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1);

    return cv::Point2d(px, py);
}
void pose_3d3d(const vector<cv::Point3f> &pts1, const vector<cv::Point3f> &pts2, cv::Mat &R, cv::Mat &t){
    cv::Point3f p1, p2;//质点
    int N = pts1.size();
    for (int i = 0; i < N; i++){
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = cv::Point3f(cv::Vec3f(p1) / N);
    p2 = cv::Point3f(cv::Vec3f(p2) / N);

    vector<cv::Point3f> q1(N), q2(N);
    //去质心
    for (int i = 0; i < N; i++){
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++){
        //W = q1*q2^T
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    //关于矩阵的迹和二次型的关系　x^T*A*x = tr(A*x*x^T)
    //tr(RW)取最大值 
    //将W进行SVD分解 W = U sigma V^T   max tr(R U sigma V^T) = max tr(sigma V^T R U)

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    cout << "U = " << U << endl;
    cout << "V = " << V << endl;
    //R = U * V^T(因为行列式的值可能是正负１，当行列式的值是负１时，取－Ｒ
    Eigen::Matrix3d R_svd = U * (V.transpose());
    if(R_svd.determinant() < 0){
        R_svd = -R_svd;
    }
    //t= p1 - R*p2
    Eigen::Vector3d t_svd = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_svd * (Eigen::Vector3d(p2.x, p2.y, p2.z));

    R = (cv::Mat_<double>(3, 3) << 
         R_svd(0, 0), R_svd(0, 1), R_svd(0, 2),
         R_svd(1, 0), R_svd(1, 1), R_svd(1, 2),
         R_svd(2, 0), R_svd(2, 1), R_svd(2, 2));
    t = (cv::Mat_<double>(3, 1) << 
         t_svd(0, 0), t_svd(1, 0), t_svd(2, 0));

    cout << "R = " << R << endl;
    cout << "t = " << t << endl;
}

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        //设置顶点初值
        virtual void setToOriginImpl(){
            _estimate = Sophus::SE3d();
        }
        //更新增量
        virtual void oplusImpl(const double* update){
            Eigen::Matrix<double, 6, 1> update_eigen;
            update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
            //左乘扰动变量，更新估计值
            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        }

        virtual bool read(istream &in) override{}
        virtual bool write(ostream &out) const override{}
};

class EdgeProj : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        //根据误差优化公式适当写构造函数
        EdgeProj(const Eigen::Vector3d & p) : _point(p){}

        virtual void computeError() override{
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            _error = _measurement - v->estimate() * _point;
        }
        virtual void linearizeOplus() override{
            VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d p_trans = T * _point;
            //误差对于位姿的导数，雅可比矩阵3x6 [I -P'^],  此处旋转在前，平移在后,注意误差公式有个负号　为-[-P'^ I]＝[P'^ -I]
            _jacobianOplusXi.block<3, 3>(0, 0) << Sophus::SE3d::hat(p_trans);
            _jacobianOplusXi.block<3, 3>(0, 3) << -Eigen::Matrix3d::Identity();
            
        }
        virtual bool read(istream &in) override {}
        virtual bool write(ostream &out) const override{}

    private:
        Eigen::Vector3d _point;
};

void BAG2O(const vector<cv::Point3f> &p_3d1, const vector<cv::Point3f> &p_3d2, cv::Mat &R, cv::Mat &t){
    //构建图优化　
    //定义BlockSoloverType -> 定义LinearSolverType -> linearsolver -> blocksolver -> 选择优化策略
    typedef g2o::BlockSolverX BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto linearsolver = g2o::make_unique<LinearSolverType>();
    auto blocksolver = g2o::make_unique<BlockSolverType>(std::move(linearsolver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blocksolver));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    //添加点
    VertexPose *v = new VertexPose();
    v->setId(0);
    v->setEstimate(Sophus::SE3d());
    optimizer.addVertex(v);

    //add edges
    for (int i = 0; i < p_3d1.size(); i++){
        EdgeProj *edge = new EdgeProj(Eigen::Vector3d(p_3d1[i].x, p_3d1[i].y, p_3d1[i].z));
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(Eigen::Vector3d(p_3d1[i].x, p_3d1[i].y, p_3d1[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity());//信息矩阵　方差矩阵的逆
        optimizer.addEdge(edge);
    }
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "optimization cost " << time_used.count() << "seconds" << endl;

    cout << "T = \n"
         << v->estimate().matrix() << endl;//得到4x4的旋转矩阵
    //将格式转成cv::Mat
    Eigen::Matrix3d R_ba = v->estimate().rotationMatrix();
    Eigen::Vector3d t_ba = v->estimate().translation();

    R = (cv::Mat_<double>(3, 3) << 
         R_ba(0, 0), R_ba(0, 1), R_ba(0, 2),
         R_ba(1, 0), R_ba(1, 1), R_ba(1, 2),
         R_ba(2, 0), R_ba(2, 1), R_ba(2, 2));
    t = (cv::Mat_<double>(3, 1) << 
         t_ba(0, 0), t_ba(1,0), t_ba(2, 0));
         
}