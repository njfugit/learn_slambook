/*
 * @Author: Jack
 * @Date: 2022-07-23 13:13:46
 * @LastEditTime: 2022-07-26 16:49:31
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch7/pose_3d2d.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<chrono>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/sparse_optimizer.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<Eigen/Core>
#include<sophus/se3.hpp>


using namespace std;

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches);

//像素坐标转成相机归一化坐标
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);
//BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Vec2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vec3d;
void BAG2O(const Vec3d &points_3d, const Vec2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose);
//BA by Gauss-Newton
void BAGaussNewton(const Vec3d &points_3d, const Vec2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose);

int main(int argc, char** argv){
    
    string img_path1 = "/home/nj/DATA/learn_slambook/ch7/1.png"; 
    string img_path2 = "/home/nj/DATA/learn_slambook/ch7/2.png";
    string depth_path1 = "/home/nj/DATA/learn_slambook/ch7/1_depth.png";
    string depth_path2 = "/home/nj/DATA/learn_slambook/ch7/2_depth.png";
    cv::Mat img1 = cv::imread(img_path1, 1);
    cv::Mat img2 = cv::imread(img_path2, 1);
    cv::Mat depth1 = cv::imread(depth_path1, -1);
    cv::Mat depth2 = cv::imread(depth_path2, -1);
    assert(img1.data != nullptr && img2.data!=nullptr);

    vector<cv::KeyPoint> kp1, kp2;
    vector<cv::DMatch> matches;
    feature_match(img1, img2, kp1, kp2, matches);

    //建立3D点
    cv::Mat K = (cv::Mat_<double>(3,3)<<  520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<cv::Point3f> pts_3d;
    vector<cv::Point2f> pts_2d;
    for(cv::DMatch &m : matches){
        ushort d = depth1.ptr<unsigned short>(int(kp1[m.queryIdx].pt.y))[int(kp1[m.queryIdx].pt.x)];
        if(d == 0)
            continue;
        float dd = d / 5000.0;//根据数据集的说明
        //计算归一化坐标
        cv::Point2d p1 = pixel2cam(kp1[m.queryIdx].pt, K);
        //计算空间上的3d坐标
        pts_3d.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
        //另一张图上的匹配的2d坐标
        pts_2d.push_back(kp2[m.trainIdx].pt);
    }
    cout << "3d-2d pairs: " << pts_3d.size() << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    cv::Mat r, t;//r为旋转向量
    cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false); //第四个是相机畸变参数
    //将旋转向量转换成旋转矩阵
    cv::Mat R;
    cv::Rodrigues(r, R);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;
    
    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;

    Vec3d pts_3d_eigen;
    Vec2d pts_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); ++i){
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
    }

    cout << "calling bundle adjustment by gauss newton" << endl;

    Sophus::SE3d pose_gn;
    t1 = chrono::steady_clock::now();
    BAGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << endl;

    cout << "calling bundle adjustment by g2o" << endl;
    Sophus::SE3d pose_g2o;
    t1 = chrono::steady_clock::now();
    BAG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;

    return 0;
}

void feature_match(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &keypoints1, vector<cv::KeyPoint> &keypoints2, vector<cv::DMatch> &matches){
    cv::Ptr<cv::FeatureDetector> detetor = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    detetor->detect(img1, keypoints1);
    detetor->detect(img2, keypoints2);

    cv::Mat descriptors1, descriptors2;
    descriptor->compute(img1, keypoints1, descriptors1);
    descriptor->compute(img2, keypoints2, descriptors2);

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

//BA by Gauss-Newton
void BAGaussNewton(const Vec3d &points_3d, const Vec2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose){
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost = 0, lastcost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for (int iter = 0; iter < iterations; ++iter){
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        for (int i = 0; i < points_3d.size(); i++){
            Eigen::Vector3d pc = pose * points_3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] * inv_z + cx, fy * pc[1] * inv_z + cy);

            Eigen::Vector2d error = points_2d[i] - proj;

            cost += error.squaredNorm();//L2范数
            //定义雅可比矩阵
            Eigen::Matrix<double, 2, 6> J;
            //输入J表达式
            J<<-fx * inv_z, 
                0,
                fx * pc[0] * inv_z2,
                fx * pc[0] * pc[1] * inv_z2,
                -fx - fx * pc[0] * pc[0] * inv_z2,
                fx * pc[1] * inv_z,
                0,
                -fy * inv_z,
                fy * pc[1] * inv_z2,
                fy + fy * pc[1] * pc[1] * inv_z2,
                -fy * pc[0] * pc[1] * inv_z2,
                -fy * pc[0] * inv_z;
                                                                                                                               
            H += J.transpose() * J;
            b += -J.transpose() * error;
        }

        Vector6d dx;
        dx = H.ldlt().solve(b);//求解
        
        if(isnan(dx[0])){
            cout << "result is nan" << endl;
            break;
        }
        
        if(iter > 0 && cost >= lastcost){
            //cost增加，说明更新的值不是很好
            cout << "cost: " << cost << ", last cost: " << lastcost << endl;
            break;
        }
        //根据计算的dx更新位姿
        pose = Sophus::SE3d::exp(dx) * pose;//左乘（李代数）增量
        lastcost = cost;
        cout << "iteration " << iter << " cost=" << setprecision(12) << cost << endl;

        if(dx.norm()<1e-6){     //对于Vector，norm返回的是向量的二范数
            break;
        }
    }
    cout << "pose by g-n: \n"
         << pose.matrix() << endl;
    
}

//通过G2O进行优化
//定义顶点
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void setToOriginImpl() override{
        //顶点设置  设置优化变量的原始值
        _estimate = Sophus::SE3d();
    }
    //左乘 SE3  更新顶点的值
    virtual void oplusImpl(const double *update) override{
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }
    virtual bool read(istream &in) override{}
    virtual bool write(ostream &out) const override{}
};

class Edgeprojection : public g2o::BaseUnaryEdge<2,Eigen::Vector2d, VertexPose>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Edgeprojection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos(pos), _k(K){}

        //需要覆写的两个函数
        virtual void computeError() override{
            //_vertices[]存储顶点信息　如果是二元边　_vertices[]大小为2
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            Sophus::SE3d T = v->estimate(); //取出顶点里面的值
            Eigen::Vector3d pos_pixel = _k*(T*_pos);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();//取pos_pixel前两个元素（固定向量版本）
        }

        //定义雅可比矩阵
        virtual void linearizeOplus() override{
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_cam = T * _pos;
            double fx = _k(0, 0);
            double fy = _k(1, 1);
            double cx = _k(0, 2);
            double cy = _k(1, 2);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double inv_z = 1.0 / Z;
            double inv_z2 = inv_z * inv_z;
            //在G2O中　旋转在前，平移在后　　　而在Sophus中　平移在前　旋转在后
            //变换后点对极小扰动量的导数为[I -P'^]  变换顺序后[-P'^ I]
            //数上推导的雅可比矩阵前三列和后三列　应该将交换顺序
            _jacobianOplusXi << 
                fx * X * Y * inv_z2,
                -fx - fx * X * X * inv_z2,
                fx * Y * inv_z,
                -fx * inv_z,
                0,
                fx * X * inv_z2,

                fy + fy * Y * Y * inv_z2,
                -fy * X * Y * inv_z2,
                -fy * X * inv_z,
                0,
                -fy * inv_z,
                fy * Y * inv_z2;
        }
        virtual bool read(istream &in) override{}
        virtual bool write(ostream &out) const override{}
    private:
        Eigen::Vector3d _pos;
        Eigen::Matrix3d _k;
};
void BAG2O(const Vec3d &points_3d, const Vec2d &points_2d, const cv::Mat &K, Sophus::SE3d &pose){

    //构建图优化
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType; //位姿是６　路标点为３
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto linearsolver = g2o::make_unique<LinearSolverType>();
    auto blocksolver = g2o::make_unique<BlockSolverType>(std::move(linearsolver));
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(blocksolver));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    //添加vertex
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);

    //K
    Eigen::Matrix3d K_eigen;
    K_eigen<<
        K.at<double>(0,0),K.at<double>(0,1),K.at<double>(0,2),
        K.at<double>(1,0),K.at<double>(1,1),K.at<double>(1,2),
        K.at<double>(2,0),K.at<double>(2,1),K.at<double>(2,2);
    
    //添加edge
    int index = 1;
    for (size_t i = 0; i < points_2d.size(); ++i){
        auto p2d = points_2d[i];
        auto p3d = points_3d[i];
        Edgeprojection *edge = new Edgeprojection(p3d, K_eigen);
        edge->setId(index);
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "optimization cost time" << time_used.count() << "seconds" << endl;
    cout << "pose estimate by g2o" << vertex_pose->estimate().matrix() << endl;//输出位姿Ｔ　４ｘ４
    pose = vertex_pose->estimate();
}