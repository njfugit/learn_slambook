/*
 * @Author: Jack
 * @Date: 2022-08-07 01:06:53
 * @LastEditTime: 2022-09-02 01:30:00
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch8/direct.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
// TODO 直接法
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>

using namespace std;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

//内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
//基线
double baseline = 0.573;
string left_picture = "/home/nj/DATA/learn_slambook/ch8/left.png";
string disparity_picture = "/home/nj/DATA/learn_slambook/ch8/disparity.png";

boost::format fmt_other("/home/nj/DATA/learn_slambook/ch8/%06d.png");
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// todo 该类用于并行计算雅可比矩阵
class computerJacobian
{

public:
    computerJacobian(const cv::Mat &img1_, const cv::Mat &img2_, const VecVector2d &px_ref_, const vector<double> depth_ref_, Sophus::SE3d &T21_) : img1(img1_), img2(img2_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_)
    {
        //初始化投影点坐标
        projection = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0));
    }
    //计算某个区域的雅可比
    void accumulate_jacobian(const cv::Range &range);

    Matrix6d getH() const { return H; }
    Vector6d getb() const { return b; }
    double cost_fuc() const { return cost; }
    VecVector2d projected_points() const { return projection; }

    void reset()
    {
        H = Matrix6d::Zero();
        b = Vector6d::Zero();
        cost = 0;
    }

private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const VecVector2d &px_ref;
    const vector<double> depth_ref;
    Sophus::SE3d &T21;
    VecVector2d projection;

    std::mutex h_mutex;
    Matrix6d H = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};

void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21);

void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21);
float GetPixelValue(const cv::Mat &img, float x, float y)
{
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols)
        x = img.cols - 1;
    if (y >= img.rows)
        y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];

    //坐标的小数部分
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - yy) * xx * data[img.step] +
        xx * yy * data[img.step + 1]);
}

int main(int argc, char **argv)
{

    cv::Mat left_img = cv::imread(left_picture, 0);
    cv::Mat disparity_img = cv::imread(disparity_picture, 0);

    cv::RNG rng;

    int nPoints = 2000;
    int boarder = 20;

    return 0;
}