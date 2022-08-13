/*
 * @Author: Jack
 * @Date: 2022-07-27 23:45:36
<<<<<<< HEAD
 * @LastEditTime: 2022-08-14 01:42:13
=======
 * @LastEditTime: 2022-08-01 01:14:47
>>>>>>> 7a0f66753985a70a31295ab0fd02c7578ef6ef3d
 * @LastEditors: your name
 * @FilePath: /ch8/o_flow.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<opencv2/opencv.hpp>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<chrono>
#include<string>

using namespace std;

string LK_path1 = "/home/nj/DATA/learn_slambook/ch8/LK1.png";
string LK_path2 = "/home/nj/DATA/learn_slambook/ch8/LK2.png";
class OpticalFlowTracker
{
    public:
        OpticalFlowTracker(
            const cv::Mat &img1_, 
            const cv::Mat &img2_,
            const vector<cv::KeyPoint> &kp1_,
            vector<cv::KeyPoint> &kp2_,
            vector<bool> &success_,
            bool inverse = true,
            bool has_initial = false):
            img1(img1_), img2(img2_), kp1(kp1_),kp2(kp2_),success(success_){}
        void calculateOpticalFlow(const cv::Range &range);
            
    private:
        const cv::Mat &img1;
        const cv::Mat &img2;
        const vector<cv::KeyPoint> &kp1;
        vector<cv::KeyPoint> &kp2;
        vector<bool> &success;
        bool inverse = true;
        bool has_initial = false;
};
/**
 * @brief 
 * @param img1 
 * @param img2 
 * @param kp1 
 * @param kp2 
 * @param success  success true if a keypoint is tracked successfully
 * @param inverse  inverse set true to enable inverse formulation 只用第一张图的梯度
 *
 */
void OpticalFlowSingle(const cv::Mat &img1,
                       const cv::Mat &img2,
                       const vector<cv::KeyPoint> &kp1,
                       vector<cv::KeyPoint> &kp2,
                       vector<bool> &success,
                       bool inverse = false, 
                       bool has_initial_guess = false);

void OpticalFlowMulti(const cv::Mat &img1,
                      const cv::Mat &img2,
                      const vector<cv::KeyPoint> &kp1,
                      vector<cv::KeyPoint> &kp2,
                      vector<bool> &success,
                      bool inverse = false);
//获取像素点的线性插值
inline float GetPixelValue(const cv::Mat &img, float x, float y){
    if(x < 0) x = 0;
    if(y < 0) y = 0;
    if(x >= img.cols-1)
        x = img.cols - 2;
    if (y >= img.rows - 1)
        y = img.rows - 2;
    float xx = x - floor(x); //floor向下取整函数　xx表示小数部分
    float yy = y - floor(y);
    int x_a1 = min(img.cols - 1, int(x) + 1);
    int y_a1 = min(img.cols - 1, int(y) + 1);

     //双线性插值　（x, y）(x+1, y) (x, y+1) (x+1, y+1)
    //int x = floor(x);
    //int y = floor(y);

    return (1 - xx) * (1 - yy) * img.at<uchar>(y, x) 
    + xx * (1 - yy) * img.at<uchar>(y, x_a1) 
    + (1 - xx) * yy * img.at<uchar>(y_a1, x) 
    + xx * yy * img.at<uchar>(y_a1, x_a1);
}

int main(int argc, char **argv){
    cv::Mat img1 = cv::imread(LK_path1, cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(LK_path2, cv::IMREAD_GRAYSCALE);

    vector<cv::KeyPoint> kp1;
    //使用harris角点作为LK光流的跟踪输入点　角点定义为在x,y方向上均有较大梯度变化的小区域
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20);
    detector->detect(img1, kp1);
    //跟踪上第二张图上这些点　在验证图中使用单层LK
    vector<cv::KeyPoint> kp2_single;
    vector<bool> success_single;
    OpticalFlowSingle(img1, img2, kp1, kp2_single, success_single);

    //多层LK
    vector<cv::KeyPoint> kp2_multi;
    vector<bool> success_multi;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    OpticalFlowMulti(img1, img2, kp1, kp2_multi, success_multi, true);//多层采用反向光流
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by gauss-newton" << time_used.count() << "seconds" << endl;
    
    //利用opencv光流验证
    vector<cv::Point2f> pt1, pt2;
    for(auto &kp : kp1)
        pt1.push_back(kp.pt);

    vector<uchar> status;
    vector<float> error;  
    t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by opencv" << time_used.count() << "seconds" << endl;

    //画出单层、多层以及opencv实现的区别
    cv::Mat img2_single;
    cv::cvtColor(img2, img2_single,cv::COLOR_GRAY2BGR);//将会灰度图转成彩色图
    for (int i = 0; i < kp2_single.size(); i++){
        if(success_single[i]){
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0), 2);
        }
    }

    cv::Mat img2_multi;
    cv::cvtColor(img2, img2_multi,cv::COLOR_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++){
        if(success_multi[i]){
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0), 2);
        }
    }

    cv::Mat img2_cv;
    cv::cvtColor(img2, img2_cv,cv::COLOR_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++){
        if(success_multi[i]){
            cv::circle(img2_cv, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_cv, pt1[i], pt2[i], cv::Scalar(0, 250, 0), 2);
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_cv);
    cv::waitKey(0);
    return 0;
}

void OpticalFlowSingle(const cv::Mat &img1,const cv::Mat &img2,const vector<cv::KeyPoint> &kp1,vector<cv::KeyPoint> &kp2,
                       vector<bool> &success,bool inverse,bool has_initial)
{
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
    //并行调用pticalFlowTracker::calculateOpticalFlow 
    cv::parallel_for_(cv::Range(0, kp1.size()), std::bind(&OpticalFlowTracker::calculateOpticalFlow, tracker, placeholders::_1));
}

void OpticalFlowTracker::calculateOpticalFlow(const cv::Range &range){
    int half_patch_size = 4;    //窗口大小
    int iterations = 10;
    //range 左闭右开 [0 ~ kp1.size())
    for (size_t i = range.start; i < range.end; i++){
        auto kp = kp1[i];
        double dx = 0, dy = 0;  //dx dy需要被估计
        if(has_initial){
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }
        double cost = 0, lastcost = 0;
        bool succ = true;   //indicate if this point succeeded

        //gauss-newton iterations
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero(); //像素坐标 误差为二维
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        Eigen::Vector2d J; //jacobian
        for (int iter = 0; iter < iterations; iter++){
            if(inverse == false){
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            }else {
                //只重置b
                b = Eigen::Vector2d::Zero();
            }
            cost = 0;
            //计算雅可比和cost
            for (int x = -half_patch_size; x < half_patch_size; x++){
                for (int y = -half_patch_size; y < half_patch_size;y++){
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                    
                    if(inverse == false){
                        //正常的光流计算　对应的雅可比为第二个图像在x+dx,y+dy处的梯度
                        J = -1.0 * Eigen::Vector2d(
                                       0.5 * (GetPixelValue(img2, kp.pt.x + x + dx + 1, kp.pt.y + y + dy) -
                                              GetPixelValue(img2, kp.pt.x + x + dx - 1, kp.pt.y + y + dy)),
                                       0.5 * (GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy + 1) -
                                              GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy - 1)));
                    }else if(iter == 0){
                        // 采用反向光流方法 　雅可比以第一张的梯度代替　J keeps same for all iterations
                        J = -1.0 * Eigen::Vector2d(
                                       0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                              GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                                       0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                              GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)));
                    }

                    b += -J * error;
                    cost += error * error;
                    if(inverse == false || iter == 0){
                        H += J * J.transpose();
                    }
                }
            }
        
            //求解更新量
            Eigen::Vector2d update = H.ldlt().solve(b);
            if (std::isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if(iter > 0 && cost > lastcost){
                break;
            }
            
            //update dx dy
            dx += update[0];
            dy += update[1];
            lastcost = cost;
            succ = true;
            if(update.norm() < 1e-2){
                break;
            }
        }
        success[i] = succ;
        kp2[i].pt = kp.pt + cv::Point2f(dx, dy);
    }
}

void OpticalFlowMulti(const cv::Mat &img1,const cv::Mat &img2,const vector<cv::KeyPoint> &kp1,
vector<cv::KeyPoint> &kp2,vector<bool> &success,bool inverse)
{

    int pyramids = 4;//金字塔层数
    double pyramids_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};//图像的缩放尺度1, 1/2, 1/4, 1/8

    //创建金字塔
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    vector<cv::Mat> pyr1, pyr2;//图像金字塔
    for (int i = 0; i < pyramids; i++){
        if(i == 0){
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }else {
            cv::Mat img1_pyr, img2_pyr;//定义新的一层图层
            cv::resize(pyr1[i - 1], img1_pyr, cv::Size(pyr1[i - 1].cols * pyramids_scale, pyr1[i - 1].rows * pyramids_scale));
            cv::resize(pyr2[i - 1], img2_pyr, cv::Size(pyr2[i - 1].cols * pyramids_scale, pyr2[i - 1].rows * pyramids_scale));
            pyr1.push_back(img1_pyr);//将缩放后图像保存
            pyr2.push_back(img2_pyr);//
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "build pyramid time:" << time_used.count() << "seconds" << endl;

    vector<cv::KeyPoint> kp1_pyr, kp2_pyr;
    //关键点坐标缩放平方倍
    for(auto &kp : kp1){
        auto kp_top = kp;
        kp_top.pt *= scales[pyramids - 1];//相应图层坐标缩放
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    for (int level = pyramids - 1; level >= 0; level--){
        success.clear();
        t1 = chrono::steady_clock::now();
        OpticalFlowSingle(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);
        t2 = chrono::steady_clock::now();
        time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "track  pyr:" << level << "cost time:" << time_used.count() << "seconds" << endl;

        if(level > 0){
            for(auto &kp:kp1_pyr)
                kp.pt /= pyramids_scale;
            for(auto &kp:kp2_pyr)
                kp.pt /= pyramids_scale;
        }
    }

    for(auto &kp:kp2_pyr)
        kp2.push_back(kp);
}