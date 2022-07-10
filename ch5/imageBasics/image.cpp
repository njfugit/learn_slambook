/*
 * @Author: Jack
 * @Date: 2022-05-02 23:13:58
 * @LastEditTime: 2022-06-08 23:27:45
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch5/imageBasics/image.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<chrono>
using namespace std;
using namespace cv;

int main(int argc, char **argv){
    //读取argv[1]指定的图像
    Mat image;
    image = imread(argv[1]);//命令行中第二个参数路径
    //判断图像文件是否正确读取
    if(image.data == nullptr){
        cout << "文件" << argv[1] << "不存在" << endl;
        return 0;
    }

    cout << "图像的高度为" << image.rows << ",图像的宽度为" << image.cols << ",图像的通道数为" << image.channels() << endl;
    imshow("imagebasic", image);
    waitKey(0); //暂停程序，等待按键输入

    //判断image的类型
    if(image.type() != CV_8UC1 && image.type() != CV_8UC3){
        cout << "图像类型不符合要求，请输入一张彩色图或者灰度图" << endl;
        return 0;
    }

    //遍历图像，也可以用于随机像素访问
    clock_t  start_t = clock();//计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    for (size_t y = 0; y < image.rows; y++)
    {
        // 用cv::Mat::ptr获得图像的行指针
        unsigned char *row_ptr = image.ptr<unsigned char>(y);//row_ptr是第y行的行指针
        for (size_t x = 0; x < image.cols; x++)
        {
            //访问位于x,y处的像素
            unsigned char *data_ptr = &row_ptr[x * image.channels()];
            //输出该像素的每个通道，如果是灰度图就只有一个通道
            for (size_t c = 0; c < image.channels(); c++){
                unsigned char data = data_ptr[c];   //data为I(x,y)第c个通道的值
            }
        }
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "遍历图像所花时间(chrono):" << time_used.count() << "s" << endl;

    clock_t end_t = clock();
    cout << "遍历图像所花时间：" << 1000 * (end_t - start_t) / (double)CLOCKS_PER_SEC << "ms" << endl;
    //cv::Mat的拷贝
    // 直接赋值并不会拷贝数据
    Mat image_another = image;
    //修改image_another会导致image变化
    image_another(Rect(0, 0, 100, 100)).setTo(0);//左上角置零
    imshow("image", image);
    waitKey(0);


    //使用clone函数来拷贝数据
    Mat image_clone = image.clone();
    image_clone(Rect(0, 0, 100, 100)).setTo(255);
    imshow("image", image);
    imshow("image_clone", image_clone);
    waitKey(0);
    destroyAllWindows();

    return 0;
}