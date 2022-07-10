/*
 * @Author: Jack
 * @Date: 2022-03-30 23:39:57
 * @LastEditTime: 2022-04-13 23:56:33
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch3/visualizeGeometry/Geometryvisualize.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include<iostream>
#include<iomanip>



using namespace std;
#include<Eigen/Geometry>
#include<Eigen/Core>

using namespace Eigen;
#include<pangolin/pangolin.h>

struct RotationMatrix
{
    Matrix3d matrix = Matrix3d::Identity();

};

//RotationMatrix输出操作符重载
//ios 类的成员函数进行格式控制主要是通过对格式状态字、域宽、填充字符和输出精度操作来完成的。
ostream &operator<<(ostream &out, const RotationMatrix &r){
    out.setf(ios::fixed);//设置输出精度为4
    Matrix3d matrix = r.matrix;
    out << '=';
    out << "[" << setprecision(2) << matrix(0, 0) << "," << matrix(0, 1) << "," << matrix(0, 2) << "],"
        << "[" << matrix(1, 0) << "," << matrix(1, 1) << "," << matrix(1, 2) << "],"
        << "[" << matrix(2, 0) << "," << matrix(2, 1) << "," << matrix(2, 2) << "]";
    return out;
}
//RotationMatrix输入操作符重载
istream &operator>>(istream &in, RotationMatrix &r ){
    return in;
}
//TranslationVector
struct TranslationVector{
    Vector3d trans = Vector3d(0, 0, 0);
};

//输出操作符重载
ostream &operator<<(ostream &out, const TranslationVector &t){
    out << "=[" << t.trans(0) << "," << t.trans(1) << "," << t.trans(2) << "]";
    return out;
}
//输入操作符重载
istream &operator>>(istream &in, TranslationVector &t){
    return in;
}

//Quaternino
struct QuaternionDraw{
    Quaterniond q;
};
//输出操作符重载
ostream &operator<<(ostream &out, const QuaternionDraw &quat){
    auto c = quat.q.coeffs();
    out << "=[" << c[0] << "," << c[1] << "," << c[2] << "," << c[3] << "]";
    return out;
}
//输入操作符重载
istream &operator>>(istream &in, const QuaternionDraw &quat){
    return in;
}

int main(int argc, char **argv){
    /*创建窗口->启用深度测试->创建相机视图->分割视窗->
    创建交互视图->创建控制面板->创建控制面板的控件选项->
    清空颜色和缓存->绘制图案->推进窗口事件*/
    
    //该函数用于创建一个指定大小、名称的GUI窗口。
    pangolin::CreateWindowAndBind("visualize geometry", 1000, 600);

    //用来开启更新深度缓冲区的功能，如果通过比较后深度值发生变化了，
    //会进行更新深度缓冲区的操作。启动它，OpenGL就可以跟踪再Z轴上的像素，
    //这样，它只会在那个像素前方没有东西时，才会绘画这个像素。
    glEnable(GL_DEPTH_TEST);
    //处构建的相机为用于观测的相机，而非SLAM中的相机传感器。
    pangolin::OpenGlRenderState s_cam(
        //projection_matrix：用于构建观察相机的内参系数
        //modelview_matrix：用于构建观察相机及其视点的初始位置
        pangolin::ProjectionMatrix(1000, 600, 420, 420, 500, 300, 0.1, 1000),
        //fu、fv、u0、v0相机的内参，对应《视觉SLAM十四讲》中内参矩阵的fx、fy、cx、cy
        pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)
        //x、y、z：相机的初始坐标
        //lx、ly、lz：相机视点的初始位置，也即相机光轴朝向
        //up：相机自身哪一轴朝上放置
        //pangolin::AxisX：X轴正方向  pangolin::AxisNegX：X轴负方向 其他轴同理
        );

    const int UI_WIDTH = 500;
    //构建交互视图对象
    /* 
    SetBounds()：用于确定视图属性，其内参数如下：
    bottom、top：视图在视窗内的上下范围，依次为下、上，采用相对坐标表示（0：最下侧，1：最上侧）
    left、right：视图在视窗内的左右范围，依次为左、右，采用相对左边表示（0：最左侧，1：最右侧）
    aspect：视图的分辨率，也即分辨率
    参数aspect取正值，将由前四个参数设置的视图大小来裁剪达到设置的分辨率
    参数aspect取负值，将拉伸图像以充满由前四个参数设置的视图范围
    */
    //SetHandler（）：用于确定视图的相机句柄
    //pangolin::Attach::Pix(UI_WIDTH)表明左边横向UI_WIDTH个像素所有部分用于显示交互视图
    pangolin::View &d_cam = pangolin::CreateDisplay().
    SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f / 600.0f).
    SetHandler(new pangolin::Handler3D(s_cam));

    //ui
    //Pangolin将所有“控件”视为一个pangolin::Var对象
    pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
    pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
    pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());//后面做了处理
    pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
    //创建控制面板
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
    
    //控件回调函数
    //一般，将构建一个循环用于检测视窗是否被关闭
    while(!pangolin::ShouldQuit()){//判断窗口是否关闭
        //用于清空色彩缓冲区和深度缓冲区，刷新显示信息。若不使用清理，视窗将自动保留上一帧信息。
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        //激活相机、交互视图
        d_cam.Activate(s_cam);
        //模型可视化矩阵赋值给matrix T_camera_world
        pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
        Matrix4d m = matrix;
        RotationMatrix R;
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                R.matrix(i, j) = m(j, i);//m矩阵 T_camera_world 4x4矩阵取左上角3x3旋转矩阵
            }
        }
        //将几个控件赋值
        rotation_matrix = R;//T_world_camera 3x3矩阵

        TranslationVector t;
        t.trans = Vector3d(m(0, 3), m(1, 3), m(2, 3));//世界原点相对于相机世界坐标系下的坐标
        t.trans = -R.matrix * t.trans;//相机在世界坐标系下的坐标（3，3，3）
        translation_vector = t;

        TranslationVector euler;
        euler.trans = R.matrix.eulerAngles(2, 1, 0);//rpy
        euler_angles = euler;

        QuaternionDraw quat;
        quat.q = Quaterniond(R.matrix);
        quaternion = quat;

        glColor3f(1.0, 1.0, 1.0);//为颜色缓存区指定确定的值
        pangolin::glDrawColouredCube();//画一个立方体
        pangolin::glDrawAxis(2);
        //画出三个坐标轴
       /*  glLineWidth(3);
        glBegin(GL_LINES);//开始画线
        glColor3f(0.8f, 0.f, 0.f);
        
        glColor3f(0.8f, 0.f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(10, 0, 0);
        glColor3f(0.f, 0.8f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 10, 0);
        glColor3f(0.f, 0.f, 0.8f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 10);
        glEnd(); */

        pangolin::FinishFrame();
    }

    return 0;
}