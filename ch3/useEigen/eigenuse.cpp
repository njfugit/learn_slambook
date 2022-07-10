/*
 * @Author: Jack
 * @Date: 2022-03-16 19:06:13
 * @LastEditTime: 2022-03-24 15:55:05
 * @LastEditors: your name
 * @Description: koro1FileHeader
 * @FilePath: /ch3/useEigen/eigenuse.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */


#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>//稠密矩阵的代数运算（特征值，逆）
#include<ctime>

using namespace std;
using namespace Eigen;

#define M_SIZE 100 

int main(){
    // Eigen 中所有向量和矩阵都是Eigen::Matrix，它是一个模板类。它的前三个参数为：数据类型，行，列
    Matrix<float, 2, 3> matirx_23;
    // 同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是Eigen::Matrix
    //下面两个一致，数据类型一个是double，一个是float
    Vector3d v_3d;
    Matrix<float, 3, 1> v_3f;
    
    // Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>
    Matrix3d m_3d = Matrix3d::Zero();

    // 如果不确定矩阵大小，可以使用动态大小的矩阵
    //两种方式
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    MatrixXd matrix_xd;
    
    //对矩阵的一些操作
    // 输入数据（初始化）这部分对“<<”运算符进行重载
    matirx_23 << 1,2,3,4,5,6;
    // 输出
    cout<<"matrix_23:"<<endl;
    //用（）来访问矩阵元素
    //cout<< "Matrix_23:"<<endl;
    for(int i = 0; i < 2; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {     
            cout<<matirx_23(i,j)<<"\t";
        }
        cout<<endl;
    }
    //矩阵和向量相乘
    v_3d << 3, 2, 1;
    v_3f << 4, 5, 6;
    // 但是在Eigen里你不能混合两种不同类型的矩阵，像这样是错的
    // Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    // 应该显式转换
    MatrixXd result = matirx_23.cast<double>() * v_3d;
    cout << "[1,2,3;4,5,6]*[3,2,1]: " << result.transpose() << endl;
    
    MatrixXf result2 = matirx_23 * v_3f;
    cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << endl;
    // 一些矩阵运算
    // 四则运算就不演示了，直接用+-*/即可。
    m_3d = Matrix3d::Random();//随机矩阵
    cout << "random matrix: \n" << m_3d << endl;
    cout << "transpose: \n" <<m_3d.transpose()<< endl; //转置
    cout << "vector transpose: \n" <<v_3d.transpose()<< endl; //转置
    cout << "sum: " << m_3d.sum() << endl; //求和
    cout << "trace: " << m_3d.trace() << endl;  // 迹  
    cout << "times 10: \n" << 10 * m_3d << endl;   // 数乘
    cout << "det: " <<m_3d.determinant()<<endl;//行列式

    //自伴随矩阵
    //实对称矩阵保证有特征值
    SelfAdjointEigenSolver<Matrix3d> eigen_slover(m_3d.transpose() * m_3d);
    cout << "Eigen values = \n" <<eigen_slover.eigenvalues()<<endl;
    cout << "Eigen vectors = \n" <<eigen_slover.eigenvectors()<<endl;

    // 解方程
    // 我们求解 matrix_NN * x = v_Nd 这个方程
    // N的大小在前边的宏里定义，它由随机数生成 
    //一般指定矩阵Eigen::Matrix大小时,最大值为50, 动态分配没有大小限制
    // 直接求逆自然是最直接的，但是求逆运算量大
    Matrix<double, Dynamic, Dynamic> matrix_NN = MatrixXd::Random(M_SIZE, M_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();//保证半正定

    Matrix<double, Dynamic, 1> v_Nd = MatrixXd::Random(M_SIZE, 1);

    clock_t start = clock();

    Matrix<double, M_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time of normal inverse is "<<1000 * (clock() - start) / (double) CLOCKS_PER_SEC<< "ms" <<endl;//(clock() - start) / (double) CLOCKS_PER_SEC 得到的时间单位是秒
    cout << "x = " << x.transpose() << endl;
    
    // 通常用矩阵分解来求，例如QR分解，速度会快很多
    start = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time of qr decomposition is "<<1000 * (clock() - start) / (double) CLOCKS_PER_SEC<< "ms" <<endl;
    cout << "x = " << x.transpose() << endl;

     // 对于正定矩阵，还可以用cholesky分解来解方程
    start = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "time of LDLT decomposition is "<<1000 * (clock() - start) / (double) CLOCKS_PER_SEC<< "ms" <<endl;
    cout << "x = " << x.transpose() << endl;
    return 0;

}