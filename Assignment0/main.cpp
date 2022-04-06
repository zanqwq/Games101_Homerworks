#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

// radian measure, use arch length / r length to measure an angle
// e.g., half circle arch length = pi*r, so the radian is pi*r/r = pi -> 180deg
#define PI (std::acos(-1))

Eigen::Vector3f rotate (Eigen::Vector3f& p, float angle) {
    Eigen::Matrix3f rotate_matrix;
    float radian = angle / 180.0f * PI;
    rotate_matrix <<
        std::cos(radian), -std::sin(radian), 0,
        std::sin(radian), std::cos(radian), 0,
        0, 0, 1;
    return rotate_matrix * p;
}

Eigen::Vector3f translate (Eigen::Vector3f& p, float x, float y) {
    Eigen::Matrix3f translate_matrix;
    translate_matrix <<
        1, 0, x,
        0, 1, y,
        0, 0, 1;
    return translate_matrix * p;
}

void do_homework0 () {
    std::cout << "Do Homework0" << std::endl;
    Eigen::Vector3f p;
    p << 2.0f, 1.0f, 1.0f;
    auto p1 = rotate(p, 45.0f);
    auto p2 = translate(p1, 1.0f, 2.0f);
    std::cout << "p after rotate 45deg = p1:" << std::endl;
    std::cout << p1 << std::endl;

    std::cout << "p1 after translate (1, 2) = p2:" << std::endl;
    std::cout << p2 << std::endl;
}

void homework0_ref () {
    std::cout << "Homework0 ref" << std::endl;
    Eigen::Vector3f p(2.0f,1.0f,1.0f); //定义我们的p点 写成齐次坐标形式
    // 目标是先旋转后平移 所以我们在p的左边依次乘以旋转矩阵和平移矩阵、
    Eigen::Matrix3f rotation;   //定义旋转矩阵
    Eigen::Matrix3f transform;  //定义平移矩阵
    double theta = 45.0/180.0*M_PI; //转换成弧度数

    //旋转矩阵写成其次坐标形式
    rotation <<
        cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;

    // 平移矩阵
    transform <<
        1, 0, 1,
        0, 1, 2,
        0, 0, 1;

    p = transform * rotation * p;  // 转换过程
    std::cout<< p << std::endl;   // 输出结果
}

int main(){
    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl; // holy shit, the output is line by line
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    // My Code
    do_homework0();
    homework0_ref();

    return 0;
}