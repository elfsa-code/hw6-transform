#include <Eigen/Dense>
#include <iostream>

int main() {
    // 定义一个点在世界坐标系中的坐标P_w
    Eigen::Vector4d P_w(1.0, 2.0, 3.0, 1.0);

    // 定义机器人在世界坐标系中的位置t_wr
    Eigen::Vector3d t_wr(4.0, 5.0, 6.0);

    // --------- 开始你的代码	---------------//
    const double D2R = M_PI/180;

    // 构建旋转矩阵R_wc，from world to robot X:45;Y:30

    Eigen::Vector3d axis_wb_(1.0, 0.0, 0.0);
    double angle_wb_ = 45*D2R;
    Eigen::AngleAxisd rotation_vector_wb_(angle_wb_, axis_wb_);
    Eigen::Matrix3d rotation_matrix_wb_ = rotation_vector_wb_.toRotationMatrix();

    //Eigen::Vector3d axis_b_b(0.0, sqrt(2)/2, -sqrt(2)/2);
    Eigen::Vector3d axis_b_b(0.0, 1.0, 0.0);
    double angle_b_b = 30*D2R;
    Eigen::AngleAxisd rotation_vector_b_b(angle_b_b, axis_b_b);
    Eigen::Matrix3d rotation_matrix_b_b = rotation_vector_b_b.toRotationMatrix();

    Eigen::Matrix3d rotation_matrix_wb = rotation_matrix_b_b*rotation_matrix_wb_;

    // 构建变换矩阵T_wc
    Eigen::Affine3d transformation_matrix_wb =
            Eigen::Affine3d::Identity();
    transformation_matrix_wb.translate(t_wr);
    transformation_matrix_wb.rotate(rotation_matrix_wb);

    // Print the transformation matrix
    std::cout << transformation_matrix_wb.matrix() << std::endl;

    // 计算点在机器人坐标系中的坐标P_c = T_cw * P_w

    Eigen::Vector4d P_c = transformation_matrix_wb.matrix().inverse()*P_w;

    // --------- 结束你的代码	---------------//

    std::cout << "The point in the robot coordinate system is: \n" << P_c << std::endl;

    return 0;
}
