#include <Eigen/Geometry>
#include <iostream>

int main() {
    // 给定X轴的欧拉角（例如，绕X轴旋转30度）
    double roll_angle = 30.0;  // 角度以度为单位

    // 将角度转换为弧度
    double roll_angle_rad = roll_angle * M_PI / 180.0;

    // 创建绕X轴旋转的AngleAxis对象
    Eigen::AngleAxisd roll_rotation(roll_angle_rad, Eigen::Vector3d::UnitX());

    // 使用AngleAxis对象创建四元数
    Eigen::Quaterniond quaternion(roll_rotation);

    // 打印四元数的分量
    std::cout << "Quaternion (w, x, y, z): (" << quaternion.w() << ", " << quaternion.x()
              << ", " << quaternion.y() << ", " << quaternion.z() << ")" << std::endl;

    return 0;
}
