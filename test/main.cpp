#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "spdlog/spdlog.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "formatter.hpp"

using namespace std;
using namespace Eigen;

#define ID 2214312083
#define STEP 0.5
#define MIN 0.01

double func(Vector2d x){
    double ret=x[0]*x[0]+x[1]*x[1];
    return ret;
}

Vector2d gradient(Vector2d x) {
    Vector2d g;
    g<<x[0]*2,x[1]*2;
    return g;
}

int main() {
    // 给定X轴的欧拉角（例如，绕X轴旋转30度）
    double roll_angle = 30.0;  // 角度以度为单位

    // 将角度转换为弧度
    double roll_angle_rad = roll_angle * M_PI / 180.0;
    std::cout<<roll_angle_rad/2<<std::endl;
    // 创建绕X轴旋转的AngleAxis对象
    Eigen::AngleAxisd roll_rotation(roll_angle_rad, Eigen::Vector3d::UnitY());

    // 使用AngleAxis对象创建四元数
    Eigen::Quaterniond quaternion(roll_rotation);

    // 打印四元数的分量
    std::cout << "Quaternion (w, x, y, z): (" << quaternion.w() << ", " << quaternion.x()
              << ", " << quaternion.y() << ", " << quaternion.z() << ")" << std::endl;

    return 0;
}

// int main(int argc, char *argv[]){
//     double x0=ID%827;
//     double y0=ID%1709;
    
//     auto console_sink =
//     std::make_shared<spdlog::sinks::stdout_color_sink_st>();
//     auto file_sink =
//     std::make_shared<spdlog::sinks::basic_file_sink_st>("optimizer.log",true);
//     spdlog::logger my_logger("optimizer", {console_sink, file_sink});
//     my_logger.set_level(spdlog::level::debug);


//     Vector2d point(x0,y0);

//     //auto logger = spdlog::stdout_color_sink_st("my_logger");

//     Matrix2d h;
//     h<<2,0,0,2;
//     Matrix2d hInverse=h.inverse();
//     Vector2d nextPoint;

//     while(true){
//         Vector2d pointGradient=gradient(point);
//         nextPoint=point-STEP*hInverse*pointGradient;
//         double gap=sqrt(
//             (point[0]-nextPoint[0])*(point[0]-nextPoint[0])+(point[1]-nextPoint[1])*(point[1]-nextPoint[1])
//             );

//         if(gap<MIN){
//             my_logger.debug("{}",point);
//             my_logger.info("{}",func(point));
//             break;
//         }
//         my_logger.debug("{}",point);
//         point=nextPoint;
//         //my_logger.debug("{}",nextPoint);
        
//     }
//     return 0;
// }


