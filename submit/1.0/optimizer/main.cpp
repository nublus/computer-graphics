#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "spdlog/spdlog.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "formatter.hpp"
#include <math.h>

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



int main(int argc, char *argv[]){
    // double x0=ID%827;
    // double y0=ID%1709;
    
    // auto console_sink =
    // std::make_shared<spdlog::sinks::stdout_color_sink_st>();
    // auto file_sink =
    // std::make_shared<spdlog::sinks::basic_file_sink_st>("optimizer.log",true);
    // spdlog::logger my_logger("my logger", {console_sink, file_sink});
    // my_logger.set_level(spdlog::level::debug);


    // Vector2d point(x0,y0);


    // Matrix2d h;
    // h<<2,0,0,2;
    // Matrix2d hInverse=h.inverse();
    // Vector2d nextPoint;

    // while(true){
    //     Vector2d pointGradient=gradient(point);
    //     nextPoint=point-STEP*hInverse*pointGradient;
    //     double gap=sqrt(
    //         (point[0]-nextPoint[0])*(point[0]-nextPoint[0])+(point[1]-nextPoint[1])*(point[1]-nextPoint[1])
    //         );
            
    //     if(gap<MIN){
    //         my_logger.debug("{}",point);
    //         my_logger.info("{}",func(point));
    //         break;
    //     }
    //     my_logger.debug("{}",point);
    //     point=nextPoint;
    //     //my_logger.debug("{}",nextPoint);
        
    // }

    float n=0.1,f=10;
    float top=0.1*tan(22.5*M_PI/180);
    float b=-top;
    float l=-1.33*top,r=1.33*top;
    Matrix4f to_o=Matrix4f::Zero();
    to_o(0,0)=n;
    to_o(1,1)=n;
    to_o(2,2)=-(n+f);//
    to_o(2,3)=-n*f;
    to_o(3,2)=-1.0;


    Matrix4f o2=Matrix4f::Zero();
    o2(0,0)=2/(r-l);
    o2(1,1)=2/(top-b);
    o2(2,2)=2/(f-n);//
    o2(3,3)=1;
    Matrix4f o1=Matrix4f::Identity();
    o1(0,3)=(r+l)/(-2);
    o1(1,3)=(top+b)/(-2);
    o1(2,3)=(n+f)/(-2);//
    //o1(2,3)=-1.1/2;
    o1(3,3)=1;

    std::cout<<"o1*o2"<<endl<<o2*o1<<endl<<endl<<"to_o: "<<endl<<to_o<<endl<<endl;
    //std::cout<<"o1:"<<endl<<o1<<endl;
    //std::cout<<"o2:"<<endl<<o2<<endl;
    std::cout<<"o2*o1*to_o:"<<endl<<o2*o1*to_o<<endl;
    // Matrix4f projection = Matrix4f::Zero();
    // projection(0, 0) = 0.3;
    // projection(1, 1) = 0.3;
    // projection(2, 2) = -0.1;
    // projection(2, 3) = 0.0f;
    // projection(3, 3) = 1.0f;
    // std::cout<<projection<<endl;
    // std::cout<<projection*to_o<<endl;
    // std::cout<<o1*o2*to_o<<endl;


    return 0;
}


