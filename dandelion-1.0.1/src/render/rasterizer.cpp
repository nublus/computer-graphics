#include <array>
#include <limits>
#include <tuple>
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "rasterizer.h"
#include "triangle.h"
#include "render_engine.h"
#include "../utils/math.hpp"

#define calculate() sleep(1)

using Eigen::Matrix4f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::fill;
using std::tuple;

// 给定坐标(x,y)以及三角形的三个顶点坐标，判断(x,y)是否在三角形的内部
bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{
    Vector3f v[3];
    for (int i = 0; i < 3; i++) v[i] = {vertices[i].x(), vertices[i].y(), 1.0};

    Vector3f p(float(x), float(y), 1.0f);
    
    const Vector3f& A=v[0];
    const Vector3f& B=v[1];
    const Vector3f& C=v[2];

    Vector3f AB=B-A;
    Vector3f BC=C-B;
    Vector3f CA=A-C;

    Vector3f AP=p-A;
    Vector3f BP=p-B;
    Vector3f CP=p-C;

    float z1 = AB.cross(AP).z();
    float z2 = BC.cross(BP).z();
    float z3 = CA.cross(CP).z();

    return (( z1>0 && z2>0 && z3>0 ) || ( z1<0 && z2<0 && z3<0 ));
}

// 对当前三角形进行光栅化
void Rasterizer::rasterize_triangle(const Triangle& t, const std::array<Vector3f, 3>& world_pos, 
                                    GL::Material material, const std::list<Light>& lights,
                                    Camera camera)
{
    // discard all pixels out of the range(including x,y,z)
    // 计算三角形的包围盒
    
    float aabb_minx=0,aabb_miny=0,aabb_maxx=0,aabb_maxy=0;
    for(size_t i=0;i<3;i++){
        const Vector4f& p=t.vertex[i];
        if(i==0){
            aabb_minx=aabb_maxx=p.x();
            aabb_miny=aabb_maxy=p.y();
            continue;
        }

        aabb_minx=p.x()<aabb_minx ? p.x():aabb_minx;
        aabb_maxx=p.x()>aabb_maxx ? p.x() : aabb_maxx;
        aabb_miny = p.y()<aabb_miny ? p.y() : aabb_miny;
        aabb_maxy = p.y() > aabb_maxy ? p.y() : aabb_maxy;
    }
    //遍历每一个pixel 对当前三角形进行光栅化，在这部分中，深度值，顶点坐标和法线均进行了透视矫正插值
    for (size_t x=(int)aabb_minx ; x<aabb_maxx ; x++){
        for(size_t y=(int)aabb_miny ; y<aabb_maxy ; y++){
            if(inside_triangle(x , y , t.vertex)){
                auto [alpha, beta, gamma] = compute_barycentric_2d(x, y, t.vertex);
                int buf_index = get_index(x,y); 
                
                Eigen::Vector3f weight(t.vertex[0][3],t.vertex[1][3],t.vertex[2][3]);
                float Z = 1.0 / (alpha / t.vertex[0].w() + beta / t.vertex[1].w() + gamma /t.vertex[2].w());
                float zp = alpha * t.vertex[0].z() / t.vertex[0].w() + beta * t.vertex[1].z() / t.vertex[1].w() + gamma * t.vertex[2].z() / t.vertex[2].w();
                zp *= Z;
                    
                if(zp < depth_buf[buf_index]){
                    depth_buf[buf_index] = zp;
                    Vector3f camera_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], weight, Z);
                    camera_normal.normalize();//法线需要归一化
                    struct FragmentShaderPayload payload = {interpolate(alpha, beta, gamma, world_pos[0], world_pos[1], world_pos[2], weight, Z), camera_normal};
                    //调用phong-shader计算面片颜色
                    frame_buf[buf_index] = this->fragment_shader(payload, material, lights, camera);
                }
            }
        }
    }
}
    
    
    // if current pixel is in current triange:
    // 1. interpolate depth(use projection correction algorithm)
    // 2. interpolate vertex positon & normal(use function:interpolate())
    // 3. fragment shading(use function:fragment_shader())
    // 4. set pixel
            

// 给定坐标(x,y)以及三角形的三个顶点坐标，计算(x,y)对应的重心坐标[alpha, beta, gamma]
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

// 对当前渲染物体的所有三角形面片进行遍历，进行几何变换以及光栅化
// 调用函数呗
void Rasterizer::draw(const std::vector<Triangle>& TriangleList, const GL::Material& material,
                      const std::list<Light>& lights, const Camera& camera)
{
    // iterate over all triangles in TriangleList
    for (const auto& t : TriangleList) {
        Triangle newtriangle = t;
        // transform vertex position to world space for interpolating
        std::array<Vector3f, 3> worldspace_pos;
        for(int i=0; i<3; i++){
            // 对每个顶点应用顶点着色器
            
            Vector4f worldpos = model * newtriangle.vertex[i];
            worldspace_pos[i]=Vector3f(worldpos.x(), worldpos.y(), worldpos.z());
            // 输出为变换到视口空间
            // 的顶点位置（方便剪裁）和世界/相机坐标系下的法线（方便插值）
            struct VertexShaderPayload payload = {newtriangle.vertex[i], newtriangle.normal[i]};
            struct VertexShaderPayload new_payload = vertex_shader(payload);
            newtriangle.vertex[i] = new_payload.position;
            newtriangle.normal[i] = new_payload.normal;  
        }

        // Use vetex_shader to transform vertex attributes(position & normals) to
        // view port and set a new triangle
        rasterize_triangle(newtriangle, worldspace_pos, material, lights, camera);
    }
}

Vector3f Rasterizer::interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1,
                                 const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3,
                                 const Eigen::Vector3f& weight, const float& Z)
{
    Vector3f interpolated_res;
    for (int i = 0; i < 3; i++) {
        interpolated_res[i] = alpha * vert1[i] / weight[0] + beta * vert2[i] / weight[1] +
                              gamma * vert3[i] / weight[2];
    }
    interpolated_res *= Z;
    return interpolated_res;
}



// 初始化整个光栅化渲染器
void Rasterizer::clear(BufferType buff)
{
    if ((buff & BufferType::Color) == BufferType::Color) {
        fill(frame_buf.begin(), frame_buf.end(), RenderEngine::background_color * 255.0f);
    }
    if ((buff & BufferType::Depth) == BufferType::Depth) {
        fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

Rasterizer::Rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

// 给定像素坐标(x,y)，计算frame buffer里对应的index
int Rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

// 给定像素点以及fragement shader得到的结果，对frame buffer中对应存储位置进行赋值
void Rasterizer::set_pixel(const Vector2i& point, const Vector3f& res)
{
    int idx        = get_index(point.x(), point.y());
    frame_buf[idx] = res;
}
