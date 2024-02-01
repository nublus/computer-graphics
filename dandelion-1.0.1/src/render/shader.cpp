#include "shader.h"
#include "../utils/math.hpp"

#ifdef _WIN32
#undef min
#undef max
#endif

using Eigen::Vector3f;
using Eigen::Vector4f;

// vertex shader & fragement shader can visit
// all the static variables below from Uniforms structure
Eigen::Matrix4f Uniforms::MVP;
Eigen::Matrix4f Uniforms::inv_trans_M;
int Uniforms::width;
int Uniforms::height;




// vertex shader
// 其中输入为模型坐标系下的顶点位置和法线，输出为变换到视口空间
// 的顶点位置（方便剪裁）和世界/相机坐标系下的法线（方便插值）
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;
    
    //Vertex position transformation顶点坐标的mvp变换，是口变换
    output_payload.position = Uniforms::MVP * payload.position;
    output_payload.position.x() = output_payload.position.x() * Uniforms::width / 2.0f /output_payload.position.w()+Uniforms::width/2.0f;
    output_payload.position.y() = output_payload.position.y() * Uniforms::height / 2.0f /output_payload.position.w()+Uniforms::height/2.0f;

    // Vertex normal transformation G = (M^−1)^T法线坐标变换到世界坐标
    Vector4f temp;
    temp << payload.normal.x(), payload.normal.y(), payload.normal.z(), 0;
    temp= Uniforms::inv_trans_M * temp;

    output_payload.normal << temp.x(), temp.y(), temp.z();
    output_payload.normal.normalize();
    return output_payload;
}

    /*! \~chinese 光源位置。 */
    //Eigen::Vector3f position;
    /*! \~chinese 光源强度。 */
    //float intensity;
Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, GL::Material material,
                               const std::list<Light>& lights, Camera camera)
{

    Vector3f result = {0, 0, 0};
    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    Eigen::Vector3f ka = material.ambient;
    Eigen::Vector3f kd = material.diffuse;
    Eigen::Vector3f ks = material.specular;
    // set ambient light intensity
    auto pos = payload.world_pos; //像素坐标
    auto n = payload.world_normal.normalized(); //像素法向
    auto view_dir = (camera.position - pos).normalized(); //视点方向
    for(auto light : lights){
        auto light_pos = light.position; //光源位置
        auto I = light.intensity; //光强
        auto rr = (light_pos-pos).squaredNorm(); //距离的平方
        auto light_dir =  (light_pos -pos).normalized(); //光源方向

        float intensity = I/rr;

        auto diffuse_light = intensity * kd * std::max(0.0f,n.dot(light_dir)); //漫反射
        auto specular_light = intensity * ks * std::pow(std::max(0.0f,n.dot((view_dir+light_dir).normalized())),material.shininess); //镜面反射
        auto ambient_light = ka * intensity; //环境光
        result += diffuse_light;
        result += specular_light;
        result += ambient_light;
    }

    for(int i=0;i<3;i++){
        if(result[i]>1) result[i]=1;
    }
    return result * 255.f;
}