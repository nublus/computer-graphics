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