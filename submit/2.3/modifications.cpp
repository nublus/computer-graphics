void Rasterizer::draw_mt(const std::vector<Triangle>& TriangleList, const GL::Material& material,
                         const std::list<Light>& lights, const Camera& camera)
{
    static std::vector< std::function<void(Triangle,std::array<Vector3f, 3>)> > tasks=
    std::vector<std::function<void(Triangle,std::array<Vector3f, 3>)> >();
    static std::vector< Triangle > triangles=std::vector<Triangle >();
    static std::vector< std::array<Vector3f, 3> > wps=std::vector< std::array<Vector3f, 3> >();
    static std::vector< std::thread > workers=std::vector< std::thread >();
    static std::mutex queue_mutex=std::mutex();
    static std::condition_variable condition=std::condition_variable();
    static bool stop=false;
    std::vector<int> testVec=std::vector<int>();
    for(size_t i = 0;i<8;++i)
        workers.emplace_back(
            []
            {
                for(;;)
                {
                    std::function<void(Triangle,std::array<Vector3f, 3>)> task;
                    Triangle triangle;
                    std::array<Vector3f, 3> wp;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock,
                            []{ return stop || !tasks.empty(); });

                        if(stop && tasks.empty())
                            return;
                        task = std::move(tasks.back());
                        triangle=std::move(triangles.back());
                        wp=std::move(wps.back());
                        tasks.pop_back();
                        triangles.pop_back();
                        wps.pop_back();
                    }
                    task(triangle,wp);
     
                }
            }
        );
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
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            triangles.push_back(std::move(newtriangle));
            wps.push_back(std::move(worldspace_pos));
            std::function<void(Triangle,std::array<Vector3f, 3>)> task=
            [this,material,lights,camera](Triangle triangle_,std::array<Vector3f, 3> wp_)
            {rasterize_triangle_mt(triangle_, wp_, material, lights, camera);};
            // don't allow enqueueing after stopping the pool
            if(stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks.emplace_back(task);
            condition.notify_one();
        }

        // Use vetex_shader to transform vertex attributes(position & normals) to
        // view port and set a new triangle
    }
    while(1){
        std::unique_lock<std::mutex> lock(queue_mutex);
        if(tasks.empty()){
            stop = true;
            break;
        }
    }
    condition.notify_all();
    for(std::thread &worker: workers)
        worker.join();
    triangles=std::vector<Triangle >();
    wps=std::vector< std::array<Vector3f, 3> >();
    workers=std::vector< std::thread >();
    stop=false;
}

// Screen space rasterization
void Rasterizer::rasterize_triangle_mt(const Triangle& t, const std::array<Vector3f, 3>& world_pos,
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
                    //frame_buf[buf_index] = phong_fragment_shader(payload, material, lights, camera);
                    frame_buf[buf_index] = this->fragment_shader(payload, material, lights, camera);
                }
            }
        }
    }
}

void RasterizerRenderer::render_mt(const Scene& scene)
{
    Rasterizer r(static_cast<int>(width), static_cast<int>(height));

    // choose the active shader for fragment shader
    std::function<Vector3f(FragmentShaderPayload, GL::Material, const std::list<Light>&, Camera)>
        active_shader = phong_fragment_shader;

    r.vertex_shader   = vertex_shader;
    r.fragment_shader = active_shader;

    // clear Color Buffer & Depth Buffer & rendering_res
    r.clear(BufferType::Color | BufferType::Depth);
    this->rendering_res.clear();

    time_point begin_time = steady_clock::now();
    
    // this line below is just for compiling and can be deleted
    (void)scene;
    
    // multi-thread render_mt should be completed here
    
    // TODO modify below to multi thread
    for (const auto& group : scene.groups) {

        Camera cam = scene.camera;
        // set r.view & r.projection
        r.view       = cam.view();
        r.projection = cam.projection();

        for (const auto& object : group->objects) {
            // set r.model
            r.model = object->model();
            // set Uniforms for vertex shader
            Uniforms::MVP         = r.projection * r.view * r.model;
            Uniforms::inv_trans_M = r.model.inverse().transpose();
            Uniforms::width       = r.width;
            Uniforms::height      = r.height;
            // input object->mesh's vertices & faces & normals data
            std::vector<Triangle> TriangleList;
            const std::vector<float>& vertices     = object->mesh.vertices.data;
            const std::vector<unsigned int>& faces = object->mesh.faces.data;
            const std::vector<float>& normals      = object->mesh.normals.data;

            for (unsigned int i = 0; i < faces.size(); i += 3) {
                // set triangle list(vertex & normal)
                TriangleList.emplace_back();
                Triangle& t = *TriangleList.rbegin();
                for (int j = 0; j < 3; j++) {
                    unsigned int idx = faces[i + j];
                    t.vertex[j]      = Vector4f(vertices[3 * idx], vertices[3 * idx + 1],
                                                vertices[3 * idx + 2], 1.0f);
                    t.normal[j] =
                        Vector3f(normals[3 * idx], normals[3 * idx + 1], normals[3 * idx + 2]);
                }
            }
            // call r.draw()
            r.draw_mt(TriangleList, object->mesh.material, scene.lights, cam);
        }
    }
}