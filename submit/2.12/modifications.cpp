void Scene::simulation_update()
{   
    if(!during_animation)   return;


    std::chrono::time_point<std::chrono::steady_clock> frame_start = std::chrono::steady_clock::now();
    //duration frame_duraton=frame_start-last_update;
    duration remain_duration=frame_start-last_update;
    duration time_consumed(0.);

    while(std::chrono::duration_cast<std::chrono::duration<float>>(remain_duration).count() > time_step){
        for (const auto& group : groups) {
            for (const auto& object : group->objects) {
                object->update(all_objects);
            }
        }
        remain_duration -= duration(time_step);
        time_consumed += duration(time_step);
    }
    last_update = last_update + std::chrono::duration_cast<std::chrono::steady_clock::duration>(time_consumed);
}

void Object::update(vector<Object*>& all_objects)
{
    // 首先调用 step 函数计下一步该物体的运动学状态。
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    
    // 将物体的位置移动到下一步状态处，但暂时不要修改物体的速度。
    //current_state.position = next_state.position;
    // 遍历 all_objects，检查该物体在下一步状态的位置处是否会与其他物体发生碰撞。
    for (auto object : all_objects) {
        object->center = next_state.position;
        object->velocity = next_state.velocity;
        object->force = next_state.acceleration * object->mass;
        object->prev_state = current_state;
        (void)object;
        // 检测该物体与另一物体是否碰撞的方法是：
        // 遍历该物体的每一条边，构造与边重合的射线去和另一物体求交，如果求交结果非空、
        // 相交处也在这条边的两个端点之间，那么该物体与另一物体发生碰撞。
        // 请时刻注意：物体 mesh 顶点的坐标都在模型坐标系下，你需要先将其变换到世界坐标系。
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            (void)v_indices;
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。
            if (BVH_for_collision) {
            } else {
            }
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
            // current_state.position ，以避免重复碰撞。
        }
    }
    // prev_state = current_state;
    // //current_state.position = next_state.position;
    // current_state.velocity = next_state.velocity;
    // current_state.acceleration = next_state.acceleration;
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
}

KineticState forward_euler_step([[maybe_unused]] const KineticState& previous,
                                const KineticState& current)
{
    KineticState result;
    result.position = current.position + current.velocity * time_step;
    result.velocity = current.velocity + current.acceleration * time_step;
    result.acceleration = current.acceleration;
    return result;
}