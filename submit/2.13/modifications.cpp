KineticState runge_kutta_step([[maybe_unused]] const KineticState& previous,
                              const KineticState& current)
{
    Vector3f v1 = current.acceleration * time_step;
    Vector3f v2 = time_step * (current.acceleration + v1 * 0.5);
    Vector3f v3 = time_step * (current.acceleration + v2 * 0.5);
    Vector3f v4 = time_step * (current.acceleration + v3);

    Vector3f v = current.velocity + (v1 + 2*v2 + 2*v3 + v4) / 6;

    Vector3f x1 = current.velocity*time_step;
    Vector3f x2 = (current.velocity + x1 * 0.5) * time_step;
    Vector3f x3 = (current.velocity + x2 * 0.5) * time_step;  
    Vector3f x4 = (current.velocity + x3) * time_step;

    Vector3f new_x = current.position + (x1 + 2 * x2 + 2 * x3 + x4) / 6;

    return KineticState(new_x, v, current.acceleration);
}

// Function to perform a single Backward Euler step
KineticState backward_euler_step([[maybe_unused]] const KineticState& previous,
                                 const KineticState& current)
{
    //先更新加速度，加速度不变
    Vector3f acc = current.acceleration;
    Vector3f v = current.velocity + acc * time_step;
    Vector3f x = current.position + v * time_step;
    return KineticState(x, v, acc);
}

// Function to perform a single Symplectic Euler step
KineticState symplectic_euler_step(const KineticState& previous, const KineticState& current)
{
    (void)previous;
    //先更新速度，在更新位移
    Vector3f v = current.velocity + current.acceleration * time_step;
    Vector3f x = current.position + v * time_step;
    return KineticState(x, v, current.acceleration);
}
