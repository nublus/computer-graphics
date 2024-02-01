Matrix4f Camera::projection()
{
    // const float fov_y = radians(fov_y_degrees);
    // const float top   = (target - position).norm() * std::tan(fov_y / 2.0f);
    // const float right = top * aspect_ratio;

    // Matrix4f projection = Matrix4f::Zero();
    // // 使用平行投影时，用户并不能从画面上直观地感受到相机的位置，
    // // 因而会产生近处物体裁剪过多的错觉。为了产程更好的观察效果，
    // // 这里没有使用相机本身的 near 而是取 near = -far 来让相机能看到“背后”的物体。
    // projection(0, 0) = 1.0f / right;
    // projection(1, 1) = 1.0f / top;
    // projection(2, 2) = -1.0f / far;
    // projection(2, 3) = 0.0f;
    // projection(3, 3) = 1.0f;


    //完成透视矩阵
    //根据GAMES101课程思路完成
    float n=near,f=far;
    float top=near*tan(fov_y_degrees*M_PI/360);
    float b=-top;
    float l=-aspect_ratio*top,r=aspect_ratio*top;


    //将frustum转换成长方体
    Matrix4f to_o=Matrix4f::Zero();
    to_o(0,0)=n;
    to_o(1,1)=n;
    to_o(2,2)=-(n+f);//
    to_o(2,3)=-n*f;
    to_o(3,2)=-1.0;

    //缩放
    Matrix4f o2=Matrix4f::Zero();
    o2(0,0)=2/(r-l);
    o2(1,1)=2/(top-b);
    o2(2,2)=2/(f-n);
    o2(3,3)=1;
    //平移
    Matrix4f o1=Matrix4f::Identity();
    o1(0,3)=(r+l)/(-2);
    o1(1,3)=(top+b)/(-2);
    o1(2,3)=(n+f)/(-2);
    
    return o2*o1*to_o;
}
