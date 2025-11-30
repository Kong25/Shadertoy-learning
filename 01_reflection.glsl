// Fork of "Ray Marching Journey Starts" by JamesZFS. https://shadertoy.com/view/wXscDr
// 2025-10-05 19:39:55

// 总结：通过 SDF Raytracing （一种 Ray Marching 做法），实现渲染。光线方向都为确定性方向，无噪声。

// Globals:
float aspect;
float ambient;
float shininess;

const int n_balls = 6;
const vec3 albedos[n_balls] = vec3[](
    vec3(0.700,0.200,0.200), 
    vec3(0.871,0.871,0.871),
    vec3(0.478,0.478,0.478), 
    vec3(0.690,0.690,0.690),
    vec3(0.302,0.302,0.302), 
    vec3(0.729,0.729,0.729)
);
const vec3 centers[n_balls] = vec3[](
    vec3(4.*sin(radians(0.)), 1, 4.*cos(radians(0.))),
    vec3(4.*sin(radians(60.)), 1, 4.*cos(radians(60.))),
    vec3(4.*sin(radians(120.)), 1, 4.*cos(radians(120.))),
    vec3(4.*sin(radians(180.)), 1, 4.*cos(radians(180.))),
    vec3(4.*sin(radians(240.)), 1, 4.*cos(radians(240.))),
    vec3(4.*sin(radians(300.)), 1, 4.*cos(radians(300.)))
);

vec3 lightPos = vec3(0, 3, 0);
const float rayEps = 1e-2;

// Get the pixel space to camera space transform
// The pixel space [0, w) x [0, h) is what the fragCoord covers
// The transform returned by this method can transform a homogenized pixel coord
//  to the *unnormalized* ray direction in the camera space
// 屏幕空间像素 → 相机空间光线方向
mat3 getPixelToCamera(float w, float h, float fov) {
    return mat3(
        1, 0, 0,
        0, 1, 0,
        -w/2., -h/2., -h/tan(radians(fov/2.))/2.
    );
}

// Get the camera space to world space transform
// where o is the ray origin, t is the target, u is the up vector
// 把相机坐标系中的方向向量，旋转回世界坐标系
mat3 getCameraToWorld(vec3 o, vec3 t, vec3 u) {
    vec3 ez = normalize(o - t);          // camera forward (指向背后，因为OpenGL camera向 -Z)
    vec3 ex = normalize(cross(u, ez));   // right axis 右轴
    vec3 ey = cross(ez, ex);             // up axis   上轴

    // ex.x ey.x ez.x
    // ex.y ey.y ez.y
    // ex.z ey.z ez.z
    return mat3(ex, ey, ez);
}

// 光线行进（ray marching）不是传统光线追踪，而是基于 Signed Distance Field (SDF) 的采样。
// 这些函数返回 点到物体表面的距离。
// 如果返回值为负数，点在物体内部；为正，点在物体外部。

/// Signed distance functions

float sdfSphere(vec3 p, float r) {
    return length(p) - r;
}

float sdfPlane(vec3 p) {
    return p.y;
}

// x: the signed distance to the closest obj
// y: the id of the closest obj (starting from 1)
vec2 sdf(vec3 p) {
    float f;
    vec2 ret = vec2(1e8, 0);
    for (int i = 0; i < n_balls; ++i) {
        f = sdfSphere(p - centers[i], 1.);
        if (f < ret.x) {
            ret = vec2(f, i+1);   // 找出离最近球的距离
        }
    }
    return ret;
}

vec3 normal(vec3 p) {
    float e = 1e-3;
    float f = sdf(p).x;
    return normalize(vec3(
        sdf(p+vec3(e, 0, 0)).x - f,
        sdf(p+vec3(0, e, 0)).x - f,
        sdf(p+vec3(0, 0, e)).x - f
    ));
}

vec3 shade(vec3 v, vec3 p, vec3 n, vec3 l, vec3 kd, float ks) {
    // return normal(p);
    // Phong model:
    float I = 1.0;
    vec3 r = reflect(-v, n);  // 反射方向

    // return I * (kd * (max(dot(n, l), 0.) + ambient) + ks * pow(max(dot(r, l), 0.0), shininess));
    return I * (
        kd * (max(dot(n, l), 0.) + ambient)   // Ambient + Diffuse
        + ks * pow(max(dot(r, l), 0.0), shininess)  // Specular
);
}

int rayMarch(inout vec3 ro, vec3 rd) {
    int maxSteps = 64;

    for (int i = 0; i < maxSteps; ++i) {   // 用 ro 来做步进

        if (ro.y < 0.) {     // floor hit  步进到 y < 0. ，击中地板
            return 0;
        }

        vec2 obj = sdf(ro);  // sphere hit 计算和球的距离，距离小于一定值则返回大于 0.0 的 y 值
        float d = obj.x;
        if (d < rayEps) {
            return int(obj.y);
        }

        ro += rd * d;  // 用 d （ro与球表面的距离）来做步进幅度，不可能撞到物体。你知道自己离物体至少 d 的距离 → 往前跳 d 就不会穿模
    }
    return -1; // no hit
}

vec3 Li_ref(vec3 ro, vec3 rd) {     // reflected version
    // Ray marching
    int hit = rayMarch(ro, rd);
    
    if (hit == -1 && rd.y >= 0.) {  // sky
        float t = rd.y;  // [0, 1]
        // 天空渐变色
        vec3 col = mix(vec3(0.824,0.894,0.890), vec3(1.000,1.000,1.000), t);
        // Fog
        col = mix(col, vec3(ambient), exp(-40.*t));
        return col;
    } 
    else if (hit > 0) {             // shape
        return shade(-rd, ro, normal(ro), normalize(lightPos - ro), albedos[hit-1], 0.2);
    }
    return vec3(0);
}

vec3 Li(vec3 ro, vec3 rd) {
    // 补充知识
    // Ray Tracing 有显式几何（Mesh / BVH），解析求交 
    // Ray Marching 使用 SDF，迭代逼近交点

    // Ray marching
    int hit = rayMarch(ro, rd);
    
    if (hit == -1 && rd.y >= 0.) {  // sky
        float t = rd.y;  // [0, 1]
        // 天空渐变色
        vec3 col = mix(vec3(0.824,0.894,0.890), vec3(1.000,1.000,1.000), t);
        // Fog
        col = mix(col, vec3(ambient), exp(-40.*t));
        return col;
    } 
    else if (hit > 0) {            // shape
        return shade(-rd, ro, normal(ro), normalize(lightPos - ro), albedos[hit-1], 0.2);
    } 
    else {                         // floor (hit == 0)
        // Find exact intersection
        float t = -ro.y / rd.y;
        ro += t * rd;      // ro 移为光线与地板的交点
        
        vec3 n = vec3(0, 1, 0);
        vec3 albedo = vec3(0.420,0.420,0.420);
        
        // Diffuse
        vec3 col = shade(-rd, ro, n, normalize(lightPos - ro), albedo, 0.0);  // specular 项给 0
        //vec3 col = vec3(0.820,0.820,0.820);
        
        // Specular reflection
        vec3 ref = reflect(rd, n);
        col += 0.5 * albedo * Li_ref(ro + ref * rayEps, ref);   // 反射版的Li，从地板交点再发射出光线
        //return col;
        
        // Check shadow
        rd = normalize(lightPos - ro);
        ro += rd * rayEps;
        hit = rayMarch(ro, rd);
        if (hit <= 0) return col;
        else {       // 击中球部，返回相关的混合阴影颜色
            float t = dot(normal(ro), -rd);
            return mix(col, vec3(ambient) * albedos[hit-1], pow(t, 1.2));
        }
    } 
    return vec3(0);
}

// Camera animation
// 获得摄像机的世界坐标
vec3 getCameraPos(float t) {
    float T = 1000.0;  // period
    float w = radians(360.) / T;  // 把角度（degrees）转换为弧度（radians）  2pi/1000
    float x = 8. * sin(w*t);      // 8 * sin(2pi/1000 * 游戏运行的秒数)    1000s 一个循环
    float z = 8. * cos(w*t);      // 8 * cos(2pi/1000 * 游戏运行的秒数)    1000s 一个循环
    float y = 5.0;                // 高度 5
    return vec3(x, y, z);
}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    // Initialization
    aspect = iResolution.x / iResolution.y; // iResolution 整体的分辨率参数
    ambient = 0.5;
    shininess = 50.;      // Phong / Blinn-Phong Specular Coefficient (Like PBR Roughness)
    
    vec3 col;

    float fov = 60.;
    vec3 eye = getCameraPos(iTime);  // 获得摄像机的世界坐标
    vec3 target = vec3(0, 0., 0);    // 看向的方向
    vec3 up = vec3(0, 1, 0);         // 摄像机 up 方向

    // 标准的光追做法 （这里的光追没做反射，是确定性光追，不是随机采样光追，所以无噪声）
    mat3 pixelToCamera = getPixelToCamera(iResolution.x, iResolution.y, fov);  // 屏幕空间像素 screen -> 相机空间光线方向 camera (这是标准 true ray tracing（世界空间射线追踪），不是 SSR ray marching)
    mat3 cameraToWorld = getCameraToWorld(eye, target, up);                    // 相机空间光线方向 camera -> 世界空间光线方向 world
    vec3 rd = cameraToWorld * normalize(pixelToCamera * vec3(fragCoord + 0.5, 1.0)); // ray_direction  屏幕像素空间先转相机空间（做归一化），相机空间再转世界空间
    vec3 ro = eye;                                                                   // ray_origin

    col = Li(ro, rd);   // SDF 光追
    
    // Output to screen
    fragColor = vec4(col,1.0);
}
