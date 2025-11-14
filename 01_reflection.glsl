// Fork of "Ray Marching Journey Starts" by JamesZFS. https://shadertoy.com/view/wXscDr
// 2025-10-05 19:39:55

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
mat3 getPixelToCamera(float w, float h, float fov) {
    return mat3(1, 0, 0, 0, 1, 0, -w/2., -h/2., -h/tan(radians(fov/2.))/2.);
}

// Get the camera space to world space transform
// where o is the ray origin, t is the target, u is the up vector
mat3 getCameraToWorld(vec3 o, vec3 t, vec3 u) {
    vec3 ez = normalize(o - t);
    vec3 ex = normalize(cross(u, ez));
    vec3 ey = cross(ez, ex);
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
            ret = vec2(f, i+1);
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
    vec3 r = reflect(-v, n);
    return I * (kd * (max(dot(n, l), 0.) + ambient) + ks * pow(max(dot(r, l), 0.0), shininess));
}

int rayMarch(inout vec3 ro, vec3 rd) {
    int maxSteps = 64;

    for (int i = 0; i < maxSteps; ++i) {
        if (ro.y < 0.) {  // floor hit
            return 0;
        }
        vec2 obj = sdf(ro);
        float d = obj.x;
        if (d < rayEps) {
            return int(obj.y);
        }
        ro += rd * d;
    }
    return -1; // no hit
}

vec3 Li_ref(vec3 ro, vec3 rd) {  // reflected version
    // Ray marching
    int hit = rayMarch(ro, rd);
    
    if (hit == -1 && rd.y >= 0.) {  // sky
        float t = rd.y;  // [0, 1]
        vec3 col = mix(vec3(0.824,0.894,0.890), vec3(1.000,1.000,1.000), t);
        // Fog
        col = mix(col, vec3(ambient), exp(-40.*t));
        return col;
    } 
    else if (hit > 0) { // shape
        return shade(-rd, ro, normal(ro), normalize(lightPos - ro), albedos[hit-1], 0.2);
    }
    return vec3(0);
}

vec3 Li(vec3 ro, vec3 rd) {
    // Ray marching
    int hit = rayMarch(ro, rd);
    
    if (hit == -1 && rd.y >= 0.) {  // sky
        float t = rd.y;  // [0, 1]
        vec3 col = mix(vec3(0.824,0.894,0.890), vec3(1.000,1.000,1.000), t);
        // Fog
        col = mix(col, vec3(ambient), exp(-40.*t));
        return col;
    } 
    else if (hit > 0) { // shape
        return shade(-rd, ro, normal(ro), normalize(lightPos - ro), albedos[hit-1], 0.2);
    } 
    else {  // floor
        // Find exact intersection
        float t = -ro.y / rd.y;
        ro += t * rd;
        
        vec3 n = vec3(0, 1, 0);
        vec3 albedo = vec3(0.420,0.420,0.420);
        
        // Diffuse
        vec3 col = shade(-rd, ro, n, normalize(lightPos - ro), albedo, 0.0);
        //vec3 col = vec3(0.820,0.820,0.820);
        
        // Specular reflection
        vec3 ref = reflect(rd, n);
        col += 0.5 * albedo * Li_ref(ro + ref * rayEps, ref);   // 反射版的Li
        //return col;
        
        // Check shadow
        rd = normalize(lightPos - ro);
        ro += rd * rayEps;
        hit = rayMarch(ro, rd);
        if (hit <= 0) return col;
        else {
            float t = dot(normal(ro), -rd);
            return mix(col, vec3(ambient) * albedos[hit-1], pow(t, 1.2));
        }
    } 
    return vec3(0);
}

// Camera animation
vec3 getCameraPos(float t) {
    float T = 1000.0;  // period
    float w = radians(360.) / T;
    float x = 8. * sin(w*t);
    float z = 8. * cos(w*t);
    float y = 5.0;
    return vec3(x, y, z);
}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    // Initialization
    aspect = iResolution.x / iResolution.y;
    ambient = 0.5;
    shininess = 50.;
    
    vec3 col;

    float fov = 60.;
    vec3 eye = getCameraPos(iTime);
    vec3 target = vec3(0, 0., 0);
    vec3 up = vec3(0, 1, 0);

    mat3 pixelToCamera = getPixelToCamera(iResolution.x, iResolution.y, fov);
    mat3 cameraToWorld = getCameraToWorld(eye, target, up);
    vec3 rd = cameraToWorld * normalize(pixelToCamera * vec3(fragCoord + 0.5, 1.0));
    vec3 ro = eye;

    col = Li(ro, rd);
    
    // Output to screen
    fragColor = vec4(col,1.0);
}