#include <optixu/optixu_math_namespace.h>
#include <cfloat>

#include "optix_path_tracer.h"

#define PI 3.14159265359f

static constexpr float offset = 1e-4f; // Global ray distance offset to avoid self-intersections

struct RayState {
    uint rnd;
    uint depth;
    float3 contrib;
    float3 color;
    float mis;
    float3 next_org;
    float3 next_dir;
    bool done;
};

struct ShadowRayState {
    bool in_shadow;
};

struct MaterialSample {
    float pdf;
    float3 color;
    float cos;
    float3 dir;
};

// Variables -----------------------------------------------------------------------

// Attributes
rtDeclareVariable(float3,   attr_normal,      attribute normal     , );
rtDeclareVariable(float3,   attr_face_normal, attribute face_normal, );
rtDeclareVariable(float2,   attr_texcoord,    attribute texcoord   , );
rtDeclareVariable(int3,     attr_index,       attribute index      , );
rtDeclareVariable(uint,     attr_material,    attribute material   , );

// Per ray
rtDeclareVariable(optix::Ray, ray,          rtCurrentRay          , );
rtDeclareVariable(float,      ray_dist,     rtIntersectionDistance, );
rtDeclareVariable(uint2,      launch_index, rtLaunchIndex         , );

// Globals
rtDeclareVariable(rtObject, top_object,        , );
rtDeclareVariable(float,    pdf_lightpick,     , );
rtDeclareVariable(uint,     frame_number,      , );
rtDeclareVariable(uint,     num_lights,        , );
rtDeclareVariable(uint,     max_path_depth,    , );
rtDeclareVariable(uint,     samples_per_pixel, , );
rtDeclareVariable(uint,     film_width,        , );
rtDeclareVariable(uint,     film_height,       , );

rtDeclareVariable(float3, cam_eye,   , );
rtDeclareVariable(float3, cam_dir,   , );
rtDeclareVariable(float3, cam_right, , );
rtDeclareVariable(float3, cam_up,    , );
rtDeclareVariable(float2, cam_dim,   , );

// Buffers
rtBuffer<float3>   frame_buffer;
rtBuffer<Material> materials;
rtBuffer<Light>    lights;

rtBuffer<float3> vertex_buffer;     
rtBuffer<float3> normal_buffer;
rtBuffer<float2> texcoord_buffer;
rtBuffer<int4>   index_buffer;

// Utility functions ---------------------------------------------------------------

__device__ int32_t xorshift(uint& seed) {
    auto x = seed;
    x = x == 0 ? 1 : x;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    seed = x;
    return x;
}

__device__ float randf(uint& rnd) {
    uint u = xorshift(rnd);
    return __int_as_float((127u << 23u) | (u & 0x7FFFFFu)) - 1.0f;
}

__device__ int32_t randi(uint& rnd) {
    return xorshift(rnd);
}

__device__ uint fnv_init() { return 0x811C9DC5u; }

__device__ uint fnv_hash(uint h, uint d) {
    h = (h * 16777619u) ^ ( d         & 0xFFu);
    h = (h * 16777619u) ^ ((d >>  8u) & 0xFFu);
    h = (h * 16777619u) ^ ((d >> 16u) & 0xFFu);
    h = (h * 16777619u) ^ ((d >> 24u) & 0xFFu);
    return h;
}

__device__ void gen_local_coords(const float3& normal, float3& tangent, float3& bitangent) {
    auto sign = normal.z >= 0.0f ? 1.0f : -1.0f;
    auto a = -1.0f / (sign + normal.z);
    auto b = normal.x * normal.y * a;

    tangent   = make_float3(1.0f + sign * normal.x * normal.x * a, sign * b, -sign * normal.x);
    bitangent = make_float3(b, sign + normal.y * normal.y * a, -normal.y);
}

__device__ float dot(const float3& a, const float3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float3 cross(const float3& a, const float3& b) {
    return make_float3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

__device__ float length(const float3& v) {
    return sqrtf(dot(v, v));
}

__device__ float3 reflect(const float3& v, const float3& n) {
    return (2 * dot(n, v)) * n - v;
}

__device__ float3 normalize(const float3& v) {
    return v * (1.0f / length(v));
}

__device__ float triangle_area(const float3& v0, const float3& v1, const float3& v2) {
    auto n = cross(v1 - v0, v2 - v0);
    return length(n) * 0.5f;
}

__device__ float3 sample_triangle(float u, float v, const float3& v0, const float3& v1, const float3& v2) {
    if (u + v >= 1.0f) {
        u = 1.0f - u;
        v = 1.0f - v;
    }
    return v0 * (1.0f - u - v) + v1 * u + v2 * v;
}

__device__ float luminance(const float3& color) {
    return color.x * 0.2126f + color.y * 0.7152f + color.z * 0.0722f;
}

__device__ float russian_roulette(const float3& color, float clamp) {
    auto prob = 2.0f * luminance(color);
    return prob > clamp ? clamp : prob;
}

template <typename T> __device__ T min(T a, T b) { return a < b ? a : b; }
template <typename T> __device__ T max(T a, T b) { return a > b ? a : b; }
template <typename T> __device__ T clamp(T a, T b, T c) { return min(c, max(a, b)); }
template <typename T, typename U> __device__ T lerp(T a, T b, U k) { return (1.0f - k) * a + k * b; }

// Textures ------------------------------------------------------------------------

__device__ float3 rgba32_to_float3(uint pix) {
    auto r =  pix        & 0xFF;
    auto g = (pix >>  8) & 0xFF;
    auto b = (pix >> 16) & 0xFF;
    auto inv = 1.0f / 255.0f;
    return make_float3(r * inv, g * inv, b * inv);
}

__device__ float3 lookup_texture(int id, float2 uv) {
    auto pix = optix::rtTex2D<float4>(id, uv.x, uv.y);
    return make_float3(pix.x, pix.y, pix.z);
}

// Mesh ----------------------------------------------------------------------------

RT_PROGRAM void mesh_intersect(int prim_id)
{
    auto ids = index_buffer[prim_id];
    auto v0 = vertex_buffer[ids.x];
    auto v1 = vertex_buffer[ids.y];
    auto v2 = vertex_buffer[ids.z];
    float3 fn;
    float t, u, v;
    if (optix::intersect_triangle(ray, v0, v1, v2, fn, t, u, v) && rtPotentialIntersection(t)) {
        attr_texcoord    = (1.0f - u - v) * texcoord_buffer[ids.x] + u * texcoord_buffer[ids.y] + v * texcoord_buffer[ids.z];
        attr_normal      = (1.0f - u - v) * normal_buffer  [ids.x] + u * normal_buffer  [ids.y] + v * normal_buffer  [ids.z];
        attr_face_normal = fn;
        attr_index       = make_int3(ids.x, ids.y, ids.z);
        attr_material    = ids.w;
        rtReportIntersection(0);
    }
}

RT_PROGRAM void mesh_bounds(int prim_id, float result[6]) {
    auto ids = index_buffer[prim_id];
    auto v0 = vertex_buffer[ids.x];
    auto v1 = vertex_buffer[ids.y];
    auto v2 = vertex_buffer[ids.z];
    result[0] = fminf(v0.x, fminf(v1.x, v2.x));
    result[1] = fminf(v0.y, fminf(v1.y, v2.y));
    result[2] = fminf(v0.z, fminf(v1.z, v2.z));
    result[3] = fmaxf(v0.x, fmaxf(v1.x, v2.x));
    result[4] = fmaxf(v0.y, fmaxf(v1.y, v2.y));
    result[5] = fmaxf(v0.z, fmaxf(v1.z, v2.z));
}

// Materials -----------------------------------------------------------------------

__device__ float phong_interp(const float3& kd, const float3& ks) {
    float lum_ks = luminance(ks);
    float lum_kd = luminance(kd);
    return lum_ks + lum_kd == 0 ? 0.0f : lum_ks / (lum_ks + lum_kd);
}

__device__ float specular_sample_pdf(float ns, const float3& normal, const float3& out_dir, const float3& in_dir) {
    auto cos = fmaxf(dot(in_dir, reflect(out_dir, normal)), 0.0f);
    return powf(cos, ns) * (ns + 1.0f) * (1.0f / (2.0f * PI));
}

__device__ float diffuse_sample_pdf(const float3& normal, const float3& in_dir) {
    return fmaxf(dot(in_dir, normal), 0.0f) * (1.0f / PI);
}

__device__ float phong_sample_pdf(const Material& mat, const float3& normal, const float2& uv, const float3& out_dir, const float3& in_dir) {
    auto ks = mat.map_ks >= 0 ? lookup_texture(mat.map_ks, uv) : mat.ks;
    auto kd = mat.map_kd >= 0 ? lookup_texture(mat.map_kd, uv) : mat.kd;
    return lerp(diffuse_sample_pdf(normal, in_dir), specular_sample_pdf(mat.ns, normal, out_dir, in_dir), phong_interp(kd, ks));
}

__device__ float3 eval_specular_bsdf(const float3& ks, float ns, const float3& normal, const float3& out_dir, const float3& in_dir) {
    auto cos = fmaxf(dot(in_dir, reflect(out_dir, normal)), 0.0f);
    return ks * powf(cos, ns) * (ns + 2.0f) * (1.0f / (2.0f * PI));
}

__device__ float3 eval_diffuse_bsdf(const float3& kd) {
    return kd * (1.0f / PI);
}

__device__ float3 eval_phong_bsdf(const Material& mat, const float3& normal, const float2& uv, const float3& out_dir, const float3& in_dir) {
    auto ks = mat.map_ks >= 0 ? lookup_texture(mat.map_ks, uv) : mat.ks;
    auto kd = mat.map_kd >= 0 ? lookup_texture(mat.map_kd, uv) : mat.kd;
    return lerp(eval_diffuse_bsdf(kd), eval_specular_bsdf(ks, mat.ns, normal, out_dir, in_dir), phong_interp(kd, ks));
}

__device__ MaterialSample sample_diffuse_bsdf(const float3& kd, const float3& normal, uint& rnd, const float3& out_dir) {
    // Cosine hemisphere sampling
    auto u = randf(rnd);
    auto v = randf(rnd);
    auto cos = sqrtf(1.0f - v);
    auto sin = sqrtf(v);
    auto phi = 2.0f * PI * u;

    float3 tangent, bitangent;
    gen_local_coords(normal, tangent, bitangent);

    MaterialSample sample;
    sample.dir = sin * cosf(phi) * tangent + sin * sinf(phi) * bitangent + cos * normal;

    sample.color = kd * (1.0f / PI);
    sample.pdf   = cos * (1.0f / PI);
    sample.cos   = cos;
    return sample;
}

__device__ MaterialSample sample_specular_bsdf(const float3& ks, float ns, const float3& normal, uint& rnd, const float3& out_dir) {
    // Cosine-power hemisphere sampling
    auto u = randf(rnd);
    auto v = randf(rnd);
    auto reflect_out = reflect(out_dir, normal);
    auto cos = powf(v, 1.0f / (ns + 1.0f));
    auto sin = sqrtf(1.0f - cos * cos);
    auto phi = 2.0f * PI * u;

    float3 tangent, bitangent;
    gen_local_coords(reflect_out, tangent, bitangent);

    MaterialSample sample;
    sample.dir = sin * cosf(phi) * tangent + sin * sinf(phi) * bitangent + cos * reflect_out;

    auto lobe = powf(cos, ns) * (1.0f / (2.0f * PI));

    sample.color = ks * lobe * (ns + 2.0f);
    sample.pdf   = lobe * (ns + 1.0f);
    sample.cos   = fmaxf(dot(sample.dir, normal), 0.0f);
    return sample;
}

__device__ MaterialSample sample_phong_bsdf(const Material& mat, const float3& normal, const float3& face_normal, const float2& uv, uint& rnd, const float3& out_dir) {
    auto ks = mat.map_ks >= 0 ? lookup_texture(mat.map_ks, uv) : mat.ks;
    auto kd = mat.map_kd >= 0 ? lookup_texture(mat.map_kd, uv) : mat.kd;
    float ns = mat.ns;
    float k = phong_interp(kd, ks);
    bool use_kd = randf(rnd) >= k;
    MaterialSample sample;

    sample = use_kd ? sample_diffuse_bsdf(kd, normal, rnd, out_dir) : sample_specular_bsdf(ks, ns, normal, rnd, out_dir);

    if (sample.pdf <= 0.0f || dot(sample.dir, face_normal) <= 0.0f) {
        sample.pdf = 1.0f;
        sample.color = make_float3(0.0f, 0.0f, 0.0f);
    } else if (use_kd) {
        sample.color = lerp(sample.color, eval_specular_bsdf(ks, ns, normal, out_dir, sample.dir), k);
        sample.pdf   = lerp(sample.pdf,   specular_sample_pdf(ns, normal, out_dir, sample.dir),    k);
    } else {
        sample.color = lerp(eval_diffuse_bsdf(kd),                  sample.color, k);
        sample.pdf   = lerp(diffuse_sample_pdf(normal, sample.dir), sample.pdf,   k);
    }
    return sample;
}

__device__ MaterialSample sample_mirror_bsdf(const Material& mat, const float3& normal, const float3& face_normal, const float3& out_dir) {
    MaterialSample sample;
    sample.cos   = 1.0f;
    sample.pdf   = 1.0f;
    sample.dir   = reflect(out_dir, normal);
    sample.color = dot(sample.dir, face_normal) <= 0 ? make_float3(0.0f, 0.0f, 0.0f) : mat.ks;
    return sample;
}

__device__ float fresnel_factor(float k, float cos_i, float cos_t) {
    const float R_s = (k * cos_i - cos_t) / (k * cos_i + cos_t);
    const float R_p = (cos_i - k * cos_t) / (cos_i + k * cos_t);
    return (R_s * R_s + R_p * R_p) * 0.5f;
}

__device__ MaterialSample sample_glass_bsdf(bool entering, const Material& mat, const float3& normal, const float3& face_normal, uint& rnd, const float3& out_dir) {
    auto n1 = 1.0f;
    auto n2 = mat.ni;
    if (!entering) {
        auto tmp = n1;
        n1 = n2;
        n2 = tmp;
    }
    auto n = n1 / n2;

    auto cos_incoming = dot(out_dir, normal);
    auto cos2_transmitted = 1.0f - n * n * (1.0f - cos_incoming * cos_incoming);

    if (cos2_transmitted > 0.0f) {
        // Refraction
        auto cos_transmitted = sqrtf(cos2_transmitted);
        auto F = fresnel_factor(n, cos_incoming, cos_transmitted);
        if (randf(rnd) > F) {
            auto t = normal * (n * cos_incoming - cos_transmitted) - out_dir * n;
            auto color = dot(t, face_normal) >= 0 ? make_float3(0.0f, 0.0f, 0.0f) : mat.tf;
            MaterialSample sample;
            sample.cos   = 1.0f;
            sample.pdf   = 1.0f;
            sample.dir   = t;
            sample.color = color;
            return sample;
        }
    }

    // Reflection
    return sample_mirror_bsdf(mat, normal, face_normal, out_dir);
}

// OptiX programs ------------------------------------------------------------------

RT_PROGRAM void path_trace() {
    auto x = launch_index.x;
    auto y = launch_index.y;
    auto samples = samples_per_pixel;
    auto result  = make_float3(0.0f, 0.0f, 0.0f);

    do { 
        // Generate ray
        uint rnd = fnv_hash(fnv_init(), x);
        rnd = fnv_hash(rnd, y);
        rnd = fnv_hash(rnd, samples);
        rnd = fnv_hash(rnd, frame_number);
        auto kx = (x + randf(rnd)) * (2.0f / film_width)  - 1.0f;
        auto ky = 1.0f - (y + randf(rnd)) * (2.0f / film_height);
        auto ray_org = cam_eye;
        auto ray_dir = normalize(cam_dir + cam_right * (cam_dim.x * kx) + cam_up * (cam_dim.y * ky));

        RayState ray_state;
        ray_state.depth   = 0;
        ray_state.rnd     = rnd;
        ray_state.contrib = make_float3(1.0f, 1.0f, 1.0f);
        ray_state.color   = make_float3(0.0f, 0.0f, 0.0f);
        ray_state.mis     = 0.0f;
        ray_state.done    = false;

        while (true) {
            optix::Ray ray;
            ray.direction = ray_dir;
            ray.origin    = ray_org;
            ray.tmin      = offset;
            ray.tmax      = FLT_MAX;
            ray.ray_type  = 0;
            rtTrace(top_object, ray, ray_state);

            if (ray_state.done)
                break;

            ray_org = ray_state.next_org;
            ray_dir = ray_state.next_dir;
        }

        result += ray_state.color;
    } while (--samples);

    frame_buffer[y * film_width + x] += result * (1.0f / samples_per_pixel);
}

rtDeclareVariable(RayState, ray_state, rtPayload, );
RT_PROGRAM void closest_hit() {
    auto normal      = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, attr_normal));
    auto face_normal = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, attr_face_normal));
    auto rnd         = ray_state.rnd;
    auto color       = ray_state.color;

    // flip normal as necessary
    bool entering = dot(ray.direction, face_normal) <= 0.0f;
    if (!entering)
        face_normal = -face_normal;
    if (dot(ray.direction, normal) > 0.0f)
        normal = -normal;

    auto& mat = materials[attr_material];
    auto out_dir = -ray.direction;

    // Handle emissive materials
    auto ke = mat.ke;
    if (entering && ke.x != 0 && ke.y != 0 && ke.z != 0) {
        auto index    = attr_index;
        auto v0       = vertex_buffer[index.x];
        auto v1       = vertex_buffer[index.y];
        auto v2       = vertex_buffer[index.z];
        auto pdf_area = 1.0f / triangle_area(v0, v1, v2);
        auto next_mis = ray_state.mis * ray_dist * ray_dist / dot(out_dir, normal);
        auto weight   = 1.0f / (1.0f + next_mis * pdf_lightpick * pdf_area);
        color += ke * ray_state.contrib * weight;
    }

    auto uv = attr_texcoord;
    auto intr_point = ray.origin + ray.direction * ray_dist;

    // Direct illumination is not possible for glass and mirrors
    if (mat.illum != 5 && mat.illum != 7) {
        // Sample a point on a light
        auto& light      = lights[(randi(rnd) & 0x7FFFFFFF) % num_lights];
        auto light_point = sample_triangle(randf(rnd), randf(rnd), light.v0, light.v1, light.v2);
        auto light_dir   = light_point - intr_point;
        auto visibility  = dot(light_dir, normal);
        auto cos_light   = -dot(light.normal, light_dir);

        if (visibility > 0 && cos_light > 0) {
            auto inv_light_dist  = 1.0f / length(light_dir);
            auto inv_light_dist2 = inv_light_dist * inv_light_dist;

            auto in_dir = light_dir * inv_light_dist;

            auto pdf_material  = phong_sample_pdf(mat, normal, uv, out_dir, in_dir);
            auto pdf_light     = light.inv_area * pdf_lightpick;
            auto inv_pdf_light = 1.0f / pdf_light;

            cos_light *= inv_light_dist;
            auto cos_surface = visibility * inv_light_dist;

            auto weight = 1.0f / (1.0f + pdf_material * cos_light * inv_light_dist2 * inv_pdf_light);
            auto geom_factor = cos_surface * cos_light * inv_light_dist2 * inv_pdf_light;

            auto contrib = light.intensity * ray_state.contrib * eval_phong_bsdf(mat, normal, uv, out_dir, in_dir);

            ShadowRayState shadow_ray_state;
            shadow_ray_state.in_shadow = false;
            optix::Ray shadow_ray;
            shadow_ray.origin    = intr_point;
            shadow_ray.direction = light_dir;
            shadow_ray.tmin      = offset;
            shadow_ray.tmax      = 1.0f - offset;
            shadow_ray.ray_type  = 1;
            rtTrace(top_object, shadow_ray, shadow_ray_state);

            if (!shadow_ray_state.in_shadow)
                color += contrib * (geom_factor * weight);
        }
    }

    // Write the new color to the state only once here
    ray_state.color = color;

    // Russian Roulette
    auto rr_prob = russian_roulette(ray_state.contrib, 0.75f);
    if (ray_state.depth >= max_path_depth || randf(rnd) >= rr_prob) {
        ray_state.done = true;
        return;
    }

    MaterialSample sample;
    bool specular = false;
    switch (mat.illum) {
        case 5: // Mirror
            sample = sample_mirror_bsdf(mat, normal, face_normal, out_dir);
            specular = true;
            break;
        case 7: // Glass
            sample = sample_glass_bsdf(entering, mat, normal, face_normal, rnd, out_dir);
            specular = true;
            break;
        default: // Corrected Phong
            sample = sample_phong_bsdf(mat, normal, face_normal, uv, rnd, out_dir);
            break; 
    }

    // Update ray state
    ray_state.depth    ++;
    ray_state.contrib  *= sample.color * (sample.cos / (sample.pdf * rr_prob));
    ray_state.mis      = specular ? 0.0f : 1.0f / sample.pdf;
    ray_state.next_org = intr_point;
    ray_state.next_dir = sample.dir;
    ray_state.rnd      = rnd;
}

rtDeclareVariable(ShadowRayState, shadow_ray_state, rtPayload, );
RT_PROGRAM void shadow() {
    shadow_ray_state.in_shadow = true;
    rtTerminateRay();
}

RT_PROGRAM void exception() {
    auto bad_color = make_float3(1.0f, 0.0f, 1.0f);
    frame_buffer[launch_index.x + launch_index.y * film_width] = bad_color;
}

RT_PROGRAM void miss() {
    ray_state.done = true;
}
