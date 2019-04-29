#include <embree3/rtcore.isph>

// Comment to use ISPC's default power function
#define pow(x, y) fastpow(x, y)

#include "embree_path_tracer.h"
#include "bench_shading.h"
#include "common.isph"

// Uniform variants ------------------------------------------------------------------

inline float uniform phong_interp(struct rgb_s uniform kd, struct rgb_s uniform ks) {
    float uniform lum_ks = luminance(ks);
    float uniform lum_kd = luminance(kd);
    return lum_ks + lum_kd == 0 ? 0.0f : lum_ks / (lum_ks + lum_kd);
}

inline struct rgb_s lookup_texture(const struct image_s uniform img, struct float2_s uv) {
    uv.x = uv.x - floor(uv.x);
    uv.y = uv.y - floor(uv.y);
    float u = uv.x * (int32_t)img.width;
    float v = uv.y * (int32_t)img.height;
    int x0 = min((int32_t)u, (int32_t)img.width  - 1);
    int y0 = min((int32_t)v, (int32_t)img.height - 1);
    int x1 = min(x0 + 1, (int32_t)img.width  - 1);
    int y1 = min(y0 + 1, (int32_t)img.height - 1);
    float kx = u - (int32_t)u;
    float ky = v - (int32_t)v;
    struct rgb_s p00 = rgba32_to_rgb(img.pixels[x0 + y0 * img.width]);
    struct rgb_s p10 = rgba32_to_rgb(img.pixels[x1 + y0 * img.width]);
    struct rgb_s p01 = rgba32_to_rgb(img.pixels[x0 + y1 * img.width]);
    struct rgb_s p11 = rgba32_to_rgb(img.pixels[x1 + y1 * img.width]);
    return lerp(lerp(p00, p10, kx), lerp(p01, p11, kx), ky);
}

inline float specular_sample_pdf(float uniform ns, struct float3_s normal, struct float3_s out_dir, struct float3_s in_dir) {
    float cos = max(dot(in_dir, reflect(out_dir, normal)), 0.0f);
    return pow(cos, ns) * (ns + 1.0f) * (1.0f / (2.0f * PI));
}

inline struct rgb_s eval_specular_bsdf(struct rgb_s uniform ks, float uniform ns, struct float3_s normal, struct float3_s out_dir, struct float3_s in_dir) {
    float cos = max(dot(in_dir, reflect(out_dir, normal)), 0.0f);
    return ks * pow(cos, ns) * (ns + 2.0f) * (1.0f / (2.0f * PI));
}

inline struct rgb_s uniform eval_diffuse_bsdf(struct rgb_s uniform kd) {
    return kd * (1.0f / PI);
}

inline struct sample_s sample_specular_bsdf(struct rgb_s uniform ks, float uniform ns, struct float3_s normal, struct float3_s face_normal, uint32_t& rnd, struct float3_s out_dir) {
    // Cosine-power hemisphere sampling
    float u = randf(rnd);
    float v = randf(rnd);
    struct float3_s reflect_out = reflect(out_dir, normal);
    float cos_ = pow(v, 1.0f / (ns + 1.0f));
    float sin_ = sqrt(1.0f - cos_ * cos_);
    float phi = 2.0f * PI * u;

    float lobe = pow(cos_, ns) * (1.0f / (2.0f * PI));

    struct float3_s tangent, bitangent;
    gen_local_coords(reflect_out, tangent, bitangent);

    struct float3_s in_dir = tangent * (sin_ * cos(phi)) + bitangent * (sin_ * sin(phi)) + reflect_out * cos_;
    struct rgb_s color     = (ks * (ns + 2.0f)) * lobe;
    float pdf              = lobe * (ns + 1.0f);
    float in_dir_cos       = max(dot(in_dir, normal), 0.0f);
    return make_sample(face_normal, in_dir, pdf, in_dir_cos, color, false);
}

inline struct sample_s sample_diffuse_bsdf(struct rgb_s uniform kd, struct float3_s normal, struct float3_s face_normal, uint32_t& rnd, struct float3_s out_dir) {
    // Cosine hemisphere sampling
    float u = randf(rnd);
    float v = randf(rnd);
    float cos_ = sqrt(1.0f - v);
    float sin_ = sqrt(v);
    float phi = 2.0f * PI * u;

    struct float3_s tangent, bitangent;
    gen_local_coords(normal, tangent, bitangent);

    struct float3_s in_dir     = tangent * (sin_ * cos(phi)) + bitangent * (sin_ * sin(phi)) + normal * cos_;
    struct rgb_s uniform color = kd * (1.0f / PI);
    float pdf                  = cos_ * (1.0f / PI);
    return make_sample(face_normal, in_dir, pdf, cos_, color, false);
}

// Material parameters -------------------------------------------------------------

#define KD_CST kd = make_rgb(0.0f, 1.0f, 0.0f);
#define KD_TEX kd = lookup_texture(*image, uv);

#define KS_CST_NS_CST ks = make_rgb(0.0f, 1.0f, 0.0f); ns = 96.0f;
#define KS_TEX_NS_CST ks = lookup_texture(*image, uv); ns = 12.0f;

#define PARAMS_VARYING \
    struct rgb_s varying kd; \
    struct rgb_s varying ks; \
    float varying ns; \
    if (geom_id & 1 == 0) { \
        KD_CST \
    } else { \
        KD_TEX \
    } \
    if (geom_id & 2 == 0) { \
        KS_CST_NS_CST \
    } else { \
        KS_TEX_NS_CST \
    }

#define PARAMS_0_UNIFORM \
    struct rgb_s uniform kd; \
    struct rgb_s uniform ks; \
    float uniform ns; \
    KD_CST \
    KS_CST_NS_CST

#define PARAMS_1_UNIFORM \
    struct rgb_s varying kd; \
    struct rgb_s uniform ks; \
    float uniform ns; \
    KD_TEX \
    KS_CST_NS_CST

#define PARAMS_2_UNIFORM \
    struct rgb_s uniform kd; \
    struct rgb_s varying ks; \
    float varying ns; \
    KD_CST \
    KS_TEX_NS_CST

#define PARAMS_3_UNIFORM \
    struct rgb_s varying kd; \
    struct rgb_s varying ks; \
    float varying ns; \
    KD_TEX \
    KS_TEX_NS_CST

// Shader --------------------------------------------------------------------------

#define SHADER(PARAMS, i) \
    struct float3_s org = make_float3(stream_in->org_x[i], stream_in->org_y[i], stream_in->org_z[i]); \
    struct float3_s dir = make_float3(stream_in->dir_x[i], stream_in->dir_y[i], stream_in->dir_z[i]); \
    struct rgb_s contrib = make_rgb(stream_in->contrib_r[i], stream_in->contrib_g[i], stream_in->contrib_b[i]); \
    int32_t prim_id = stream_in->prim_id[i]; \
    uint32_t rnd = stream_in->rnd[i]; \
    int32_t depth = stream_in->depth[i]; \
    float t = stream_in->t[i]; \
    float u = stream_in->u[i]; \
    float v = stream_in->v[i]; \
 \
    struct float3_s point = org + dir * t; \
 \
    int32_t i0 = indices[prim_id * 4 + 0]; \
    int32_t i1 = indices[prim_id * 4 + 1]; \
    int32_t i2 = indices[prim_id * 4 + 2]; \
    struct float3_s normal = normals[i0]   * (1.0f - u - v) + normals[i1]   * u + normals[i2]   * v; \
    struct float2_s uv     = texcoords[i0] * (1.0f - u - v) + texcoords[i1] * u + texcoords[i2] * v; \
    struct float3_s face_normal = face_normals[prim_id]; \
 \
    PARAMS \
    float k = phong_interp(kd, ks); \
    bool use_kd = randf(rnd) >= k; \
    struct float3_s out_dir = make_float3(-dir.x, -dir.y, -dir.z); \
 \
    struct sample_s sample; \
    if (use_kd) { \
        sample = sample_diffuse_bsdf(kd, normal, face_normal, rnd, out_dir); \
        sample.color = lerp(sample.color, eval_specular_bsdf(ks, ns, normal, out_dir, sample.dir), k); \
        sample.pdf   = lerp(sample.pdf,   specular_sample_pdf(ns, normal, out_dir, sample.dir),    k); \
    } else { \
        sample = sample_specular_bsdf(ks, ns, normal, face_normal, rnd, out_dir); \
        sample.color = lerp(eval_diffuse_bsdf(kd),                  sample.color, k); \
        sample.pdf   = lerp(diffuse_sample_pdf(normal, sample.dir), sample.pdf,   k); \
    } \
    contrib = contrib * sample.color * (sample.cos / sample.pdf); \
 \
    stream_out->org_x[i] = point.x; \
    stream_out->org_y[i] = point.y; \
    stream_out->org_z[i] = point.z; \
    stream_out->dir_x[i] = sample.dir.x; \
    stream_out->dir_y[i] = sample.dir.y; \
    stream_out->dir_z[i] = sample.dir.z; \
    stream_out->tmin[i] = offset; \
    stream_out->tmax[i] = FLT_MAX; \
    stream_out->depth[i] = depth + 1; \
    stream_out->contrib_r[i] = contrib.r; \
    stream_out->contrib_g[i] = contrib.g; \
    stream_out->contrib_b[i] = contrib.b; \
    stream_out->rnd[i] = rnd; \
    stream_out->mis[i] = 1.0f / sample.pdf;

// Shading benchmarks --------------------------------------------------------------

export void shade_varying(struct stream_s* uniform stream_in,
                          struct stream_s* uniform stream_out,
                          struct float3_s* uniform normals,
                          struct float3_s* uniform face_normals,
                          struct float2_s* uniform texcoords,
                          int32_t* uniform indices,
                          struct image_s* uniform image,
                          int32_t uniform num_rays,
                          int32_t uniform num_iters) {
    for (uniform int iter = 0; iter < num_iters; ++iter) {
        foreach (i = 0 ... num_rays) { \
            int32_t geom_id = stream_in->geom_id[i];
            SHADER(PARAMS_VARYING, i)
        }
    }
}

export void shade_uniform(struct stream_s* uniform stream_in,
                          struct stream_s* uniform stream_out,
                          struct float3_s* uniform normals,
                          struct float3_s* uniform face_normals,
                          struct float2_s* uniform texcoords,
                          int32_t* uniform indices,
                          struct image_s* uniform image,
                          int32_t* uniform begins,
                          int32_t* uniform ends,
                          int32_t uniform num_iters) {
    for (uniform int iter = 0; iter < num_iters; ++iter) {
#define SHADE_GEOM(PARAMS, id) \
        foreach (i = begins[id] ... ends[id]) { \
            SHADER(PARAMS, i) \
        }

        SHADE_GEOM(PARAMS_0_UNIFORM, 0)
        SHADE_GEOM(PARAMS_1_UNIFORM, 1)
        SHADE_GEOM(PARAMS_2_UNIFORM, 2)
        SHADE_GEOM(PARAMS_3_UNIFORM, 3)
    }
}
