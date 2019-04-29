#include <embree3/rtcore.isph>

#include "embree_path_tracer.h"
#include "common.isph"

// Renderer ------------------------------------------------------------------------

inline struct rgb_s render_pixel(const struct scene_s* uniform scene, counters_s* uniform counters, int x, int y, int z, uniform size_t iter, uniform int max_path_len) {
    struct rgb_s color = { 0.0f, 0.0f, 0.0f };
    const struct camera_s* uniform cam = scene->camera;

    // generate ray
    uint32_t rnd = fnv_hash(fnv_init(), iter);
    rnd = fnv_hash(rnd, x);
    rnd = fnv_hash(rnd, y);
    rnd = fnv_hash(rnd, z);
    float kx = (x + randf(rnd)) * (2.0f / scene->film_width)  - 1.0f;
    float ky = 1.0f - (y + randf(rnd)) * (2.0f / scene->film_height);
    struct float3_s ray_dir = normalize(cam->dir + cam->right * (cam->w * kx) + cam->up * (cam->h * ky));

    RTCRay shadow_ray;
    shadow_ray.org_x = 0.0f;
    shadow_ray.org_y = 0.0f;
    shadow_ray.org_z = 0.0f;
    shadow_ray.dir_y = 0.0f;
    shadow_ray.dir_y = 1.0f;
    shadow_ray.dir_y = 0.0f;
    shadow_ray.time  = 0.0f;
    shadow_ray.tnear = 0.0f;
    shadow_ray.tfar  = 0.0f;
    shadow_ray.mask  = -1;
    shadow_ray.id    = 0;
    shadow_ray.flags = 0;

    RTCRayHit ray;
    ray.ray.org_x = cam->eye.x;
    ray.ray.org_y = cam->eye.y;
    ray.ray.org_z = cam->eye.z;
    ray.ray.dir_x = ray_dir.x;
    ray.ray.dir_y = ray_dir.y;
    ray.ray.dir_z = ray_dir.z;
    ray.ray.time  = 0.0f;
    ray.ray.tnear = offset;
    ray.ray.tfar  = FLT_MAX;
    ray.ray.mask  = -1;
    ray.ray.id    = 0;
    ray.ray.flags = 0;

    ray.hit.geomID = -1;
    ray.hit.primID = -1;

    float mis = 0.0f;
    struct rgb_s contrib = { 1.0f, 1.0f, 1.0f };
    uniform const float pdf_lightpick = scene->pdf_lightpick;

    uniform fastdiv_t light_div = make_fastdiv(scene->num_lights);

    for (uniform int i = 0; i <= max_path_len; ++i) {
        uniform RTCIntersectContext context;
        context.flags = i == 0 ? RTC_INTERSECT_CONTEXT_FLAG_COHERENT : RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
        context.filter = NULL;
        context.instID[0] = RTC_INVALID_GEOMETRY_ID;

        uniform int64_t t0 = clock_us();
        rtcIntersectV(scene->scene, &context, &ray);
        uniform int64_t t1 = clock_us();
#ifdef ENABLE_TIMING
        counters->total_rays += popcnt(lanemask());
        if (i == 0) counters->primary += t1 - t0;
        else        counters->bounces += t1 - t0;
#endif
        int prim_id = ray.hit.primID;
        if (prim_id < 0)
            break;

        float t = ray.ray.tfar;
        float u = ray.hit.u;
        float v = ray.hit.v;
        float w = 1.0f - u - v;
        int i0 = scene->indices[prim_id * 4 + 0];
        int i1 = scene->indices[prim_id * 4 + 1];
        int i2 = scene->indices[prim_id * 4 + 2];
        int mat_id = scene->indices[prim_id * 4 + 3];

        struct float3_s intr_point = { ray.ray.org_x + ray.ray.dir_x * t, ray.ray.org_y + ray.ray.dir_y * t, ray.ray.org_z + ray.ray.dir_z * t };

        struct float3_s face_normal = { ray.hit.Ng_x, ray.hit.Ng_y, ray.hit.Ng_z };
        struct float3_s normal      = normalize(scene->normals[i0] * w + scene->normals[i1] * u + scene->normals[i2] * v);
        struct float2_s texcoords   = scene->texcoords[i0] * w + scene->texcoords[i1] * u + scene->texcoords[i2] * v;
        struct float3_s out_dir     = { -ray.ray.dir_x, -ray.ray.dir_y, -ray.ray.dir_z };

        // Compute triangle area and normalize face normal
        float inv_area = 1.0f / (0.5f * length(face_normal));
        face_normal = face_normal * 0.5f * inv_area;

        bool entering = dot(out_dir, face_normal) >= 0.0f;
        if (!entering)
            face_normal = negate(face_normal);
        if (dot(out_dir, normal) < 0.0f)
            normal = negate(normal);

        // Handle emissive materials
        const struct material_s* mat = scene->materials + mat_id;
        if (entering && mat->light != 0) {
            float next_mis = mis * t * t / dot(out_dir, normal);
            float weight   = 1.0f / (1.0f + next_mis * pdf_lightpick * inv_area);
            color = color + mat->ke * contrib * weight;
        }

        // Next event estimation
        int illum = mat->illum;
        if (illum != 5 && illum != 7) {
            // Sample a point on a light
            int light_index = (randi(rnd) & 0x7FFFFFFF);
            light_index = light_index - fastdiv(light_div, light_index) * scene->num_lights; // faster than light_index %= scene->num_lights
            const struct light_s* light = scene->lights + light_index;
            struct float3_s light_point = sample_triangle(randf(rnd), randf(rnd), light->v0, light->v1, light->v2);
            struct float3_s light_dir   = light_point - intr_point;
            float visibility  = dot(light_dir, normal);
            float cos_light   = -dot(light->n, light_dir);

            if (visibility > 0 && cos_light > 0) {
                float inv_light_dist  = 1.0f / length(light_dir);
                float inv_light_dist2 = inv_light_dist * inv_light_dist;

                struct float3_s in_dir = light_dir * inv_light_dist;

                float pdf_material  = phong_sample_pdf(scene->images, mat, normal, texcoords, out_dir, in_dir);
                float pdf_light     = light->inv_area * pdf_lightpick;
                float inv_pdf_light = 1.0f / pdf_light;

                cos_light *= inv_light_dist;
                float cos_surface = visibility * inv_light_dist;

                float weight = 1.0f / (1.0f + pdf_material * cos_light * inv_light_dist2 * inv_pdf_light);
                float geom_factor = cos_surface * cos_light * inv_light_dist2 * inv_pdf_light;

                struct rgb_s light_contrib = light->color * contrib * eval_phong_bsdf(scene->images, mat, normal, texcoords, out_dir, in_dir) * (geom_factor * weight);

                shadow_ray.org_x = intr_point.x;
                shadow_ray.org_y = intr_point.y;
                shadow_ray.org_z = intr_point.z;
                shadow_ray.dir_x = light_dir.x;
                shadow_ray.dir_y = light_dir.y;
                shadow_ray.dir_z = light_dir.z;
                shadow_ray.tnear = offset;
                shadow_ray.tfar  = 1.0f - offset;

                uniform RTCIntersectContext shadow_context;
                shadow_context.flags = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
                shadow_context.filter = NULL;
                shadow_context.instID[0] = RTC_INVALID_GEOMETRY_ID;

                uniform int64_t t2 = clock_us();
                rtcOccludedV(scene->scene, &shadow_context, &shadow_ray);
                uniform int64_t t3 = clock_us();
#ifdef ENABLE_TIMING
                counters->shadow += t3 - t2;
                counters->shade  -= t3 - t2;
#endif

                if (shadow_ray.tfar >= 0.0f)
                    color = color + light_contrib;
            }
        }

        // Russian Roulette
        float rr_prob = russian_roulette(contrib, 0.75f);
        if (randf(rnd) >= rr_prob) break;

        // Bounce
        struct sample_s sample;
        bool specular = false;
        switch (illum) {
            case 5: // Mirror
                sample = sample_mirror_bsdf(mat, normal, face_normal, out_dir);
                specular = true;
                break;
            case 7: // Glass
                sample = sample_glass_bsdf(entering, mat, normal, face_normal, rnd, out_dir);
                specular = true;
                break;
            default: // Corrected Phong
                sample = sample_phong_bsdf(scene->images, mat, normal, face_normal, texcoords, rnd, out_dir);
                break;
        }

        // Update ray
        contrib  = contrib * sample.color * (sample.cos / (sample.pdf * rr_prob));
        mis      = specular ? 0.0f : 1.0f / sample.pdf;

        ray.ray.org_x = intr_point.x;
        ray.ray.org_y = intr_point.y;
        ray.ray.org_z = intr_point.z;
        ray.ray.dir_x = sample.dir.x;
        ray.ray.dir_y = sample.dir.y;
        ray.ray.dir_z = sample.dir.z;
        ray.ray.tfar  = FLT_MAX;

        ray.hit.geomID = -1;
        ray.hit.primID = -1;
        uniform int64_t t4 = clock_us();
#ifdef ENABLE_TIMING
        counters->shade += t4 - t1;
#endif
    }

    return color;
}

export void render_tile(const struct scene_s* uniform scene, counters_s* uniform counters, uniform uint32_t xmin, uniform uint32_t ymin, uniform uint32_t xmax, uniform uint32_t ymax, uniform uint32_t iter, uniform uint32_t spp, uniform uint32_t max_path_len) {
    uniform int64_t start = clock_us();
    float* uniform film = scene->film;
    foreach_tiled(i = xmin...xmax, j = ymin...ymax) {
        struct rgb_s color = { 0.0f, 0.0f, 0.0f };
        for (int k = 0; k < spp; k++)
            color = color + render_pixel(scene, counters, i, j, k, iter, max_path_len);
        int index = j * scene->film_width + i;
        uniform float inv = 1.0f / spp;
        uniform int pmax = scene->film_width * scene->film_height;
        for (uniform int lane = 0; lane < programCount; lane++) {
            uniform int p = extract(index, lane);
            if (p < pmax) {
                film[p * 3 + 0] += extract(color.r * inv, lane);
                film[p * 3 + 1] += extract(color.g * inv, lane);
                film[p * 3 + 2] += extract(color.b * inv, lane);
            }
        }
    }
#ifdef ENABLE_TIMING
    counters->total += clock_us() - start;
#endif
}
