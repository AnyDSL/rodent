#include <iostream>
#include <cstring>
#include <vector>
#include <numeric>
#include <algorithm>

#include "traversal.h"
#include "load_bvh.h"
#include "load_rays.h"

inline void check_argument(int i, int argc, char** argv) {
    if (i + 1 >= argc) {
        std::cerr << "Missing argument for " << argv[i] << std::endl;
        exit(1);
    }
}

inline void usage() {
    std::cout << "Usage: frontend [options]\n"
                 "Available options:\n"
                 "  -bvh     --bvh-file        Sets the BVH file to use\n"
                 "  -ray     --ray-file        Sets the ray file to use\n"
                 "  -tmin                      Sets the minimum distance along the rays (default: 0)\n"
                 "  -tmax                      Sets the maximum distance along the rays (default: 1e9)\n"
                 "  -bench   --bench-iters     Sets the number of benchmark iterations (default: 1)\n"
                 "  -warmup  --bench-warmup    Sets the number of warmup iterations (default: 0)\n";
}

static double bench_cpu(Bvh4* bvh4, RayAoS* rays, HitAoS* hits, size_t n) {
    auto t0 = anydsl_get_micro_time();
    frontend_cpu_traverse_bvh(bvh4, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

int main(int argc, char** argv) {
    std::string ray_file;
    std::string bvh_file;
    float tmin = 0.0f, tmax = 1e9f;
    int iters = 1;
    int warmup = 0;

    for (int i = 1; i < argc; i++) {
        auto arg = argv[i];
        if (arg[0] == '-') {
            if (!strcmp(arg, "-h") || !strcmp(arg, "--help")) {
                usage();
                return 0;
            } else if (!strcmp(arg, "-bvh") || !strcmp(arg, "--bvh-file")) {
                check_argument(i, argc, argv);
                bvh_file = argv[++i];
            } else if (!strcmp(arg, "-ray") || !strcmp(arg, "--ray-file")) {
                check_argument(i, argc, argv);
                ray_file = argv[++i];
            } else if (!strcmp(arg, "-tmin")) {
                check_argument(i, argc, argv);
                tmin = strtof(argv[++i], nullptr);
            } else if (!strcmp(arg, "-tmax")) {
                check_argument(i, argc, argv);
                tmax = strtof(argv[++i], nullptr);
            } else if (!strcmp(arg, "-bench") || !strcmp(arg, "--bench-iters")) {
                check_argument(i, argc, argv);
                iters = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "-warmup") || !strcmp(arg, "--warmup-iters")) {
                check_argument(i, argc, argv);
                warmup = strtol(argv[++i], nullptr, 10);
            } else {
                std::cerr << "Unknown option '" << arg << "'" << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Invalid argument '" << arg << "'" << std::endl;
            return 1;
        }
    }

    if (bvh_file == "") {
        std::cerr << "No BVH file specified" << std::endl;
        return 1;
    }
    anydsl::Array<Bvh4Node> nodes;
    anydsl::Array<Bvh4Tri>  tris;
    if (!load_bvh(bvh_file, nodes, tris)) {
        std::cerr << "Cannot load BVH file" << std::endl;
        return 1;
    }

    if (ray_file == "") {
        std::cerr << "No ray file specified" << std::endl;
        return 1;
    }
    anydsl::Array<RayAoS> rays;
    if (!load_rays(ray_file, rays, tmin, tmax)) {
        std::cerr << "Cannot load BVH file" << std::endl;
        return 1;
    }

    anydsl::Array<HitAoS> hits(rays.size());
    Bvh4 bvh4 { nodes.data(), tris.data() };

    for (int i = 0; i < warmup; i++) {
        bench_cpu(&bvh4, rays.data(), hits.data(), rays.size());
    }

    std::vector<double> timings;
    for (int i = 0; i < iters; i++) {
        timings.push_back(bench_cpu(&bvh4, rays.data(), hits.data(), rays.size()));
    }

    size_t intr = 0;
    for (int i = 0; i < rays.size(); i++)
        intr += (hits[i].tri_id >= 0);

    std::sort(timings.begin(), timings.end());
    auto sum = std::accumulate(timings.begin(), timings.end(), 0.0);
    auto avg = sum / timings.size();
    auto med = timings[timings.size() / 2];
    auto min = *std::min_element(timings.begin(), timings.end());
    std::cout << intr << " intersection(s)." << std::endl;
    std::cout << sum << "ms for " << iters << " iteration(s)." << std::endl;
    std::cout << rays.size() * iters / (1000.0 * sum) << " Mrays/sec." << std::endl;
    std::cout << "# Average: " << avg << " ms" << std::endl;
    std::cout << "# Median: " << med  << " ms" << std::endl;
    std::cout << "# Min: " << min << " ms" << std::endl;
    return 0;
}
