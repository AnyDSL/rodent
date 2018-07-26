# Rodent

Rodent is a BVH traversal library and renderer implemented using the AnyDSL compiler framework (https://anydsl.github.io/).

# Building

The dependencies are: CMake, AnyDSL (`llvm_60` branch), libpng, SDL2, and optionally the Embree sources for the benchmarking tools.
Once the dependencies are installed, use the following commands to build the project:

    mkdir build
    cd build
    cmake ..
    # Optional: Create benchmarking tools for Embree and BVH extractor tools
    # cmake .. -DEMBREE_ROOT_DIR = <path to Embree sources>
    make

The current settings assume a machine with AVX2 or higher. To change this, follow these steps:

- Edit the source code of [Thorin](https://github.com/AnyDSL/thorin/blob/master/src/thorin/be/llvm/vectorize.cpp#L150), and recompile it.
- Edit the [source code](tools/bench_traversal/bench_traversal.impala) of the benchmarking tool. Initially, all variants will be generated, but depending on available ISAs, some might perform poorly. Here is a table of the recommended settings for each ISA.

|   ISA   |      Recommended settings      |
|---------|--------------------------------|
|  AVX2   |           *default*            |
|  AVX    | disable cpu_int_min_max        |
|  SSE2   | disable ray8 and bvh8 variants |

- Set the `CLANG_FLAGS` CMake variable to the desired ISA.
- Recompile Rodent

# Testing

Test files are provided in the `testing` directory. Use the following commands to test the code:

    cd build/bin
    ./bench_traversal -bvh ../../testing/sponza.bvh -ray ../../testing/sponza-primary.rays --bench 50 --warmup 10 --tmax 5000 -o output-hybrid-primary.fbuf
    ./bench_traversal -bvh ../../testing/sponza.bvh -ray ../../testing/sponza-primary.rays --bench 50 --warmup 10 --tmax 5000 -s -o output-single-primary.fbuf
    ./bench_traversal -bvh ../../testing/sponza.bvh -ray ../../testing/sponza-random.rays --bench 50 --warmup 10 --tmax 1 -o output-hybrid-random.fbuf
    ./bench_traversal -bvh ../../testing/sponza.bvh -ray ../../testing/sponza-random.rays --bench 50 --warmup 10 --tmax 1 -s -o output-single-random.fbuf
    ./fbuf2png -n output-hybrid-primary.fbuf output-hybrid-primary.png
    ./fbuf2png -n output-single-primary.fbuf output-single-primary.png
    ./fbuf2png -n output-hybrid-random.fbuf output-hybrid-random.png
    ./fbuf2png -n output-single-random.fbuf output-single-random.png

This will run the traversal on the test set, and generate images as a result. For the primary ray distribution, the _hybrid_ and _single_ variants should generate the same images. The reference images for primary and random rays are in the `testing` directory.

Running the tool with the flag `--help` will provide a list of additional options.
