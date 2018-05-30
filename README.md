# Rodent

Rodent is a BVH traversal library implemented using the AnyDSL compiler framework (https://anydsl.github.io/).

# Building

The dependencies are: CMake, AnyDSL, libpng, SDL2, and optionally the Embree sources for the benchmarking tools.
Once the dependencies are installed, use the following commands to build the project:

    mkdir build
    cd build
    cmake ..
    # Optional: Create benchmarking tools for Embree and BVH extractor tools
    # cmake .. -DEMBREE_ROOT_DIR = <path to Embree sources>
    make
    
The current settings assume a machine with AVX2 or higher. To change this, edit the [source code](tools/bench_traversal/bench_traversal.impala) of the benchmarking tool to only use AVX or SSE, and set the `CLANG_FLAGS` CMake variable to the desired ISA.

# Testing

Test files are provided in the `testing` directory. Use the following commands to test the code:

    cd build/bin
    ./bench_traversal -bvh ../../testing/sponza.bvh -ray ../../testing/sponza-primary.rays -bench 50 -warmup 10 -tmax 5000 -o output-packet-primary.fbuf
    ./bench_traversal -bvh ../../testing/sponza.bvh -ray ../../testing/sponza-primary.rays -bench 50 -warmup 10 -tmax 5000 -s -o output-single-primary.fbuf
    ./bench_traversal -bvh ../../testing/sponza.bvh -ray ../../testing/sponza-random.rays -bench 50 -warmup 10 -tmax 1 -o output-packet-random.fbuf
    ./bench_traversal -bvh ../../testing/sponza.bvh -ray ../../testing/sponza-random.rays -bench 50 -warmup 10 -tmax 1 -s -o output-single-random.fbuf
    ./fbuf2png -n output-packet-primary.fbuf output-packet-primary.png
    ./fbuf2png -n output-single-primary.fbuf output-single-primary.png
    ./fbuf2png -n output-packet-random.fbuf output-packet-random.png
    ./fbuf2png -n output-single-random.fbuf output-single-random.png

This will run the traversal on the test set, and generate images as a result. Given the same ray distribution, the _packet_ and _single_ variants should generate the same images. The reference images for primary and random rays are in the `testing` directory.
