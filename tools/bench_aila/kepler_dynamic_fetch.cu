/*
 *  Copyright (c) 2009-2011, NVIDIA Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of NVIDIA Corporation nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
    GK104-optimized variant of the "Persistent speculative
    while-while" kernel used in:

    "Understanding the Efficiency of Ray Traversal on GPUs",
    Timo Aila and Samuli Laine,
    Proc. High-Performance Graphics 2009

    This variant fetches new work dynamically as soon as the
    warp occupancy drops below a pre-determined threshold.
*/

#include "CudaTracerKernels.hpp"

//------------------------------------------------------------------------

#define STACK_SIZE              64          // Size of the traversal stack in local memory.
#define DYNAMIC_FETCH_THRESHOLD 20          // If fewer than this active, fetch new rays

static __device__ int g_warpCounter;    // Work counter for persistent threads.

// Old Config ------------------------------------------------------------------------
/*
extern "C" __global__ void queryConfig(void)
{
    g_config.bvhLayout = BVHLayout_Compact2;
    g_config.blockWidth = 32;
    g_config.blockHeight = 4;
    g_config.usePersistentThreads = 1;
}*/

// New Config ------------------------------------------------------------------------

static constexpr int g_blockWidth  = 32;
static constexpr int g_blockHeight = 4;

__forceinline__ __device__ float prodsign(float x, float y) {
    return __int_as_float(__float_as_int(x) ^ (__float_as_int(y) & 0x80000000));
}

//------------------------------------------------------------------------

TRACE_FUNC
{
    // Traversal stack in CUDA thread-local memory.

    int traversalStack[STACK_SIZE];
    traversalStack[0] = EntrypointSentinel; // Bottom-most entry.

    // Live state during traversal, stored in registers.

    float   origx, origy, origz;            // Ray origin.
    char*   stackPtr;                       // Current position in traversal stack.
    int     leafAddr;                       // First postponed leaf, non-negative if none.
    //int     leafAddr2;                      // Second postponed leaf, non-negative if none.
    int     nodeAddr = EntrypointSentinel;  // Non-negative: current internal node, negative: second postponed leaf.
    int     hitIndex;                       // Triangle index of the closest intersection, -1 if none.
    float   hitT;                           // t-value of the closest intersection.
    float   hitU;
    float   hitV;
    float   tmin;
    int     rayidx;
    float   oodx;
    float   oody;
    float   oodz;
    float   dirx;
    float   diry;
    float   dirz;
    float   idirx;
    float   idiry;
    float   idirz;

    static constexpr unsigned all_mask = unsigned(-1);

    // Initialize persistent threads.

    __shared__ volatile int nextRayArray[MaxBlockHeight]; // Current ray index in global buffer.

    // Persistent threads: fetch and process rays in a loop.

    do
    {
        const int tidx = threadIdx.x;
        volatile int& rayBase = nextRayArray[threadIdx.y];

        // Fetch new rays from the global pool using lane 0.

        const bool          terminated     = nodeAddr==EntrypointSentinel;
        const unsigned int  maskTerminated = __ballot_sync(all_mask, terminated);
        const int           numTerminated  = __popc(maskTerminated);
        const int           idxTerminated  = __popc(maskTerminated & ((1u<<tidx)-1));

        if(terminated)
        {
            if (idxTerminated == 0)
                rayBase = atomicAdd(&g_warpCounter, numTerminated);

            rayidx = rayBase + idxTerminated;
            if (rayidx >= numRays)
                break;

            // Fetch ray.

            float4 o = FETCH_GLOBAL(rays, rayidx * 2 + 0, float4);
            float4 d = FETCH_GLOBAL(rays, rayidx * 2 + 1, float4);
            origx = o.x;
            origy = o.y;
            origz = o.z;
            tmin  = o.w;
            dirx  = d.x;
            diry  = d.y;
            dirz  = d.z;
            hitT  = d.w;
            hitU = hitV = 0.0f;
            float ooeps = exp2f(-80.0f); // Avoid div by zero.
            idirx = 1.0f / (fabsf(d.x) > ooeps ? d.x : copysignf(ooeps, d.x));
            idiry = 1.0f / (fabsf(d.y) > ooeps ? d.y : copysignf(ooeps, d.y));
            idirz = 1.0f / (fabsf(d.z) > ooeps ? d.z : copysignf(ooeps, d.z));
            oodx  = origx * idirx;
            oody  = origy * idiry;
            oodz  = origz * idirz;

            // Setup traversal.

            stackPtr = (char*)&traversalStack[0];
            leafAddr = 0;   // No postponed leaf.
            //leafAddr2= 0;   // No postponed leaf.
            nodeAddr = 0;   // Start from the root.
            hitIndex = -1;  // No triangle intersected so far.
        }

        // Traversal loop.

        while(nodeAddr != EntrypointSentinel)
        {
            // Traverse internal nodes until all SIMD lanes have found a leaf.

//          while (nodeAddr >= 0 && nodeAddr != EntrypointSentinel)
            while (uint(nodeAddr) < uint(EntrypointSentinel))   // functionally equivalent, but faster
            {
                // Fetch AABBs of the two child nodes.
                const float4 n0xy = tex1Dfetch(t_nodesA, nodeAddr + 0); // (c0.lo.x, c0.hi.x, c0.lo.y, c0.hi.y)
                const float4 n1xy = tex1Dfetch(t_nodesA, nodeAddr + 1); // (c1.lo.x, c1.hi.x, c1.lo.y, c1.hi.y)
                const float4 nz   = tex1Dfetch(t_nodesA, nodeAddr + 2); // (c0.lo.z, c0.hi.z, c1.lo.z, c1.hi.z)
                      float4 tmp  = tex1Dfetch(t_nodesA, nodeAddr + 3); // child_index0, child_index1
                      int2  cnodes= *(int2*)&tmp;

                // Intersect the ray against the child nodes.

                const float c0lox = n0xy.x * idirx - oodx;
                const float c0hix = n0xy.y * idirx - oodx;
                const float c0loy = n0xy.z * idiry - oody;
                const float c0hiy = n0xy.w * idiry - oody;
                const float c0loz = nz.x   * idirz - oodz;
                const float c0hiz = nz.y   * idirz - oodz;
                const float c1loz = nz.z   * idirz - oodz;
                const float c1hiz = nz.w   * idirz - oodz;
                const float c0min = spanBeginKepler(c0lox, c0hix, c0loy, c0hiy, c0loz, c0hiz, tmin);
                const float c0max = spanEndKepler  (c0lox, c0hix, c0loy, c0hiy, c0loz, c0hiz, hitT);
                const float c1lox = n1xy.x * idirx - oodx;
                const float c1hix = n1xy.y * idirx - oodx;
                const float c1loy = n1xy.z * idiry - oody;
                const float c1hiy = n1xy.w * idiry - oody;
                const float c1min = spanBeginKepler(c1lox, c1hix, c1loy, c1hiy, c1loz, c1hiz, tmin);
                const float c1max = spanEndKepler  (c1lox, c1hix, c1loy, c1hiy, c1loz, c1hiz, hitT);

                bool swp = (c1min < c0min);

                bool traverseChild0 = (c0max >= c0min);
                bool traverseChild1 = (c1max >= c1min);

                // Neither child was intersected => pop stack.

                if (!traverseChild0 && !traverseChild1)
                {
                    nodeAddr = *(int*)stackPtr;
                    stackPtr -= 4;
                }

                // Otherwise => fetch child pointers.

                else
                {
                    nodeAddr = (traverseChild0) ? cnodes.x : cnodes.y;

                    // Both children were intersected => push the farther one.

                    if (traverseChild0 && traverseChild1)
                    {
                        if (swp)
                            swap(nodeAddr, cnodes.y);
                        stackPtr += 4;
                        *(int*)stackPtr = cnodes.y;
                    }
                }

                // First leaf => postpone and continue traversal.

                if (nodeAddr < 0 && leafAddr  >= 0)     // Postpone max 1
//              if (nodeAddr < 0 && leafAddr2 >= 0)     // Postpone max 2
                {
                    //leafAddr2= leafAddr;          // postpone 2
                    leafAddr = nodeAddr;
                    nodeAddr = *(int*)stackPtr;
                    stackPtr -= 4;
                }

                // All SIMD lanes have found a leaf? => process them.

                // NOTE: inline PTX implementation of "if(!__any(leafAddr >= 0)) break;".
                // tried everything with CUDA 4.2 but always got several redundant instructions.

                unsigned int mask;
                asm("{\n"
                    "   .reg .pred p;               \n"
                    "setp.ge.s32        p, %1, 0;   \n"
                    "vote.sync.ballot.b32    %0,p,0xffffffff;       \n"
                    "}"
                    : "=r"(mask)
                    : "r"(leafAddr));
                if(!mask)
                    break;

                //if(!__any(leafAddr >= 0))
                //    break;
            }

            // Process postponed leaf nodes.

            while (leafAddr < 0)
            {
                for (int triAddr = ~leafAddr;; triAddr += 3)
                {
                    // Original Woop Intersection ---------------------------------------------------------------
                    /*
                    // Tris in TEX (good to fetch as a single batch)
                    const float4 v00 = tex1Dfetch(t_trisA, triAddr + 0);
                    const float4 v11 = tex1Dfetch(t_trisA, triAddr + 1);
                    const float4 v22 = tex1Dfetch(t_trisA, triAddr + 2);

                    // End marker (negative zero) => all triangles processed.
                    if (__float_as_int(v00.x) == 0x80000000)
                        break;

                    float Oz = v00.w - origx*v00.x - origy*v00.y - origz*v00.z;
                    float invDz = 1.0f / (dirx*v00.x + diry*v00.y + dirz*v00.z);
                    float t = Oz * invDz;

                    if (t > tmin && t < hitT)
                    {
                        // Compute and check barycentric u.

                        float Ox = v11.w + origx*v11.x + origy*v11.y + origz*v11.z;
                        float Dx = dirx*v11.x + diry*v11.y + dirz*v11.z;
                        float u = Ox + t*Dx;

                        if (u >= 0.0f)
                        {
                            // Compute and check barycentric v.

                            float Oy = v22.w + origx*v22.x + origy*v22.y + origz*v22.z;
                            float Dy = dirx*v22.x + diry*v22.y + dirz*v22.z;
                            float v = Oy + t*Dy;

                            if (v >= 0.0f && u + v <= 1.0f)
                            {
                                // Record intersection.
                                // Closest intersection not required => terminate.

                                hitT = t;
                                hitIndex = triAddr;
                                if (anyHit)
                                {
                                    nodeAddr = EntrypointSentinel;
                                    break;
                                }
                            }
                        }
                    }*/
                    // Moeller Trumbore Intersection ----------------------------------------------------------------
                    const float4 v0 = tex1Dfetch(t_trisA, triAddr + 0);
                    const float4 e1 = tex1Dfetch(t_trisA, triAddr + 1);
                    const float4 e2 = tex1Dfetch(t_trisA, triAddr + 2);

                    float nx = e1.y * e2.z - e1.z * e2.y;
                    float ny = e1.z * e2.x - e1.x * e2.z;
                    float nz = e1.x * e2.y - e1.y * e2.x;
                    float cx = v0.x - origx;
                    float cy = v0.y - origy;
                    float cz = v0.z - origz;
                    float rx = diry * cz - dirz * cy;
                    float ry = dirz * cx - dirx * cz;
                    float rz = dirx * cy - diry * cx;
                    float det = nx * dirx + ny * diry + nz * dirz;
                    float abs_det = det < 0 ? -det : det;

                    float u = prodsign(rx * e2.x + ry * e2.y + rz * e2.z, det);
                    float v = prodsign(rx * e1.x + ry * e1.y + rz * e1.z, det);

                    if (u >= 0.0f && v >= 0.0f && abs_det >= u + v && abs_det != 0.0f) {
                        float t = prodsign(cx * nx + cy * ny + cz * nz, det);
                        if (t >= abs_det * tmin && t <= abs_det * hitT) {
                            float inv_det = 1.0f / abs_det;
                            hitT = t * inv_det;
                            hitU = u * inv_det;
                            hitV = v * inv_det;
                            hitIndex = triAddr;
                            if (anyHit) {
                                nodeAddr = EntrypointSentinel;
                                break;
                            }
                        }
                    }

                    if (__float_as_int(e2.w) & 0x80000000)
                        break;
                } // triangle

                // Another leaf was postponed => process it as well.

//              if(leafAddr2<0) { leafAddr = leafAddr2; leafAddr2=0; } else     // postpone2
                {
                    leafAddr = nodeAddr;
                    if (nodeAddr < 0)
                    {
                        nodeAddr = *(int*)stackPtr;
                        stackPtr -= 4;
                    }
                }
            } // leaf

            // DYNAMIC FETCH

            if( __popc(__ballot_sync(all_mask, true)) < DYNAMIC_FETCH_THRESHOLD )
                break;

        } // traversal

        // Remap intersected triangle index, and store the result.

        if (hitIndex == -1) { STORE_RESULT(rayidx, -1, hitT, 0, 0); }
        else                { STORE_RESULT(rayidx, __int_as_float(FETCH_TEXTURE(triIndices, hitIndex, int)), hitT, hitU, hitV); }
    } while(true);
}

// Kernel Launch ------------------------------------------------------------------------

#include <fstream>
#include <iostream>
#include <vector>
#include <cassert>

#include "traversal.h"

#define CHECK_CUDA_CALL(x) check_cuda_call(x, __FILE__, __LINE__)

__host__ static void check_cuda_call(cudaError_t err, const char* file, int line) {
    if (err != cudaSuccess) {
        std::cerr << file << "(" << line << "): " << cudaGetErrorString(err) << std::endl;
        abort();
    }
}

static float4* cuda_nodes = nullptr;
static float4* cuda_tris  = nullptr;
static int*    cuda_ids   = nullptr;

void setup_traversal(const Node2* nodes, size_t num_nodes, const Tri1* tris, size_t num_tris) {
    assert(sizeof(Node2) == sizeof(float4) * 4);
    assert(sizeof(Tri1)  == sizeof(float4) * 3);
    assert(!cuda_nodes && !cuda_tris && !cuda_ids);

    CHECK_CUDA_CALL(cudaMalloc(&cuda_nodes, sizeof(float4) * 4 * num_nodes));
    CHECK_CUDA_CALL(cudaMalloc(&cuda_tris,  sizeof(float4) * 3 * num_tris));
    CHECK_CUDA_CALL(cudaMalloc(&cuda_ids,   sizeof(int)    * 3 * num_tris));

    std::vector<int> ids(num_tris * 3);
    for (int i = 0; i < num_tris; i++) {
        ids[3 * i + 0] = tris[i].prim_id;
        ids[3 * i + 1] = tris[i].prim_id;
        ids[3 * i + 2] = tris[i].prim_id;
    }
    CHECK_CUDA_CALL(cudaMemcpy(cuda_ids, ids.data(), sizeof(int) * 3 * num_tris, cudaMemcpyHostToDevice));

    std::vector<float4> nodes_aila(num_nodes * 4);
    for (int i = 0; i < num_nodes; i++) {
        const auto& node = nodes[i];
        nodes_aila[i * 4 + 0] = make_float4(node.bounds[0], node.bounds[1], node.bounds[ 2], node.bounds[ 3]);
        nodes_aila[i * 4 + 1] = make_float4(node.bounds[4], node.bounds[5], node.bounds[ 6], node.bounds[ 7]);
        nodes_aila[i * 4 + 2] = make_float4(node.bounds[8], node.bounds[9], node.bounds[10], node.bounds[11]);
        // indexing is done on float4, not on nodes/tris
        union { int i; float f; } left { .i = node.child[0] < 0 ? ~((~node.child[0]) * 3) : (node.child[0] - 1) * 4 };
        union { int i; float f; } right{ .i = node.child[1] < 0 ? ~((~node.child[1]) * 3) : (node.child[1] - 1) * 4 };
        nodes_aila[i * 4 + 3] = make_float4(left.f, right.f, 0, 0);
    }
    CHECK_CUDA_CALL(cudaMemcpy(cuda_nodes, nodes_aila.data(), sizeof(float4) * 4 * num_nodes, cudaMemcpyHostToDevice));
    
    CHECK_CUDA_CALL(cudaMemcpy(cuda_tris, tris, sizeof(float4) * 3 * num_tris, cudaMemcpyHostToDevice));

    CHECK_CUDA_CALL(cudaBindTexture(nullptr, t_nodesA,     cuda_nodes, sizeof(float4) * 4 * num_nodes));
    CHECK_CUDA_CALL(cudaBindTexture(nullptr, t_trisA,      cuda_tris,  sizeof(float4) * 3 * num_tris ));
    CHECK_CUDA_CALL(cudaBindTexture(nullptr, t_triIndices, cuda_ids,   sizeof(int)    * 3 * num_tris ));
}

void shutdown_traversal() {
    CHECK_CUDA_CALL(cudaUnbindTexture(t_nodesA));
    CHECK_CUDA_CALL(cudaUnbindTexture(t_trisA));
    CHECK_CUDA_CALL(cudaUnbindTexture(t_triIndices));
    CHECK_CUDA_CALL(cudaFree(cuda_tris));
    CHECK_CUDA_CALL(cudaFree(cuda_ids));
    CHECK_CUDA_CALL(cudaFree(cuda_nodes));
}

void bench_traversal(const Ray1* rays, Hit1* hits, int num_rays, double* timings, int ntimes, bool any) {
    assert(sizeof(Ray1) == sizeof(float4) * 2);
    assert(sizeof(Hit1) == sizeof(int4));

    float4* cuda_rays;
    int4*   cuda_hits;
    CHECK_CUDA_CALL(cudaMalloc(&cuda_rays, sizeof(float4) * 2 * num_rays));
    CHECK_CUDA_CALL(cudaMalloc(&cuda_hits, sizeof(int4) * num_rays));
    CHECK_CUDA_CALL(cudaMemcpy(cuda_rays, rays, sizeof(float4) * 2 * num_rays, cudaMemcpyHostToDevice));

    CHECK_CUDA_CALL(cudaDeviceSynchronize());

    int num_blocks = 0;
    CHECK_CUDA_CALL(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&num_blocks, trace, g_blockWidth * g_blockHeight, MaxBlockHeight * sizeof(int)));
    int dev = 0;
    CHECK_CUDA_CALL(cudaGetDevice(&dev));
    cudaDeviceProp props;
    CHECK_CUDA_CALL(cudaGetDeviceProperties(&props, dev));

    num_blocks *= props.multiProcessorCount;

    cudaEvent_t start, end;
    CHECK_CUDA_CALL(cudaEventCreate(&start));
    CHECK_CUDA_CALL(cudaEventCreate(&end));

    dim3 grid(num_blocks, 1, 1);
    dim3 block(g_blockWidth, g_blockHeight, 1);
    for (int i = 0; i < ntimes; i++) {
        // Here, we assume that the memcpy can be hidden in a real world application
        // Therefore, it is not included in the performance measurement
        int zero = 0;
        cudaMemcpyToSymbol(g_warpCounter, &zero, sizeof(int));

        CHECK_CUDA_CALL(cudaEventRecord(start));
        trace<<<grid, block>>>(
            num_rays,
            any,
            cuda_rays,
            cuda_hits,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr
        );
        CHECK_CUDA_CALL(cudaEventRecord(end));
        CHECK_CUDA_CALL(cudaEventSynchronize(end));

        float ms;
        CHECK_CUDA_CALL(cudaEventElapsedTime(&ms, start, end));
        if (timings) timings[i] = ms;
    }

    CHECK_CUDA_CALL(cudaEventDestroy(start));
    CHECK_CUDA_CALL(cudaEventDestroy(end));

    CHECK_CUDA_CALL(cudaMemcpy(hits, cuda_hits, sizeof(int4) * num_rays, cudaMemcpyDeviceToHost));

    CHECK_CUDA_CALL(cudaFree(cuda_rays));
    CHECK_CUDA_CALL(cudaFree(cuda_hits));
} 
