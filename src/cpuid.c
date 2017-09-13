#include <stdio.h>

int main(int argc, char** argv) {
    int exit = 1;

    #ifdef __ARM_NEON_FP
    printf("neon "); exit = 0;
    #endif

    if (__builtin_cpu_supports("sse4.2")) { printf("sse4.2 "); exit = 0; }
    if (__builtin_cpu_supports("avx"))    { printf("avx ");    exit = 0; }
    if (__builtin_cpu_supports("avx2"))   { printf("avx2 ");   exit = 0; }

    return exit;
}
