// Benchmark.cpp : Defines the entry point for the console application.
//
#include "BVH.h"
#include "Benchmark\Benchmark.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "Core.h"

BENCHMARK_MAIN()

template <u32 leafSize>
class BVHBench : public ::benchmark::Fixture
{
public:
    void SetUp(const ::benchmark::State& st)
    {
        char const* cachePath = "..\\..\\happy\\bvh.cache";

        LoadBVHFromOBJ("..\\..\\happy\\buddha.obj", m_BVH, leafSize);
    }

    BVH::Root m_BVH;
};

void Test(benchmark::State& st, BVH::Root& bvh)
{
    NVec rayLength = NVecMake(100.0f);
    WVec normal;

    int range = 50;

    for (auto _ : st)
    {
        for (int z = 0; z < range; ++z)
        {
            for (int y = 0; y < range; ++y)
            {
                for (int x = 0; x < range; ++x)
                {
                    WVec rayP = WVecMake2(((x / 50.0f) - 1.0f) * 20.0f, ((y / 50.0f) - 1.0f) * 20.0f, ((z / 50.0f) - 1.0f) * 20.0f);
                    WVec rayD = WVecNormalise(-rayP);
                    WVec invRayD = {
                        NVecMake(1.0f) / rayD.x,
                        NVecMake(1.0f) / rayD.y,
                        NVecMake(1.0f) / rayD.z,
                    };
                    WVec invSign = {
                        NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayD.x, NVecMake(0.0f))),
                        NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayD.y, NVecMake(0.0f))),
                        NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayD.z, NVecMake(0.0f))),
                    };

                    bvh.FindClosest(rayP, rayD, invRayD, invSign, rayLength, normal);
                }
            }
        }
    }
}

typedef BVHBench<2> BVHBench2;
typedef BVHBench<5> BVHBench5;
typedef BVHBench<10> BVHBench10;
typedef BVHBench<20> BVHBench20;
typedef BVHBench<50> BVHBench50;
typedef BVHBench<100> BVHBench100;

BENCHMARK_DEFINE_F(BVHBench2, RayTest2) (benchmark::State& st) { Test(st, m_BVH); };
BENCHMARK_DEFINE_F(BVHBench5, RayTest5) (benchmark::State& st) { Test(st, m_BVH); };
BENCHMARK_DEFINE_F(BVHBench10, RayTest10) (benchmark::State& st) { Test(st, m_BVH); };
BENCHMARK_DEFINE_F(BVHBench20, RayTest20) (benchmark::State& st) { Test(st, m_BVH); };
BENCHMARK_DEFINE_F(BVHBench50, RayTest50) (benchmark::State& st) { Test(st, m_BVH); };
BENCHMARK_DEFINE_F(BVHBench100, RayTest100) (benchmark::State& st) { Test(st, m_BVH); };

BENCHMARK_REGISTER_F(BVHBench2, RayTest2)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BVHBench5, RayTest5)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BVHBench10, RayTest10)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BVHBench20, RayTest20)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BVHBench50, RayTest50)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BVHBench100, RayTest100)->Unit(benchmark::kMillisecond);