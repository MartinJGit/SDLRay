#ifdef _DEBUG
#include "Core.h"

#include "gtest\gtest.h"
#include "Intersection.h"

#define EXPECT_VECTOR_EQ(a, b) \
    EXPECT_FLOAT_EQ(a.m128_f32[0], b.m128_f32[0]); \
    EXPECT_FLOAT_EQ(a.m128_f32[1], b.m128_f32[1]); \
    EXPECT_FLOAT_EQ(a.m128_f32[2], b.m128_f32[2]);

TEST(RayAABB, RayAABBParallel)
{
    float a = 1.0f;
    float b = 0.0f;

    WVec rayPos = WVecMake2(0.0f, 0.0f, 10.0f);
    WVec invRayDir = WVecMake2(a / b, a / b, 1.0f / -1.0f);
    WVec rayDirSign = WVecMake2(0.0f, 0.0f, 1.0f);

    AABB aabb;
    aabb.m_Min = Vec3Make(0.0f, 0.0f, 0.0f);
    aabb.m_Max = Vec3Make(2.0f, 2.0f, 0.0f);
    
    NVec dist;
    WVec normal;

    RayAABB(rayPos, invRayDir, rayDirSign, aabb, dist, normal);

    EXPECT_VECTOR_EQ(dist, Vec3Make(9.0f, FLT_MAX, FLT_MAX, FLT_MAX));
}

#ifndef SIMD
TEST(RaySphere, Miss)
{
    RaySphereIntersection()
}
#else
TEST(RaySphere, Miss)
{
    WVec rayPos = { NVecMake(0.0f), NVecMake(0.0f), NVecMake(10.0f) };
    WVec rayDir = { NVecMake(0.0f), NVecMake(0.0f), NVecMake(-1.0f) };
    NVec spherePos = NVecMake(0.0f);
    NVec sphereRadiusSq = NVecMake(1.0f);
    NVec dist = NVecMake(FLT_MAX);
    RaySphereIntersection(rayPos, rayDir, spherePos, sphereRadiusSq, dist);

    EXPECT_VECTOR_EQ(dist, NVecMake(9.0f));
}

TEST(RaySphere, Normal)
{
    WVec rayPos = { NVecMake(0.0f), NVecMake(10.0f), NVecMake(0.0f) };
    WVec rayDir = { NVecMake(0.0f), NVecMake(-1.0f), NVecMake(0.0f) };
    NVec spherePos = NVecMake(0.0f);
    NVec sphereRadiusSq = NVecMake(1.0f);
    NVec dist = NVecMake(FLT_MAX);
    WVec normal;
    RaySphereIntersection(rayPos, rayDir, spherePos, sphereRadiusSq, dist, normal);

    EXPECT_VECTOR_EQ(normal.x, NVecMake(0.0f));
    EXPECT_VECTOR_EQ(normal.y, NVecMake(1.0f));
    EXPECT_VECTOR_EQ(normal.z, NVecMake(0.0f));
}

TEST(RayTriangle, Miss)
{
    WVec rayPos = { Vec3Make(0.0f, -5.0f, 5.0f, 0.0f), NVecMake(10.0f), Vec3Make(0.0f, 0.0f, 0.0f, 5.0f) };
    WVec rayDir = { NVecMake(0.0f), NVecMake(-1.0f), NVecMake(0.0f) };
    Vec3 triPosA = Vec3Make(-1.0f, 0.0f, 0.0f);
    Vec3 triPosB = Vec3Make(0.0f, 0.0f, 1.0f);
    Vec3 triPosC = Vec3Make(1.0f, 0.0f, 0.0f);
    NVec dist = NVecMake(FLT_MAX);
    RayTriange4(rayPos, rayDir, triPosA, triPosB, triPosC, dist);

    EXPECT_VECTOR_EQ(dist, Vec3Make(10.0f, FLT_MAX, FLT_MAX, FLT_MAX));
}
#endif

#endif