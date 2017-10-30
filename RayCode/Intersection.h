#pragma once

#include "Vector.h"

#ifndef SIMD
void RayAABB(Vec3 rayPos, Vec3 invRayDir, Vec3 rayDirSign, AABB aabb, float& dist);
void RayAABB(Vec3 rayPos, Vec3 invRayDir, Vec3 rayDirSign, AABB aabb, float& dist, Vec3& normal);
void RaySphereIntersection(Vec3 const& rayPos, Vec3 const& rayDir, Vec3 const& pos, float radiusSq, float& dist, Vec3& normal);

#else
void RayAABB(WVec rayPos, WVec invRayDir, WVec rayDirSign, AABB aabb, NVec& dist, WVec& normal);
inline void RayAABB(WVec rayPos, WVec invRayDir, WVec rayDirSign, AABB aabb, NVec& dist)
{
    WVec normal;
    // TODO: RayAABB test that doesn't calculate the normal
    RayAABB(rayPos, invRayDir, rayDirSign, aabb, dist, normal);
}
void RaySphereIntersection(WVec rayPos, WVec rayDir, Vec3 pos, NVec radiusSq, NVec& dist, WVec& normal);
inline void RaySphereIntersection(WVec rayPos, WVec rayDir, Vec3 pos, NVec radiusSq, NVec& dist)
{
    WVec normal;
    RaySphereIntersection(rayPos, rayDir, pos, radiusSq, dist, normal);
}

void RayTriange4(Vec43 rayPos, Vec43 invRayDir, Vec43 rayDirSign, Vec3 triA, Vec3 triB, Vec3 triC, Vec3& dist, Vec43& normal);

inline void RayTriange4(Vec43 rayPos, Vec43 invRayDir, Vec43 rayDirSign, Vec3 triA, Vec3 triB, Vec3 triC, Vec3& dist)
{
    Vec43 normal;
    RayTriange4(rayPos, invRayDir, rayDirSign, triA, triB, triC, dist, normal);
}
#endif