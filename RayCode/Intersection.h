#pragma once

#include "Vector.h"

#ifndef SIMD
void RayAABB(Vec3 rayPos, Vec3 invRayDir, Vec3 rayDirSign, AABB aabb, float& dist);
void RayAABB(Vec3 rayPos, Vec3 invRayDir, Vec3 rayDirSign, AABB aabb, float& dist, Vec3& normal);
void RaySphereIntersection(Vec3 const& rayPos, Vec3 const& rayDir, Vec3 const& pos, float radiusSq, float& dist, Vec3& normal);

#else
void RayAABB(WVec rayPos, WVec invRayDir, WVec rayDirSign, AABB aabb, NVec& dist, WVec& normal);
void RayAABB(WVec rayPos, WVec invRayDir, AABB aabb, NVec& dist);

//void RayDualAABB(WVec rayPos, WVec invRayDir, Vec3 aabbX, Vec3 aabbY, Vec3 aabbZ, NVec& dist1, NVec& dist2);

void RaySphereIntersection(WVec rayPos, WVec rayDir, Vec3 pos, NVec radiusSq, NVec& dist, WVec& normal);
inline void RaySphereIntersection(WVec rayPos, WVec rayDir, Vec3 pos, NVec radiusSq, NVec& dist)
{
    WVec normal;
    RaySphereIntersection(rayPos, rayDir, pos, radiusSq, dist, normal);
}

void RayTriange4(WVec rayPos, WVec rayDir, Vec3 triA, Vec3 triB, Vec3 triC, NVec& dist);
#endif