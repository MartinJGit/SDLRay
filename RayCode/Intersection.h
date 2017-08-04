#pragma once

#include "Vector.h"

#ifndef SIMD
void RayAABB(Vec3 rayPos, Vec3 invRayDir, Vec3 rayDirSign, AABB aabb, float& dist, Vec3& normal);
void RaySphereIntersection(Vec3 const& rayPos, Vec3 const& rayDir, Vec3 const& pos, float radiusSq, float& dist, Vec3& normal);

#else
void RayAABB4(Vec43 rayPos, Vec43 invRayDir, Vec43 rayDirSign, AABB aabb, Vec3& dist, Vec43& normal);
void RaySphereIntersection4(Vec43 rayPos, Vec43 rayDir, Vec3 pos, Vec3 radiusSq, Vec3& dist, Vec43& normal);


void RayTriange4(Vec43 rayPos, Vec43 invRayDir, Vec43 rayDirSign, Vec3 triA, Vec3 triB, Vec3 triC, Vec3& dist, Vec43& normal);
#endif