#include "Intersection.h"

#ifndef SIMD
void RayAABB(Vec3 rayPos, Vec3 invRayDir, Vec3 rayDirSign, AABB aabb, float& dist, Vec3& normal)
{
    float xBoundsMax = (aabb.m_Max.x - rayPos.x) * invRayDir.x;
    float xBoundsMin = (aabb.m_Min.x - rayPos.x) * invRayDir.x;

    float yBoundsMax = (aabb.m_Max.y - rayPos.y) * invRayDir.y;
    float yBoundsMin = (aabb.m_Min.y - rayPos.y) * invRayDir.y;

    float zBoundsMax = (aabb.m_Max.z - rayPos.z) * invRayDir.z;
    float zBoundsMin = (aabb.m_Min.z - rayPos.z) * invRayDir.z;

    float xMin = MIN(xBoundsMin, xBoundsMax);
    float xMax = MAX(xBoundsMin, xBoundsMax);

    float yMin = MIN(yBoundsMin, yBoundsMax);
    float yMax = MAX(yBoundsMin, yBoundsMax);

    if (xMax < yMin || yMax < xMin)
    {
        return;
    }

    int interceptAxis =  yMin > xMin ? 1 : 0;

    float tMin = MAX(xMin, yMin);
    float tMax = MIN(xMax, yMax);

    float zMin = MIN(zBoundsMin, zBoundsMax);
    float zMax = MAX(zBoundsMin, zBoundsMax);

    if (zMin > tMin)
    {
        tMin = zMin;
        interceptAxis = 2;
    }

    dist = tMin;
    Vec3 normals[3] =
    {
        Vec3Make(1.0f, 0.0f, 0.0f),
        Vec3Make(0.0f, 1.0f, 0.0f),
        Vec3Make(0.0f, 0.0f, 1.0f),
    };
    normal = normals[interceptAxis] * rayDirSign.m128_f32[interceptAxis];
}

void RaySphereIntersection(Vec3 const& rayPos, Vec3 const& rayDir, Vec3 const& pos, float radiusSq, float& dist, Vec3& normal)
{
    Vec3 l = pos - rayPos;
    float tca = Dot(l, rayDir);

    float hypSq = Dot(l, l);
    float oppSq = tca * tca;
    float dSq = hypSq - oppSq;

    if (tca < 0.0f || dSq  > radiusSq)
    {
        return;
    }

    float thc = sqrtf(radiusSq - dSq);
    float t0 = tca - thc;

    dist = t0;
    normal = Vec3Normalise((rayPos + rayDir * t0) - pos);
}

#else
void RayAABB4(Vec43 rayPos, Vec43 invRayDir, Vec43 rayDirSign, AABB aabb, Vec3& dist, Vec43& normal)
{
    Vec3 xBoundsMax = (Vec3Make(aabb.m_Max.m128_f32[0]) - rayPos.x) * invRayDir.x;
    Vec3 xBoundsMin = (Vec3Make(aabb.m_Min.m128_f32[0]) - rayPos.x) * invRayDir.x;

    Vec3 yBoundsMax = (Vec3Make(aabb.m_Max.m128_f32[1]) - rayPos.y) * invRayDir.y;
    Vec3 yBoundsMin = (Vec3Make(aabb.m_Min.m128_f32[1]) - rayPos.y) * invRayDir.y;

    Vec3 zBoundsMax = (Vec3Make(aabb.m_Max.m128_f32[2]) - rayPos.z) * invRayDir.z;
    Vec3 zBoundsMin = (Vec3Make(aabb.m_Min.m128_f32[2]) - rayPos.z) * invRayDir.z;

    Vec3 xMin = Vec3Min(xBoundsMin, xBoundsMax);
    Vec3 xMax = Vec3Max(xBoundsMin, xBoundsMax);

    Vec3 yMin = Vec3Min(yBoundsMin, yBoundsMax);
    Vec3 yMax = Vec3Max(yBoundsMin, yBoundsMax);

    Vec3 interceptNormalAxis = Vec3GT(yMin, xMin) * _mm_set_ps1(1.0f);

    Vec3 selection = Vec3LT(xMax, yMin) * Vec3LT(yMax, xMin);// * Vec3LT(yMax, _mm_set_ps1(0.0f)) * Vec3LT(xMax, _mm_set_ps1(0.0f));
    Vec3 tMin = Vec3Max(xMin, yMin);
    Vec3 tMax = Vec3Min(xMax, yMax);

    Vec3 zMin = Vec3Min(zBoundsMin, zBoundsMax);
    Vec3 zMax = Vec3Max(zBoundsMin, zBoundsMax);

    selection = selection * Vec3LT(zMax, tMin) * Vec3LT(tMax, zMin);// *Vec3LT(zMax, _mm_set_ps1(0.0f));

    interceptNormalAxis = Vec3Select(_mm_set_ps1(2.0f), interceptNormalAxis, Vec3GT(zMin, tMin));

    tMin = Vec3Max(tMin, zMin);

    selection = selection * Vec3LT(tMin, _mm_set_ps1(0.0f));

    normal.x = Vec3Select(_mm_set_ps1(1.0f), _mm_set_ps1(0.0f), _mm_cmpeq_ps(interceptNormalAxis, _mm_set_ps1(0.0f))) * rayDirSign.x;
    normal.y = Vec3Select(_mm_set_ps1(1.0f), _mm_set_ps1(0.0f), _mm_cmpeq_ps(interceptNormalAxis, _mm_set_ps1(1.0f))) * rayDirSign.y;
    normal.z = Vec3Select(_mm_set_ps1(1.0f), _mm_set_ps1(0.0f), _mm_cmpeq_ps(interceptNormalAxis, _mm_set_ps1(2.0f))) * rayDirSign.z;

    dist = Vec3Select(dist, tMin, selection);


}

void RaySphereIntersection4(Vec43 rayPos, Vec43 rayDir, Vec3 pos, Vec3 radiusSq, Vec3& dist, Vec43& normal)
{
#if 1
    Vec43 posSplatted = Vec43Make(pos);

    Vec43 l = posSplatted - rayPos;



    Vec3 tca = Vec43Dot(l, rayDir);

    Vec3 hypSq = Vec43Dot(l, l);
    Vec3 oppSq = tca * tca;
    Vec3 d = _mm_sqrt_ps(hypSq - oppSq);

    Vec3 selectCriteria = _mm_cmpge_ps(tca, _mm_set_ps1(0.0f));
    if (_mm_movemask_ps(selectCriteria) == 0)
    {
        return;
    }

    Vec3 thc = _mm_sqrt_ps(radiusSq - (d * d));
    Vec3 t0 = tca - thc;

    selectCriteria = Vec3LT(d, radiusSq);

    if (_mm_movemask_ps(selectCriteria) == 0)
    {
        return;
    }

    dist = Vec3Select(t0, dist, selectCriteria);
    normal = (rayPos + rayDir * t0) - posSplatted;
    normal = Vec43Normalise(normal);
#else
    dist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };

    Vec43 posExtened = { Vec3Make(pos.m128_f32[0]), Vec3Make(pos.m128_f32[1]), Vec3Make(pos.m128_f32[2]) };

    Vec43 rayToSphere = posExtened - rayPos;

    Vec3 a = Vec43Dot(rayToSphere, rayToSphere);

    Vec3 aGreaterThanZero = Vec3GT(a, Vec3Make(0.0f, 0.0f, 0.0f, 0.0f));

    Vec3 b = Vec43Dot(rayToSphere, rayDir);
    Vec3 d = radiusSq - (a - (b * b));

    Vec3 minD = b - Vec3Sqrt(d);

    dist = Vec3Select(minD, dist, aGreaterThanZero);
    normal = Vec43Normalise((rayPos + rayDir * minD) - posExtened);
#endif
}

void RayTriange4(Vec43 rayPos, Vec43 rayDir, Vec43 rayDirSign, Vec3 triA, Vec3 triB, Vec3 triC, Vec3& dist, Vec43& normal)
{
    Vec3 triangleNormal = Vec3Normalise(Vec3Cross(triA - triB, triA - triC));

    Vec43 triangleNormalE = { triangleNormal, triangleNormal, triangleNormal };

    Vec3 distToTri = Vec43Dot(rayDir, triangleNormalE);

    normal.x = Vec3Make(0.0f);
    normal.y = Vec3Make(1.0f);
    normal.z = Vec3Make(0.0f);
}
#endif