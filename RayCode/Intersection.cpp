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

    if (xMax < yMin || yMax < xMin || xMin < 0.0f || yMin < 0.0f)
    {
        return;
    }

    int interceptAxis =  yMin > xMin ? 1 : 0;

    float tMin = MAX(xMin, yMin);
    float tMax = MIN(xMax, yMax);

    float zMin = MIN(zBoundsMin, zBoundsMax);
    float zMax = MAX(zBoundsMin, zBoundsMax);

    if (zMax < tMin || tMax < zMin || zMin < 0.0f)
    {
        return;
    }

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
void RayAABB(WVec rayPos, WVec invRayDir, WVec rayDirSign, AABB aabb, NVec& dist, WVec& normal)
{
    NVec xBoundsMax = (NVecMake(aabb.m_Max.m128_f32[0]) - rayPos.x) * invRayDir.x;
    NVec xBoundsMin = (NVecMake(aabb.m_Min.m128_f32[0]) - rayPos.x) * invRayDir.x;

    NVec yBoundsMax = (NVecMake(aabb.m_Max.m128_f32[1]) - rayPos.y) * invRayDir.y;
    NVec yBoundsMin = (NVecMake(aabb.m_Min.m128_f32[1]) - rayPos.y) * invRayDir.y;

    NVec zBoundsMax = (NVecMake(aabb.m_Max.m128_f32[2]) - rayPos.z) * invRayDir.z;
    NVec zBoundsMin = (NVecMake(aabb.m_Min.m128_f32[2]) - rayPos.z) * invRayDir.z;

    NVec xMin = NVecMin(xBoundsMin, xBoundsMax);
    NVec xMax = NVecMax(xBoundsMin, xBoundsMax);

    NVec yMin = NVecMin(yBoundsMin, yBoundsMax);
    NVec yMax = NVecMax(yBoundsMin, yBoundsMax);

    NVec interceptNormalAxis = NVecGT(yMin, xMin) * NVecMake(1.0f);

    NVec selection = NVecLT(xMax, yMin) * NVecLT(yMax, xMin);// * Vec3LT(yMax, _mm_set_ps1(0.0f)) * Vec3LT(xMax, _mm_set_ps1(0.0f));
    NVec tMin = NVecMax(xMin, yMin);
    NVec tMax = NVecMin(xMax, yMax);

    NVec zMin = NVecMin(zBoundsMin, zBoundsMax);
    NVec zMax = NVecMax(zBoundsMin, zBoundsMax);

    selection = selection * NVecLT(zMax, tMin) * NVecLT(tMax, zMin);// *Vec3LT(zMax, _mm_set_ps1(0.0f));

    interceptNormalAxis = NVecSelect(NVecMake(2.0f), interceptNormalAxis, NVecGT(zMin, tMin));

    tMin = NVecMax(tMin, zMin);

    selection = selection * NVecLT(tMin, NVecMake(0.0f));

    normal.x = NVecSelect(NVecMake(1.0f), NVecMake(0.0f), NVecEQ(interceptNormalAxis, NVecMake(0.0f))) * rayDirSign.x;
    normal.y = NVecSelect(NVecMake(1.0f), NVecMake(0.0f), NVecEQ(interceptNormalAxis, NVecMake(1.0f))) * rayDirSign.y;
    normal.z = NVecSelect(NVecMake(1.0f), NVecMake(0.0f), NVecEQ(interceptNormalAxis, NVecMake(2.0f))) * rayDirSign.z;

    dist = NVecSelect(dist, tMin, selection);


}

void RaySphereIntersection(WVec rayPos, WVec rayDir, Vec3 pos, NVec radiusSq, NVec& dist, WVec& normal)
{
#if 0
    WVec posSplatted = WVecMake2(pos.m128_f32[0], pos.m128_f32[1], pos.m128_f32[2]);

    WVec l = posSplatted - rayPos;



    NVec tca = WVecDot(l, rayDir);

    NVec hypSq = WVecDot(l, l);
    NVec oppSq = tca * tca;
    NVec d = NVecSqrt(hypSq - oppSq);

    NVec selectCriteria = NVecEQ(tca, NVecMake(0.0f));
    if (NVecMoveMask(selectCriteria) == 0)
    {
        return;
    }

    NVec thc = NVecSqrt(radiusSq - (d * d));
    NVec t0 = tca - thc;

    selectCriteria = NVecLT(d, radiusSq);

    if (NVecMoveMask(selectCriteria) == 0)
    {
        return;
    }

    dist = NVecSelect(t0, dist, selectCriteria);
    normal = (rayPos + rayDir * t0) - posSplatted;
    normal = WVecNormalise(normal);
#else
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