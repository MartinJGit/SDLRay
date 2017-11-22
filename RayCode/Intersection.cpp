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



void RayDualAABB(WVec rayPos, WVec invRayDir, Vec3 aabbX, Vec3 aabbY, Vec3 aabbZ, NVec& dist1, NVec& dist2)
{
    NVec xBoundsMax = (NVecMake(aabb.m_Max.m128_f32[0]) - rayPos.x) * invRayDir.x;
    NVec xBoundsMin = (NVecMake(aabb.m_Min.m128_f32[0]) - rayPos.x) * invRayDir.x;

    NVec yBoundsMax = (NVecMake(aabb.m_Max.m128_f32[1]) - rayPos.y) * invRayDir.y;
    NVec yBoundsMin = (NVecMake(aabb.m_Min.m128_f32[1]) - rayPos.y) * invRayDir.y;

    NVec zBoundsMax = (NVecMake(aabb.m_Max.m128_f32[2]) - rayPos.z) * invRayDir.z;
    NVec zBoundsMin = (NVecMake(aabb.m_Min.m128_f32[2]) - rayPos.z) * invRayDir.z;

    xMin = NVecMin(xBoundsMin, xBoundsMax);
    NVec xMax = NVecMax(xBoundsMin, xBoundsMax);

    yMin = NVecMin(yBoundsMin, yBoundsMax);
    NVec yMax = NVecMax(yBoundsMin, yBoundsMax);

    NVec selection = NVecLT(xMax, yMin) * NVecLT(yMax, xMin);// * Vec3LT(yMax, _mm_set_ps1(0.0f)) * Vec3LT(xMax, _mm_set_ps1(0.0f));
    tMin = NVecMax(xMin, yMin);
    NVec tMax = NVecMin(xMax, yMax);

    zMin = NVecMin(zBoundsMin, zBoundsMax);
    NVec zMax = NVecMax(zBoundsMin, zBoundsMax);

    selection = selection * NVecLT(zMax, tMin) * NVecLT(tMax, zMin);// *Vec3LT(zMax, _mm_set_ps1(0.0f));

    tMin = NVecMax(tMin, zMin);
    selection = selection * NVecLT(tMin, NVecMake(0.0f));
    dist = NVecSelect(dist, tMin, selection);
}
#else
inline void RayAABB(WVec rayPos, WVec invRayDir, AABB aabb, NVec& dist, NVec& xMin, NVec& yMin, NVec& zMin, NVec& tMin)
{
    NVec xBoundsMax = (NVecMake(aabb.m_Max.m128_f32[0]) - rayPos.x) * invRayDir.x;
    NVec xBoundsMin = (NVecMake(aabb.m_Min.m128_f32[0]) - rayPos.x) * invRayDir.x;

    NVec yBoundsMax = (NVecMake(aabb.m_Max.m128_f32[1]) - rayPos.y) * invRayDir.y;
    NVec yBoundsMin = (NVecMake(aabb.m_Min.m128_f32[1]) - rayPos.y) * invRayDir.y;

    NVec zBoundsMax = (NVecMake(aabb.m_Max.m128_f32[2]) - rayPos.z) * invRayDir.z;
    NVec zBoundsMin = (NVecMake(aabb.m_Min.m128_f32[2]) - rayPos.z) * invRayDir.z;

    xMin = NVecMin(xBoundsMin, xBoundsMax);
    NVec xMax = NVecMax(xBoundsMin, xBoundsMax);

    yMin = NVecMin(yBoundsMin, yBoundsMax);
    NVec yMax = NVecMax(yBoundsMin, yBoundsMax);

    NVec selection = NVecLT(xMax, yMin) * NVecLT(yMax, xMin);// * Vec3LT(yMax, _mm_set_ps1(0.0f)) * Vec3LT(xMax, _mm_set_ps1(0.0f));
    tMin = NVecMax(xMin, yMin);
    NVec tMax = NVecMin(xMax, yMax);

    zMin = NVecMin(zBoundsMin, zBoundsMax);
    NVec zMax = NVecMax(zBoundsMin, zBoundsMax);

    selection = selection * NVecLT(zMax, tMin) * NVecLT(tMax, zMin);// *Vec3LT(zMax, _mm_set_ps1(0.0f));

    tMin = NVecMax(tMin, zMin);
    selection = selection * NVecLT(tMin, NVecMake(0.0f));
    dist = NVecSelect(dist, tMin, selection);
}

void RayAABB(WVec rayPos, WVec invRayDir, AABB aabb, NVec& dist)
{
    NVec xMin, yMin, zMin, tMin;
    RayAABB(rayPos, invRayDir, aabb, dist, xMin, yMin, zMin, tMin);
}

void RayAABB(WVec rayPos, WVec invRayDir, WVec rayDirSign, AABB aabb, NVec& dist, WVec& normal)
{
    NVec xMin, yMin, zMin, tMin;
    RayAABB(rayPos, invRayDir, aabb, dist, xMin, yMin, zMin, tMin);

    NVec interceptNormalAxis = NVecGT(yMin, xMin) * NVecMake(1.0f);
    interceptNormalAxis = NVecSelect(NVecMake(2.0f), interceptNormalAxis, NVecGT(zMin, tMin));

    normal.x = NVecSelect(NVecMake(1.0f), NVecMake(0.0f), NVecEQ(interceptNormalAxis, NVecMake(0.0f))) * rayDirSign.x;
    normal.y = NVecSelect(NVecMake(1.0f), NVecMake(0.0f), NVecEQ(interceptNormalAxis, NVecMake(1.0f))) * rayDirSign.y;
    normal.z = NVecSelect(NVecMake(1.0f), NVecMake(0.0f), NVecEQ(interceptNormalAxis, NVecMake(2.0f))) * rayDirSign.z;
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
    WVec posExtened = { NVecMake(pos.m128_f32[0]), NVecMake(pos.m128_f32[1]), NVecMake(pos.m128_f32[2]) };

    WVec rayToSphere = posExtened - rayPos;

    NVec a = WVecDot(rayToSphere, rayToSphere);

    NVec aGreaterThanZero = NVecGT(a, NVecMake(0.0f));

    NVec b = WVecDot(rayToSphere, rayDir);
    NVec d = radiusSq - (a - (b * b));

    NVec minD = b - NVecSqrt(d);

    dist = NVecSelect(minD, dist, aGreaterThanZero);
    normal = WVecNormalise((rayPos + rayDir * minD) - posExtened);
#endif
}

static const float cEPSILON = 0.00001f;

void RayTriange4(WVec rayPos, WVec rayDir, Vec3 triA, Vec3 triB, Vec3 triC, NVec& dist)
{
    Vec3 edge1 = triB - triA;
    Vec3 edge2 = triC - triA;

    WVec edge1Ext = WVecMake2(edge1.m128_f32[0], edge1.m128_f32[1], edge1.m128_f32[2]);
    WVec edge2Ext = WVecMake2(edge2.m128_f32[0], edge2.m128_f32[1], edge2.m128_f32[2]);

    WVec pvec;
    pvec = WVecCross(rayDir, edge2Ext);

    NVec det = WVecDot(edge1Ext, pvec);
    /*
    NVec lt0 = NVecLT(det, NVecMake(0.0f));
    pvec = WVecSelect(-pvec, pvec, lt0);
    det = NVecSelect(-det, det, lt0);
    edge1Ext = WVecSelect(-edge1Ext, edge1Ext, lt0);
    edge2Ext = WVecSelect(-edge2Ext, edge2Ext, lt0);
    
    if (det < 0.0f)
    {
        pvec = -pvec;
        det = -det;
        edge1 = -edge1;
        edge2 = -edge2;
    }*/
    NVec excludeResult = NVecLT(det, NVecMake(cEPSILON));

    WVec tvec = rayPos - WVecMake2(triA.m128_f32[0], triA.m128_f32[1], triA.m128_f32[2]);

    NVec u = WVecDot(tvec, pvec);
    excludeResult = NVecOr(NVecOr(excludeResult, NVecLT(u, NVecMake(0.0f))), NVecGT(u, det));

    WVec qvec = WVecCross(tvec, edge1Ext);

    NVec v = WVecDot(rayDir, qvec);
    excludeResult = NVecOr(NVecOr(excludeResult, NVecLT(v, NVecMake(0.0f))), NVecGT(u + v, det));

    NVec t = WVecDot(edge2Ext, qvec);
    excludeResult = NVecOr(excludeResult, NVecLT(t, NVecMake(0.0f)));

    dist = t / det;
    dist = NVecSelect(NVecMake(FLT_MAX), dist, excludeResult);
}
#endif