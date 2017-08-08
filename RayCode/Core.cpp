#include "math.h"
#include "Core.h"
#include <array>
#include <vector>
#include <concurrent_queue.h>
#include <thread>
#include "Windows.h"
#include "Vector.h"
#include "Intersection.h"

#ifdef _DEBUG
#include "gtest\gtest.h"
#endif

double GetInvFrequency()
{
    static double s_Frequency = 0;
    static double s_InvFrequency = 0;
    if (s_Frequency == 0)
    {
        LARGE_INTEGER frequency;
        QueryPerformanceFrequency(&frequency);
        s_Frequency = static_cast<double>(frequency.QuadPart);
        s_InvFrequency = 1.0 / static_cast<double>(frequency.QuadPart);
    }
    return s_InvFrequency;
}

double Now()
{
    LARGE_INTEGER time;
    time.QuadPart = 1;
    QueryPerformanceCounter(&time);
    return (double)time.QuadPart * GetInvFrequency();
}


struct IntersectionResult
{
    float m_Dist;
    Vec3 m_Normal;
};

struct Ray
{
    Vec3 m_Pos;
    Vec3 m_Dir;
    Vec3 m_InvDir;
    Vec3 m_Sign;
};

struct WorldData
{
    std::vector<Vec3> m_Spheres;
    std::vector<AABB> m_AABB;
};

void Entry(WorldData*& worldData)
{
#ifdef _DEBUG
    int argc = 1;
    char* nullPtr[] = { "GTest" };
    ::testing::InitGoogleTest(&argc, nullPtr);

    RUN_ALL_TESTS();
#endif

    CreateWorkers(15);

    if (worldData == nullptr)
    {
        worldData = new WorldData();

        for (int z = 0; z < 3; ++z)
        {
            for (int y = 0; y < 3; ++y)
            {
                for (int x = 0; x < 3; ++x)
                {
                    float xInterp = x / (3.0f - 1);
                    float yInterp = y / (3.0f - 1);
                    float zInterp = z / (3.0f - 1);
                    Vec3 sphereC = Vec3Make(6.0f * xInterp, 6.0f * yInterp, 6.0f * zInterp) - Vec3Make(3.0f, 3.0f, 9.0f);
                    worldData->m_Spheres.push_back(sphereC);
                }
            }
        }

        for (int y = 0; y < 3; ++y)
        {
            for (int x = 0; x < 3; ++x)
            {
                float xInterp = x / (3.0f - 1);
                float yInterp = y / (3.0f - 1);
                Vec3 sphereC = Vec3Make(6.0f * xInterp, 6.0f * yInterp, 0.0f) - Vec3Make(3.0f, 3.0f, 0.0f);
                AABB aab;
                aab.m_Min = sphereC - Vec3Make(0.5f);
                aab.m_Max = sphereC + Vec3Make(0.5f);
                worldData->m_AABB.push_back(aab);
            }
        }
    }
}

void Exit(WorldData*& worldData)
{
    DestroyWorkers();
}

#if 0
void RaySphereIntersectionTest(Ray r, Vec3 pos, Vec3 radiusSq, IntersectionResult& res)
{
    // Find the closest point on the ray to the sphere position, if the distance is less than the radius, we have a collision.
    float collisionDistance = FLT_MAX;

    Vec3 rayStartToSphereCentre = pos - r.m_Pos;
    Vec3 dot = Dot(rayStartToSphereCentre, r.m_Dir);
    if (dot.m128_f32[0] > 0.0f)
    {
        Vec3 closestPoint = r.m_Pos + r.m_Dir * dot;

        Vec3 pointToSphere = closestPoint - pos;
        Vec3 lenthSq = VecLengthSq(pointToSphere);
        Vec3 collided = _mm_cmple_ps(lenthSq, radiusSq);
        if (lenthSq.m128_f32[0] >= 0.0f && lenthSq.m128_f32[0] < radiusSq.m128_f32[0])
        {
            collisionDistance = dot.m128_f32[0];
            if (pointToSphere.m128_f32[0] > 0.000001f)
            {
                res.m_Normal = Vec3Normalise(pointToSphere);
            }
            else
            {
                res.m_Normal = -r.m_Dir;
            }
        }
    }

    res.m_Dist = collisionDistance;
}
#else

#endif
#if 1


#else
void RaySphereIntersection4(Vec3 pX, Vec3 pY, Vec3 pZ, Vec3 dX, Vec3 dY, Vec3 dZ, Vec3 pos, Vec3 radiusSq, Vec3& dist, Vec3& normalsX, Vec3& normalsY, Vec3& normalsZ)
{
    dist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };

    Vec3 posX = { pos.m128_f32[0], pos.m128_f32[0] , pos.m128_f32[0] , pos.m128_f32[0] };
    Vec3 posY = { pos.m128_f32[1], pos.m128_f32[1] , pos.m128_f32[1] , pos.m128_f32[1] };
    Vec3 posZ = { pos.m128_f32[2], pos.m128_f32[2] , pos.m128_f32[2] , pos.m128_f32[2] };

    Vec3 rayToSphereX = posX - pX;
    Vec3 rayToSphereY = posY - pY;
    Vec3 rayToSphereZ = posZ - pZ;

    Vec3 a = rayToSphereX * rayToSphereX + rayToSphereY * rayToSphereY + rayToSphereZ * rayToSphereZ;

    Vec3 aGreaterThanZero = Vec3GT(a, Vec3Make(0.0f, 0.0f, 0.0f, 0.0f));

    Vec3 b = rayToSphereX * dX + rayToSphereY * dY + rayToSphereZ * dZ;
    Vec3 d = radiusSq - (a - (b * b));


    Vec3 minD = b - Vec3Sqrt(d);

    dist = Vec3Select(minD, dist, aGreaterThanZero);
    normalsX = (pX + dX * minD) - posX;
    normalsY = (pY + dY * minD) - posY;
    normalsZ = (pZ + dZ * minD) - posZ;

    Vec3 length = Vec3Sqrt(normalsX * normalsX + normalsY * normalsY + normalsZ * normalsZ);

    normalsX = normalsX / length;
    normalsY = normalsY / length;
    normalsZ = normalsZ / length;
}

#endif

#ifdef SIMD
void VECTOR_CALL TraceRay4(WVec rayP, WVec rayD, WVec invRayD, WVec rayDSign, WorldData const& world, int depth, WVec& results)
{
    NVec sphereRadius = NVecMake(1.0f);

    Vec3 lightDir = Vec3Normalise(Vec3Make(-1.0f, -1.0f, 0.0f, 0.0f));
    lightDir = -lightDir;

    WVec lightColor = WVecMake2(0.5f, 0.5f, 0.5f);

    WVec lightDirExp = WVecMake2(
        lightDir.m128_f32[0],
        lightDir.m128_f32[1],
        lightDir.m128_f32[2]);

    NVec closestIntersection = NVecMake(0.0f);
    WVec closestNormal = WVecMake(0.0f);

    for (auto sphere : world.m_Spheres)
    {
        WVec normal;

        NVec intersectionDist = NVecMake(0.0f);
        RaySphereIntersection(rayP, rayD, sphere, sphereRadius, intersectionDist, normal);

        NVec useNewNormal = NVecLT(intersectionDist, closestIntersection);
        closestIntersection = NVecSelect(intersectionDist, closestIntersection, useNewNormal);
        closestNormal = WVecSelect(normal, closestNormal, useNewNormal);
    }

    for (auto aabb : world.m_AABB)
    {
        WVec normal;

        NVec intersectionDist = NVecMake(0.0f);
        RayAABB(rayP, invRayD, rayDSign, aabb, intersectionDist, normal);

        NVec useNewNormal = NVecLT(intersectionDist, closestIntersection);
        closestIntersection = NVecSelect(intersectionDist, closestIntersection, useNewNormal);
        closestNormal = WVecSelect(normal, closestNormal, useNewNormal);
    }


    NVec litColor = NVecMax(WVecDot(closestNormal, lightDirExp), NVecMake(0.0f));
    WVec rayColors = lightColor * litColor + WVecMake(0.2f);
    NVec foundIntersection = NVecLT(closestIntersection, NVecMake(0.0f));
    results = WVecSelect(rayColors, results, foundIntersection);

    if (NVecMoveMask(foundIntersection) > 0 && depth > 0)
    {
        WVec rayDir = WVecReflect(rayD, closestNormal);
        WVec rayPos = rayP + rayD * closestIntersection;

        WVec rayDSign = {
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.x, NVecMake(0.0f))),
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.y, NVecMake(0.0f))),
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.z, NVecMake(0.0f))),
        };

        WVec invRayDir = {
            NVecMake(1.0f) / rayDir.x,
            NVecMake(1.0f) / rayDir.y,
            NVecMake(1.0f) / rayDir.z,
        };

        WVec reflectionRes = WVecMake(0.0f);
        TraceRay4(rayPos, rayDir, invRayDir, rayDSign, world, depth - 1, reflectionRes);

        results = results + reflectionRes * NVecMake(0.25f);
    }
}

#ifdef AVX
#if 0
void VECTOR_CALL TraceRay8(Vec83 rayP, Vec83 rayD, Vec83 invRayD, Vec83 rayDSign, WorldData const& world, int depth, Vec83& results)
{
    Vec3 sphereRadius = { 1.0f, 1.0f, 1.0f, 1.0f };

    Vec3 lightDir = Vec3Normalise(Vec3Make(-1.0f, -1.0f, 0.0f, 0.0f));
    lightDir = -lightDir;

    Vec83 lightColor = { Vec3Make(0.5f), Vec3Make(0.5f), Vec3Make(0.5f) };

    Vec83 lightDirExp = {
        Vec3Make(lightDir.m128_f32[0]),
        Vec3Make(lightDir.m128_f32[1]),
        Vec3Make(lightDir.m128_f32[2]) };

    Vec3 closestIntersection = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
    Vec83 closestNormal = { Vec3Make(0.0f), Vec3Make(0.0f), Vec3Make(0.0f) };

    for (auto sphere : world.m_Spheres)
    {
        Vec83 normal;

        Vec8 intersectionDist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
        RaySphereIntersection4(rayP, rayD, sphere, sphereRadius, intersectionDist, normal);

        Vec3 useNewNormal = Vec3LT(intersectionDist, closestIntersection);
        closestIntersection = Vec3Select(intersectionDist, closestIntersection, useNewNormal);
        closestNormal = Vec43Select(normal, closestNormal, useNewNormal);
    }

    for (auto aabb : world.m_AABB)
    {
        Vec83 normal;

        Vec8 intersectionDist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
        RayAABB4(rayP, invRayD, rayDSign, aabb, intersectionDist, normal);

        Vec3 useNewNormal = Vec3LT(intersectionDist, closestIntersection);
        closestIntersection = Vec3Select(intersectionDist, closestIntersection, useNewNormal);
        closestNormal = Vec43Select(normal, closestNormal, useNewNormal);
    }


    Vec3 litColor = Vec3Max(Vec43Dot(closestNormal, lightDirExp), Vec3Make(0.0f));
    Vec83 rayColors = lightColor * litColor + Vec43Make(Vec3Make(0.2f, 0.2f, 0.2f, 0.0f));
    Vec3 foundIntersection = Vec3LT(closestIntersection, { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX });
    results = Vec43Select(rayColors, results, foundIntersection);

    if (Vec3MoveMask(foundIntersection) > 0 && depth > 0)
    {
        Vec83 rayDir = Vec43Reflect(rayD, closestNormal);
        Vec83 rayPos = rayP + rayD * closestIntersection;

        Vec83 rayDSign = {
            Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.x, Vec3Make(0.0f))),
            Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.y, Vec3Make(0.0f))),
            Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.z, Vec3Make(0.0f))),
        };

        Vec83 invRayDir = {
            Vec3Make(1.0f) / rayDir.x,
            Vec3Make(1.0f) / rayDir.y,
            Vec3Make(1.0f) / rayDir.z,
        };

        Vec83 reflectionRes = { Vec3Make(0.0f), Vec3Make(0.0f), Vec3Make(0.0f) };
        TraceRay4(rayPos, rayDir, invRayDir, rayDSign, world, depth - 1, reflectionRes);

        results = results + reflectionRes * Vec3Make(0.25f, 0.25f, 0.25f, 0.25f);
    }
}
#endif
#endif
#else

Color VECTOR_CALL TraceRay(Ray ray, WorldData const& world, int depth)
{
    Color result = { 0.0f, 0.0f, 0.0f, 1.0f };

    float sphereRadius = 1.0f;

    Vec3 lightDir = Vec3Normalise(Vec3Make(-1.0f, -1.0f, 0.0f));

    IntersectionResult closestIntersection;
    closestIntersection.m_Dist = FLT_MAX;

    for (auto sphere : world.m_Spheres)
    {
        IntersectionResult res;
        res.m_Dist = FLT_MAX;
        RaySphereIntersection(ray.m_Pos, ray.m_Dir, sphere, sphereRadius, res.m_Dist, res.m_Normal);
        if (res.m_Dist < closestIntersection.m_Dist)
        {
            closestIntersection = res;
        }
    }

    for (auto aabb : world.m_AABB)
    {
        IntersectionResult res;
        res.m_Dist = FLT_MAX;
        RayAABB(ray.m_Pos, ray.m_InvDir, ray.m_Sign, aabb, res.m_Dist, res.m_Normal);
        if (res.m_Dist < closestIntersection.m_Dist)
        {
            closestIntersection = res;
        }
    }

    if (closestIntersection.m_Dist != FLT_MAX)
    {
        Vec3 lightDir = Vec3Normalise({ -1.0f, -1.0f, 0.0f });
        lightDir = -lightDir;

        Vec3 lightColor = Vec3Make(0.5f);


        float lightIntensity = MAX(Dot(closestIntersection.m_Normal, lightDir), 0.0f);
        Vec3 rayColors = lightColor * lightIntensity + Vec3Make(0.2f, 0.2f, 0.2f);
        result.m128_f32[0] = rayColors.x;
        result.m128_f32[1] = rayColors.y;
        result.m128_f32[2] = rayColors.z;

        if (depth > 0)
        {
            Ray reflectedRay;
            reflectedRay.m_Dir = Vec3Reflect(ray.m_Dir, closestIntersection.m_Normal);
            reflectedRay.m_Pos = ray.m_Pos + ray.m_Dir * closestIntersection.m_Dist;

            reflectedRay.m_Sign = {
                reflectedRay.m_Dir.x > 0.0f ? 1.0f : -1.0f,
                reflectedRay.m_Dir.y > 0.0f ? 1.0f : -1.0f,
                reflectedRay.m_Dir.z > 0.0f ? 1.0f : -1.0f };

            reflectedRay.m_InvDir = {
                1.0f / reflectedRay.m_Dir.x,
                1.0f / reflectedRay.m_Dir.y,
                1.0f / reflectedRay.m_Dir.z,
            };

            Color reflectedRes = TraceRay(reflectedRay, world, depth - 1);
            result.m128_f32[0] += reflectedRes.m128_f32[0] * 0.25f;
            result.m128_f32[1] += reflectedRes.m128_f32[1] * 0.25f;
            result.m128_f32[2] += reflectedRes.m128_f32[2] * 0.25f;
        }
    }

    return result;
}
#endif

struct RowTrace
{
    Color* m_Buffer;
    int m_Width;
    int m_Height;
    int m_Row;
    WorldData* m_World;
};

Concurrency::concurrent_queue<RowTrace> gRows;
std::atomic<int> gRowCount;

float xRot = 0.0f;

void TraceRow(RowTrace row)
{
    Vec3 cameraPosition = Vec3Make(0.0f);

    cameraPosition = Vec3Make(sinf(xRot) * 10.0f, 0.0f, cosf(xRot) * 10.0f);

    Vec3 cameraDir = Vec3Normalise(-cameraPosition);// TransformByMatrix(Vec3Make(0.0f, 0.0f, -1.0f), CreateRotationMatrixX(xRot) * CreateRotationMatrixZ(zRot));
#if 0
    Vec3 worldUp = Vec3Make(0.0f, 1.0f, 0.0f, 0.0f);
    Vec3 cameraRight = Vec3Normalise(Vec3Cross(worldUp, cameraPosition));

    Vec3 cameraUp = Vec3Cross(cameraDir, cameraRight);
#endif

    Vec3 topLeft = { -0.5f, 0.5f, 9.0f };
    Vec3 topRight{ 0.5f, 0.5f, 9.0f };

    Vec3 bottomLeft = { -0.5f, -0.5f, 9.0f };
    Vec3 bottomRight{ 0.5f, -0.5f, 9.0f };

#ifdef SIMD

    WVec topLeftS = WVecMake2(-0.5f, 0.5f, 9.0f);
    WVec topRightS = WVecMake2(0.5f, 0.5f, 9.0f);
    WVec bottomLeftS = WVecMake2(-0.5f, -0.5f, 9.0f);
    WVec bottomRightS = WVecMake2(0.5f, -0.5f, 9.0f);

    WVec rayPos = WVecMake2(cameraPosition.m128_f32[0], cameraPosition.m128_f32[1], cameraPosition.m128_f32[2]);

    NVec yNorm = NVecMake(row.m_Row / (row.m_Height - 1.0f));

    int const rayWidth = sizeof(NVec) / sizeof(float);

    for (int x = 0; x < row.m_Width; x += rayWidth)
    {
#ifdef SIMD
        NVec xNorm = { x / (row.m_Width - 1.0f), (x + 1) / (row.m_Width - 1.0f), (x + 2) / (row.m_Width - 1.0f), (x + 3) / (row.m_Width - 1.0f) };
#else
        NVec xNorm = Vec8Make(
            x / (row.m_Width - 1.0f), 
            (x + 1) / (row.m_Width - 1.0f), 
            (x + 2) / (row.m_Width - 1.0f), 
            (x + 3) / (row.m_Width - 1.0f),
            (x + 4) / (row.m_Width - 1.0f),
            (x + 5) / (row.m_Width - 1.0f),
            (x + 6) / (row.m_Width - 1.0f),
            (x + 7) / (row.m_Width - 1.0f));
#endif

        WVec rayPoint = WVecLerp(WVecLerp(topLeftS, topRightS, xNorm), WVecLerp(bottomLeftS, bottomRightS, xNorm), yNorm);
        WVec rayDir = WVecNormalise(rayPoint - rayPos);
        WVec color = WVecMake(0.0f);

        WVec invRayDir = {
            NVecMake(1.0f) / rayDir.x,
            NVecMake(1.0f) / rayDir.y,
            NVecMake(1.0f) / rayDir.z,
        };

        WVec rayDSign = {
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.x, NVecMake(0.0f))),
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.y, NVecMake(0.0f))),
            NVecSelect(NVecMake(-1.0f), NVecMake(1.0f), NVecGT(rayDir.z, NVecMake(0.0f))),
        };

        TraceRay4(rayPos, rayDir, invRayDir, rayDSign, *row.m_World, 1, color);

#ifdef AVX
        row.m_Buffer[row.m_Row * row.m_Width + x + 0] = { color.x.m256_f32[0], color.y.m256_f32[0], color.z.m256_f32[0], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 1] = { color.x.m256_f32[1], color.y.m256_f32[1], color.z.m256_f32[1], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 2] = { color.x.m256_f32[2], color.y.m256_f32[2], color.z.m256_f32[2], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 3] = { color.x.m256_f32[3], color.y.m256_f32[3], color.z.m256_f32[3], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 4] = { color.x.m256_f32[4], color.y.m256_f32[4], color.z.m256_f32[4], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 5] = { color.x.m256_f32[5], color.y.m256_f32[5], color.z.m256_f32[5], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 6] = { color.x.m256_f32[6], color.y.m256_f32[6], color.z.m256_f32[6], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 7] = { color.x.m256_f32[7], color.y.m256_f32[7], color.z.m256_f32[7], 0.0f };
#else
        row.m_Buffer[row.m_Row * row.m_Width + x + 0] = { color.x.m128_f32[0], color.y.m128_f32[0], color.z.m128_f32[0], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 1] = { color.x.m128_f32[1], color.y.m128_f32[1], color.z.m128_f32[1], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 2] = { color.x.m128_f32[2], color.y.m128_f32[2], color.z.m128_f32[2], 0.0f };
        row.m_Buffer[row.m_Row * row.m_Width + x + 3] = { color.x.m128_f32[3], color.y.m128_f32[3], color.z.m128_f32[3], 0.0f };
#endif
    }

#else
    float yNorm = row.m_Row / (row.m_Height - 1.0f);
    for (int x = 0; x < row.m_Width; ++x)
    {
        Ray ray;
        ray.m_Pos = cameraPosition;

        float xNorm = x / (row.m_Width - 1.0f);


        Vec3 rayPoint = Lerp(Lerp(topLeft, topRight, xNorm), Lerp(bottomLeft, bottomRight, xNorm), yNorm);
        ray.m_Dir = rayPoint - cameraPosition;
        ray.m_Dir = Vec3Normalise(ray.m_Dir);
        ray.m_InvDir = Vec3Make(1.0f / ray.m_Dir.x, 1.0f / ray.m_Dir.y, 1.0f / ray.m_Dir.z);
        ray.m_Sign = Vec3Make(ray.m_Dir.x > 0.0f ? 1.0f : -1.0f, ray.m_Dir.y > 0.0f ? 1.0f : -1.0f, ray.m_Dir.z > 0.0f ? 1.0f : -1.0f);
        Color result = TraceRay(ray, *row.m_World, 1);
        row.m_Buffer[row.m_Row * row.m_Width + x] = result;
    }
#endif

   --gRowCount;
}

bool gShutdownWorkers = true;

void ThreadWorker()
{
    while (!gShutdownWorkers)
    {
        RowTrace r;
        while (gRows.try_pop(r))
        {
            TraceRow(r);
        }

        Sleep(5);
    }
}

std::vector<std::unique_ptr<std::thread>> threads;

void DestroyWorkers()
{
    gShutdownWorkers = true;
    for (auto& worker : threads)
    {
        worker->join();
    }

    threads.clear();
}

void CreateWorkers(int workerThreads)
{

    gShutdownWorkers = false;
    for (int threadI = 0; threadI < workerThreads; ++threadI)
    {
        threads.push_back(std::make_unique<std::thread>(ThreadWorker));
    }
}

void Trace(WorldData*& worldData, Color* buffer, int width, int height, int workerThreads)
{
    gRowCount = height;

    for (int y = 0; y < height; ++y)
    {
        RowTrace rowTrace;
        rowTrace.m_Height = height;
        rowTrace.m_Width = width;
        rowTrace.m_Buffer = buffer;
        rowTrace.m_Row = y;
        rowTrace.m_World = worldData;
        gRows.push(rowTrace);
    }

    RowTrace r;
    while (gRows.try_pop(r))
    {
        TraceRow(r);
    }

    // Wait for all rows to complete
    while (gRowCount > 0)
    {
    }
}

void TracePixel(WorldData*& worldData, int width, int height, int workerCount, int x, int y)
{
#if 0
    Vec3 cameraPosition = { 0.0f, 0.0f, 10.0f };

    cameraPosition = Vec3Make(sinf(xRot) * 10.0f, 0.0f, cosf(xRot) * 10.0f);

    Vec3 cameraDir = Vec3Normalise(-cameraPosition);// TransformByMatrix(Vec3Make(0.0f, 0.0f, -1.0f), CreateRotationMatrixX(xRot) * CreateRotationMatrixZ(zRot));


    Vec3 topLeft = { -0.5f, 0.5f, 9.0f };
    Vec3 topRight{ 0.5f, 0.5f, 9.0f };

    Vec3 bottomLeft = { -0.5f, -0.5f, 9.0f };
    Vec3 bottomRight{ 0.5f, -0.5f, 9.0f };

#ifdef SIMD
    Vec3 worldUp = Vec3Make(0.0f, 1.0f, 0.0f);
    Vec3 cameraRight = Vec3Normalise(Vec3Cross(worldUp, cameraPosition));

    Vec3 cameraUp = Vec3Cross(cameraDir, cameraRight);


    Ray ray;
    ray.m_Pos = cameraPosition;


    int xRounded = x / 4 * 4;

    float xNorm0 = (xRounded + 0) / (width - 1.0f);
    float xNorm1 = (xRounded + 1) / (width - 1.0f);
    float xNorm2 = (xRounded + 2) / (width - 1.0f);
    float xNorm3 = (xRounded + 3) / (width - 1.0f);

    Vec43 topLeftS = Vec43Make(topLeft);
    Vec43 topRightS = Vec43Make(topRight);
    Vec43 bottomLeftS = Vec43Make(bottomLeft);
    Vec43 bottomRightS = Vec43Make(bottomRight);

    Vec43 rayPos = Vec43Make(cameraPosition);

    Vec3 yNorm = Vec3Make(y / (height - 1.0f));
    
    Vec3 xNorm = Vec3Make(xRounded / (width - 1.0f), (xRounded + 1) / (width - 1.0f), (xRounded + 2) / (width - 1.0f), (xRounded + 3) / (width - 1.0f));

    Vec43 rayPoint = Vec43Lerp(Vec43Lerp(topLeftS, topRightS, xNorm), Vec43Lerp(bottomLeftS, bottomRightS, xNorm), yNorm);
    Vec43 rayDir = Vec43Normalise(rayPoint - rayPos);
    Vec43 color = Vec43Make(0.0f);

    Vec43 invRayDir = {
        Vec3Make(1.0f) / rayDir.x,
        Vec3Make(1.0f) / rayDir.y,
        Vec3Make(1.0f) / rayDir.z,
    };

    Vec43 rayDSign = {
        Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.x, Vec3Make(0.0f))),
        Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.y, Vec3Make(0.0f))),
        Vec3Select(Vec3Make(-1.0f), Vec3Make(1.0f), Vec3GT(rayDir.z, Vec3Make(0.0f))),
    };

    Vec43 res;
    TraceRay4(rayPoint, rayDir, invRayDir, rayDSign, *worldData, 1, res);
#else

    float xNorm = x / (width - 1.0f);
    float yNorm = y / (height - 1.0f);

    Ray ray;
    ray.m_Pos = Lerp(Lerp(topLeft, topRight, xNorm), Lerp(bottomLeft, bottomRight, xNorm), yNorm);
    ray.m_Dir = Vec3Normalise(cameraPosition);
    ray.m_InvDir = {
        1.0f / ray.m_Dir.x,
        1.0f / ray.m_Dir.y,
        1.0f / ray.m_Dir.z,
    };
    ray.m_Sign = {
        ray.m_Dir.x > 0.0f ? 1.0f : 0.0f, 
        ray.m_Dir.y > 0.0f ? 1.0f : 0.0f,
        ray.m_Dir.z > 0.0f ? 1.0f : 0.0f };

    Vec43 res;
    TraceRay(ray, *worldData, 1);
#endif
#endif
}
