#include "math.h"
#include "Core.h"
#include <smmintrin.h>
#include <array>
#include <vector>
#include <concurrent_queue.h>
#include <thread>
#include "Windows.h"

#define VECTOR_CALL __vectorcall

float PI = 3.14159265358979f;

inline void ASSERT_TRUE(bool condition)
{
    if (!condition)
    {
        int i = 0; ++i;
    }
}

inline void ASSERT_CLOSE(float a, float b)
{
    float diff = a - b;
    ASSERT_TRUE(diff < 0.00001f && diff > -0.00001f);
}

inline void ASSERT_EQUAL(float a, float b)
{
    float diff = a - b;
    ASSERT_TRUE(a == b);
}

double GetInvFrequency()
{
    static double s_Frequency = 0;
    static double s_InvFrequency = 0;
    if (s_Frequency == 0)
    {
        LARGE_INTEGER frequency;
        QueryPerformanceFrequency(&frequency);
        s_Frequency = static_cast<unsigned long long>(frequency.QuadPart);
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

typedef __m128 Vec3;

struct IntersectionResult
{
    float m_Dist;
    Vec3 m_Normal;
};

struct Matrix
{
    Vec3 c0, c1, c2, c3;
};

struct Vec43
{
    Vec3 x, y, z;
};

struct Ray
{
    Vec3 m_Pos;
    Vec3 m_Dir;
};

inline void ASSERT_CLOSE(Vec3 a, Vec3 b)
{
    ASSERT_CLOSE(a.m128_f32[0], b.m128_f32[0]);
    ASSERT_CLOSE(a.m128_f32[1], b.m128_f32[1]);
    ASSERT_CLOSE(a.m128_f32[2], b.m128_f32[2]);
}

Vec3 Vec3Make(float a)
{
    return _mm_set_ps1(a);
}

Vec3 Vec3Make(float a, float b, float c, float d)
{
    return _mm_set_ps(d, c, b, a);
}

Vec3 Vec3Make(float a, float b, float c)
{
    return _mm_set_ps(1.0f, c, b, a);
}

Vec3 operator - (Vec3 a, Vec3 b)
{
    return _mm_sub_ps(a, b);
}

Vec3 operator + (Vec3 a, Vec3 b)
{
    return _mm_add_ps(a, b);
}

Vec3 operator * (Vec3 a, Vec3 b)
{
    return _mm_mul_ps(a, b);
}

Vec3 operator * (Vec3 a, float b)
{
    Vec3 expanded = _mm_set1_ps(b);
    return _mm_mul_ps(a, expanded);
}

Vec3 operator / (Vec3 a, Vec3 b)
{
    return _mm_div_ps(a, b);
}

Vec3 Dot(Vec3 a, Vec3 b)
{
    int const mask = 0x77;
    return _mm_dp_ps(a, b, mask);
}

Vec3 Vec3Cross(Vec3 a, Vec3 b)
{
    return _mm_sub_ps(
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2))),
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)))
    );
}

Vec3 operator -(Vec3 a)
{
    return _mm_mul_ps(a, _mm_set1_ps(-1));
}

Vec3 VecLengthSq(Vec3 a)
{
    return Dot(a, a);
}

Vec3 Lerp(Vec3 a, Vec3 b, float t)
{
    return a + (b - a) * t;
}

Vec3 Lerp(Vec3 a, Vec3 b, Vec3 t)
{
    return a + (b - a) * t;
}

Vec3 Vec3Min(Vec3 a, Vec3 b)
{
    return _mm_min_ps(a, b);
}

Vec3 Vec3Max(Vec3 a, Vec3 b)
{
    return _mm_max_ps(a, b);
}

Vec3 Vec3LT(Vec3 a, Vec3 b)
{
    return _mm_cmplt_ps(a, b);
}

Vec3 Vec3GT(Vec3 a, Vec3 b)
{
    return _mm_cmpgt_ps(a, b);
}

Vec3 Vec3And(Vec3 a, Vec3 b)
{
    return _mm_and_ps(a, b);
}

Vec3 Vec3Nand(Vec3 a, Vec3 b)
{
    return _mm_andnot_ps(b, a);
}

Vec3 VECTOR_CALL Vec3Normalise(Vec3 a)
{
    Vec3 length = VecLengthSq(a);
    __m128 and = { 0, FLT_MAX, FLT_MIN, FLT_MIN };
    __m128 normalised = _mm_mul_ps(a, _mm_rsqrt_ps(length));
    normalised.m128_f32[3] = 1.0f;
    return normalised;
}

Vec3 VECTOR_CALL Vec3Sqrt(Vec3 a)
{
    return _mm_sqrt_ps(a);
}

Vec3 VECTOR_CALL Vec3Select(Vec3 a, Vec3 b, Vec3 mask)
{
    return Vec3And(a, mask) + Vec3Nand(b, mask);
}

Vec3 VECTOR_CALL Reflect(Vec3 dir, Vec3 normal)
{
    Vec3 a = Dot(dir, normal);
    Vec3 b = a * normal;
    Vec3 c = b * 2;
    Vec3 res = dir - c;
    return res;
}

Vec3 VECTOR_CALL Vec43Dot(Vec43 a, Vec43 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec43 VECTOR_CALL operator - (Vec43 a, Vec43 b)
{
    Vec43 res;
    res.x = a.x - b.x;
    res.y = a.y - b.y;
    res.z = a.z - b.z;
    return res;
}

Vec43 VECTOR_CALL operator - (Vec3 a, Vec43 b)
{
    Vec43 res;
    res.x = a - b.x;
    res.y = a - b.y;
    res.z = a - b.z;
    return res;
}

Vec43 VECTOR_CALL operator - (Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = a.x - b;
    res.y = a.y - b;
    res.z = a.z - b;
    return res;
}

Vec43 VECTOR_CALL operator * (Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = a.x * b;
    res.y = a.y * b;
    res.z = a.z * b;
    return res;
}

Vec43 VECTOR_CALL operator + (Vec43 a, Vec43 b)
{
    Vec43 res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;
    return res;
}

Vec43 VECTOR_CALL Vec43And(Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = Vec3And(a.x, b);
    res.y = Vec3And(a.y, b);
    res.z = Vec3And(a.z, b);
    return res;
}

Vec43 VECTOR_CALL Vec43Nand(Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = Vec3Nand(a.x, b);
    res.y = Vec3Nand(a.y, b);
    res.z = Vec3Nand(a.z, b);
    return res;
}

Vec43 VECTOR_CALL Vec43Select(Vec43 a, Vec43 b, Vec3 mask)
{
    return Vec43And(a, mask) + Vec43Nand(b, mask);
}


Vec3 VECTOR_CALL Vec43Length(Vec43 a)
{
    return Vec3Sqrt(Vec43Dot(a, a));
}

Vec43 VECTOR_CALL Vec43Normalise(Vec43 a)
{
    Vec3 length = Vec43Length(a);
    
    Vec43 res;
    res.x = a.x / length;
    res.y = a.y / length;
    res.z = a.z / length;
    return res;
}

Vec43 VECTOR_CALL Vec43Reflect(Vec43 dir, Vec43 normal)
{
    Vec3 a = Vec43Dot(dir, normal);
    Vec43 b = normal * a;
    Vec43 c = b * Vec3Make(2.0f);
    Vec43 res = dir - c;
    return res;
}

Vec43 VECTOR_CALL Vec43Make(Vec3 splat)
{
    Vec43 vecM;
    vecM.x = Vec3Make(splat.m128_f32[0]);
    vecM.y = Vec3Make(splat.m128_f32[1]);
    vecM.z = Vec3Make(splat.m128_f32[2]);
    return vecM;
}

Vec43 VECTOR_CALL Vec43Make(float splat)
{
    return Vec43Make(Vec3Make(splat));
}

Vec43 VECTOR_CALL Vec43Lerp(Vec43 a, Vec43 b, Vec3 t)
{
    Vec43 res;
    res.x = a.x + (b.x - a.x) * t;
    res.y = a.y + (b.y - a.y) * t;
    res.z = a.z + (b.z - a.z) * t;
    return res;
}

Vec3 __vectorcall TransformByMatrix(Vec3 a, Matrix m)
{
    Vec3 x = _mm_permute_ps(a, _MM_SHUFFLE(0, 0, 0, 0));
    Vec3 y = _mm_permute_ps(a, _MM_SHUFFLE(1, 1, 1, 1));
    Vec3 z = _mm_permute_ps(a, _MM_SHUFFLE(2, 2, 2, 2));
    Vec3 w = Vec3Make(1.0f);

    return _mm_add_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x, m.c0), _mm_mul_ps(y, m.c1)), _mm_mul_ps(z, m.c2)), _mm_mul_ps(w, m.c3));
}

Matrix __vectorcall operator * (Matrix a, Matrix b)
{
    Matrix c;

    Vec3 ar0 = Vec3Make(a.c0.m128_f32[0], a.c1.m128_f32[0], a.c2.m128_f32[0], a.c3.m128_f32[0]);
    Vec3 ar1 = Vec3Make(a.c0.m128_f32[1], a.c1.m128_f32[1], a.c2.m128_f32[1], a.c3.m128_f32[1]);
    Vec3 ar2 = Vec3Make(a.c0.m128_f32[2], a.c1.m128_f32[2], a.c2.m128_f32[2], a.c3.m128_f32[2]);
    Vec3 ar3 = Vec3Make(a.c0.m128_f32[3], a.c1.m128_f32[3], a.c2.m128_f32[3], a.c3.m128_f32[3]);

    c.c0 = Vec3Make(Dot(ar0, b.c0).m128_f32[0], Dot(ar0, b.c1).m128_f32[0], Dot(ar0, b.c2).m128_f32[0], Dot(ar0, b.c3).m128_f32[0]);
    c.c1 = Vec3Make(Dot(ar1, b.c0).m128_f32[0], Dot(ar1, b.c1).m128_f32[0], Dot(ar1, b.c2).m128_f32[0], Dot(ar1, b.c3).m128_f32[0]);
    c.c2 = Vec3Make(Dot(ar2, b.c0).m128_f32[0], Dot(ar2, b.c1).m128_f32[0], Dot(ar2, b.c2).m128_f32[0], Dot(ar2, b.c3).m128_f32[0]);
    c.c3 = Vec3Make(Dot(ar3, b.c0).m128_f32[0], Dot(ar3, b.c1).m128_f32[0], Dot(ar3, b.c2).m128_f32[0], Dot(ar3, b.c3).m128_f32[0]);

    return c;
}

Vec3 __vectorcall GetTranslation(Matrix a)
{
    return a.c3;
}

Matrix __vectorcall CreateTranslationMatrix(Vec3 a)
{
    Matrix m;
#if 0
    m.c0 = Vec3Make(1.0f, 0.0f, 0.0f, a.m128_f32[0]);
    m.c1 = Vec3Make(0.0f, 1.0f, 0.0f, a.m128_f32[1]);
    m.c2 = Vec3Make(0.0f, 0.0f, 1.0f, a.m128_f32[2]);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, a.m128_f32[3]);
#else
    m.c0 = Vec3Make(1.0f, 0.0f, 0.0f, 0.0f);
    m.c1 = Vec3Make(0.0f, 1.0f, 0.0f, 0.0f);
    m.c2 = Vec3Make(0.0f, 0.0f, 1.0f, 0.0f);
    m.c3 = a;
#endif
    return m;
}

Matrix __vectorcall CreateRotationMatrixX(float a)
{
    Matrix m;
    m.c0 = Vec3Make(1.0f, 0.0f, 0.0f, 0.0f);
    m.c1 = Vec3Make(0.0f, cosf(a), -sinf(a), 0.0f);
    m.c2 = Vec3Make(0.0f, sinf(a), cosf(a), 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}


Matrix __vectorcall CreateRotationMatrixY(float a)
{
    Matrix m;
    m.c0 = Vec3Make(cosf(a), 0.0f, sinf(a), 0.0f);
    m.c1 = Vec3Make(0.0f, 1.0f, 0.0f, 0.0f);
    m.c2 = Vec3Make(-sinf(a), 0.0f, cosf(a), 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}


Matrix __vectorcall CreateRotationMatrixZ(float a)
{
    Matrix m;
    m.c0 = Vec3Make(cosf(a), -sinf(a), 0.0f, 0.0f);
    m.c1 = Vec3Make(sinf(a), cosf(a), 0.0f, 0.0f);
    m.c2 = Vec3Make(0.0f, 0.0f, 1.0f, 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}

void Entry(void*& worldData)
{
    {
        Vec3 a = Vec3Make(0.0f, 0.0f, 3.0f);
        Matrix translationMatrix = CreateTranslationMatrix(a);
        Vec3 transformedVec = TransformByMatrix(a, translationMatrix);

        ASSERT_EQUAL(transformedVec.m128_f32[2], 6.0f);
    }

    {
        Vec3 a = Vec3Make(0.0f, 0.0f, 1.0f);
        Matrix translationMatrix = CreateRotationMatrixX(PI);
        Vec3 transformedVec = TransformByMatrix(a, translationMatrix);

        ASSERT_CLOSE(transformedVec, Vec3Make(0.0f, 0.0f, -1.0f));

    }

    {
        Vec3 a = Vec3Make(0.0f, 0.0f, 1.0f);
        Matrix translationMatrix = CreateRotationMatrixY(PI * 0.5f);
        Vec3 transformedVec = TransformByMatrix(a, translationMatrix);

        ASSERT_CLOSE(transformedVec, Vec3Make(-1.0f, 0.0f, 0.0f));
    }

    {
        Matrix translationMatrixA = CreateTranslationMatrix(Vec3Make(0.0f, 0.0f, 1.0f));
        Matrix translationMatrixB = CreateTranslationMatrix(Vec3Make(0.0f, 3.0f, 0.0f));
        Matrix composedMatrix = translationMatrixA * translationMatrixB;
        Vec3 translation = GetTranslation(composedMatrix);
        ASSERT_CLOSE(translation, Vec3Make(0.0f, 3.0f, 1.0f));

        Vec3 transformedVec = TransformByMatrix(Vec3Make(5.0f, 0.0f, 0.0f), composedMatrix);

        ASSERT_CLOSE(transformedVec, Vec3Make(5.0f, 3.0f, 1.0f));
    }

    {
        Vec43 tes = { 
            Vec3Make(-1.0f, 0.0f, 1.0f, 0.0f),
            Vec3Make(0.0f, 1.0f, 0.0f, -1.0f),
            Vec3Make(0.0f) };
        Vec43 refNorm = { Vec3Make(0.0f), Vec3Make(1.0f), Vec3Make(0.0f) };
        Vec43 reflectedVal = Vec43Reflect(tes, refNorm);
    }


    CreateWorkers(15);
}

void Exit(void*& worldData)
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
void VECTOR_CALL RaySphereIntersectionTest(Ray r, Vec3 pos, Vec3 radiusSq, IntersectionResult& res)
{
    res.m_Dist = FLT_MAX;

    Vec3 l = pos - r.m_Pos;

    

    Vec3 tca = Dot(l, r.m_Dir);
    //if (tca > 0.0f)
    {
        Vec3 hypSq = Dot(l, l);
        Vec3 oppSq = tca * tca;
        Vec3 d = _mm_sqrt_ps(hypSq - oppSq);

        //if (d < radiusSq.m128_f32[0])
        {
            Vec3 thc = _mm_sqrt_ps(radiusSq - (d * d));
            Vec3 t0 = tca - thc;

            {
                res.m_Dist = (tca.m128_f32[0] > 0.0f) && (d.m128_f32[0] < radiusSq.m128_f32[0]) ? t0.m128_f32[0] : FLT_MAX;
                res.m_Normal = (r.m_Pos + r.m_Dir * t0) - pos;
                res.m_Normal = Vec3Normalise(res.m_Normal);
            }
        }
    }
}
#endif
#if 1
void VECTOR_CALL RaySphereIntersectionTest4(Vec43 rayPos, Vec43 rayDir, Vec3 pos, Vec3 radiusSq, Vec3& dist, Vec43& normal)
{
#if 1
    dist = _mm_set_ps1(FLT_MAX);

    Vec43 posSplatted = Vec43Make(pos);

    Vec43 l = posSplatted - rayPos;



    Vec3 tca = Vec43Dot(l, rayDir);

    Vec3 hypSq = Vec43Dot(l, l);
    Vec3 oppSq = tca * tca;
    Vec3 d = _mm_sqrt_ps(hypSq - oppSq);


    Vec3 thc = _mm_sqrt_ps(radiusSq - (d * d));
    Vec3 t0 = tca - thc;

    Vec3 selectCriteria = _mm_and_ps(_mm_cmpge_ps(tca, _mm_set_ps1(0.0f)), Vec3LT(d, radiusSq));

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
#else
void RaySphereIntersectionTest4(Vec3 pX, Vec3 pY, Vec3 pZ, Vec3 dX, Vec3 dY, Vec3 dZ, Vec3 pos, Vec3 radiusSq, Vec3& dist, Vec3& normalsX, Vec3& normalsY, Vec3& normalsZ)
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

#if 1
void VECTOR_CALL TraceRay4(Vec43 rayP, Vec43 rayD, std::vector<Vec3> const& spheres, int depth, Vec43& results)
{
    Vec3 sphereRadius = { 1.0f, 1.0f, 1.0f, 1.0f };

    Vec3 lightDir = Vec3Normalise(Vec3Make(-1.0f, -1.0f, 0.0f, 0.0f));
    lightDir = -lightDir;

    Vec43 lightColor = { Vec3Make(1.0f), Vec3Make(1.0f), Vec3Make(1.0f) };

    Vec43 lightDirExp = {
        Vec3Make(lightDir.m128_f32[0]),
        Vec3Make(lightDir.m128_f32[1]),
        Vec3Make(lightDir.m128_f32[2]) };

    Vec3 closestIntersection = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
    Vec43 closestNormal = { Vec3Make(0.0f), Vec3Make(0.0f), Vec3Make(0.0f) };

    for (size_t sphereI = 0; sphereI < spheres.size(); ++sphereI)
    {
        Vec43 normal;

        Vec3 intersectionDist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
        RaySphereIntersectionTest4(rayP, rayD, spheres[sphereI], sphereRadius, intersectionDist, normal);

        Vec3 useNewNormal = Vec3LT(intersectionDist, closestIntersection);

        closestIntersection = Vec3Select(intersectionDist, closestIntersection, useNewNormal);

        closestNormal = Vec43Select(normal, closestNormal, useNewNormal);
    }


    Vec3 litColor = Vec3Max(Vec43Dot(closestNormal, lightDirExp), Vec3Make(0.0f));
    Vec43 rayColors = lightColor * litColor + Vec43Make(Vec3Make(0.2f, 0.2f, 0.2f, 0.0f));
    Vec3 foundIntersection = Vec3LT(closestIntersection, { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX });
    results = Vec43Select(rayColors, results, foundIntersection);

    if (_mm_movemask_ps(foundIntersection) > 0 && depth > 0)
    {
        Vec43 rayDir = Vec43Reflect(rayD, closestNormal);
        Vec43 rayPos = rayP + rayD * closestIntersection;

        Vec43 reflectionRes = { Vec3Make(0.0f), Vec3Make(0.0f), Vec3Make(0.0f) };
        TraceRay4(rayPos, rayDir, spheres, depth - 1, reflectionRes);

        rayColors = rayColors + reflectionRes * Vec3Make(0.25f, 0.25f, 0.25f, 0.25f);
    }
}
#else
void TraceRay4(Vec3 rayX, Vec3 rayY, Vec3 rayZ, Vec3 dirX, Vec3 dirY, Vec3 dirZ, std::vector<Vec3> const& spheres, int depth, Vec3& resultsR, Vec3& resultsG, Vec3& resultsB)
{
    Vec3 sphereRadius = { 1.0f, 1.0f, 1.0f };

    Vec3 lightDir = Vec3Normalise(Vec3Make(-1.0f, -1.0f, 0.0f, 0.0f));
    lightDir = -lightDir;

    Vec3 lightDirX = Vec3Make(lightDir.m128_f32[0]);
    Vec3 lightDirY = Vec3Make(lightDir.m128_f32[1]);
    Vec3 lightDirZ = Vec3Make(lightDir.m128_f32[2]);

    Vec3 closestIntersection = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
    Vec3 closestIntersectionNormalX = Vec3Make(0.0f);
    Vec3 closestIntersectionNormalY = Vec3Make(0.0f);
    Vec3 closestIntersectionNormalZ = Vec3Make(0.0f);

    for (size_t sphereI = 0; sphereI < spheres.size(); ++sphereI)
    {
        Vec3 normalX;
        Vec3 normalY;
        Vec3 normalZ;

        Vec3 intersectionDist = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
        RaySphereIntersectionTest4(rayX, rayY, rayZ, dirX, dirY, dirZ, spheres[sphereI], sphereRadius, intersectionDist, normalX, normalY, normalZ);

        Vec3 useNewNormal = Vec3LT(intersectionDist, closestIntersection);

        closestIntersection = Vec3Select(intersectionDist, closestIntersection, useNewNormal);

        closestIntersectionNormalX = Vec3Select(closestIntersectionNormalX, normalX, useNewNormal);
        closestIntersectionNormalY = Vec3Select(closestIntersectionNormalY, normalY, useNewNormal);
        closestIntersectionNormalY = Vec3Select(closestIntersectionNormalZ, normalZ, useNewNormal);
    }


    Vec3 litColor = closestIntersectionNormalX * lightDirX + closestIntersectionNormalY * lightDirY + closestIntersectionNormalZ * lightDirZ;
    litColor = Vec3Max(litColor, Vec3Make(0.0f));

    Vec3 colorMask = Vec3LT(closestIntersection, { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX });

    resultsR = Vec3Select(litColor, resultsR, colorMask);
    resultsG = Vec3Select(litColor, resultsG, colorMask);
    resultsB = Vec3Select(litColor, resultsB, colorMask);

#if 0
    
    resultsR = Vec3Select(clo)

    if (closestIntersection.m_Dist < FLT_MAX)
    {
        results[0] = _mm_max_ps(Dot(closestIntersection.m_Normal, lightDir), _mm_set1_ps(0.0f));

#if 0
        // Fire a reflection ray
        if (depth >= 1 && )
        {
            Ray reflectionRay;
            reflectionRay.m_Dir = Reflect(ray.m_Dir, closestIntersection.m_Normal);
            reflectionRay.m_Pos = ray.m_Pos + ray.m_Dir * closestIntersection.m_Dist;
            //reflectionRay.m_Pos = reflectionRay.m_Pos + reflectionRay.m_Dir * 0.0001f; // Fudge factor to prevent rays from intersecting with the object fired from

            Color reflectionRes = TraceRay(reflectionRay, spheres, depth - 1);

            if (reflectionRes.m128_f32[0] > 0.0f)
            {
                result = result + reflectionRes * Vec3Make(0.25f, 0.25f, 0.25f, 0.25f);
            }

        }
#endif
    }

    return result;
#endif
}

#endif

Color VECTOR_CALL TraceRay(Ray ray, std::vector<Vec3> const& spheres, int depth)
{
    Color result = { 0.0f, 0.0f, 0.0f, 1.0f };

    Vec3 sphereRadius = { 1.0f, 1.0f, 1.0f, 1.0f };

    Vec3 lightDir = Vec3Normalise(Vec3Make(-1.0f, -1.0f, 0.0f, 0.0f));

    IntersectionResult closestIntersection;
    closestIntersection.m_Dist = FLT_MAX;

    for (size_t sphereI = 0; sphereI < spheres.size(); ++sphereI)
    {
        Vec3 colNormal;
        IntersectionResult res;
        res.m_Dist = FLT_MAX;
        RaySphereIntersectionTest(ray, spheres[sphereI], sphereRadius, res);
        if (res.m_Dist < closestIntersection.m_Dist)
        {
            closestIntersection = res;
        }
    }

    if (closestIntersection.m_Dist < FLT_MAX)
    {
        result = Dot(closestIntersection.m_Normal, lightDir);
        result = _mm_max_ps(result, _mm_set1_ps(0.0f));

#if 0
        // Fire a reflection ray
        if (depth >= 1)
        {
            Ray reflectionRay;
            reflectionRay.m_Dir = Reflect(ray.m_Dir, closestIntersection.m_Normal);
            reflectionRay.m_Pos = ray.m_Pos + ray.m_Dir * closestIntersection.m_Dist;
            //reflectionRay.m_Pos = reflectionRay.m_Pos + reflectionRay.m_Dir * 0.0001f; // Fudge factor to prevent rays from intersecting with the object fired from

            Color reflectionRes = TraceRay(reflectionRay, spheres, depth - 1);

            if (reflectionRes.m128_f32[0] > 0.0f)
            {
                result = result + reflectionRes * Vec3Make(0.25f, 0.25f, 0.25f, 0.25f);
            }

        }
#endif
    }

    return result;
}

struct RowTrace
{
    Color* m_Buffer;
    int m_Width;
    int m_Height;
    int m_Row;
    std::vector<Vec3>* spheres;
};

Concurrency::concurrent_queue<RowTrace> gRows;
std::atomic<int> gRowCount;

float xRot = 0.0f;

void TraceRow(RowTrace row)
{

    Vec3 cameraPosition = { 0.0f, 0.0f, 10.0f };

    cameraPosition = Vec3Make(sinf(xRot) * 10.0f, 0.0f, cosf(xRot) * 10.0f);

    Vec3 cameraDir = Vec3Normalise(-cameraPosition);// TransformByMatrix(Vec3Make(0.0f, 0.0f, -1.0f), CreateRotationMatrixX(xRot) * CreateRotationMatrixZ(zRot));

    Vec3 worldUp = Vec3Make(0.0f, 1.0f, 0.0f, 0.0f);
    Vec3 cameraRight = Vec3Normalise(Vec3Cross(worldUp, cameraPosition));

    Vec3 cameraUp = Vec3Cross(cameraDir, cameraRight);

    Vec3 topLeft = { -0.5f, 0.5f, 9.0f, 1.0f };
    Vec3 topRight{ 0.5f, 0.5f, 9.0f, 1.0f };

    Vec3 bottomLeft = { -0.5f, -0.5f, 9.0f, 1.0f };
    Vec3 bottomRight{ 0.5f, -0.5f, 9.0f, 1.0f };

    static bool trace4 = true;

    if (trace4)
    {
        Vec43 topLeftS = Vec43Make(topLeft);
        Vec43 topRightS = Vec43Make(topRight);
        Vec43 bottomLeftS = Vec43Make(bottomLeft);
        Vec43 bottomRightS = Vec43Make(bottomRight);

        Vec43 rayPos = Vec43Make(cameraPosition);

        Vec3 yNorm = Vec3Make(row.m_Row / (row.m_Height - 1.0f));
        for (int x = 0; x < row.m_Width; x += 4)
        {
            Vec3 xNorm = Vec3Make( x / (row.m_Width - 1.0f), (x + 1) / (row.m_Width - 1.0f), (x + 2) / (row.m_Width - 1.0f), (x + 3) / (row.m_Width - 1.0f) );

            Vec43 rayPoint = Vec43Lerp(Vec43Lerp(topLeftS, topRightS, xNorm), Vec43Lerp(bottomLeftS, bottomRightS, xNorm), yNorm);
            Vec43 rayDir = Vec43Normalise(rayPoint - rayPos);
            Vec43 color = Vec43Make(0.0f);

            TraceRay4(rayPos, rayDir, *row.spheres, 1, color);
            row.m_Buffer[row.m_Row * row.m_Width + x] = Vec3Make(color.x.m128_f32[0], color.y.m128_f32[0], color.z.m128_f32[0], 0.0f);
            row.m_Buffer[row.m_Row * row.m_Width + x + 1] = Vec3Make(color.x.m128_f32[1], color.y.m128_f32[1], color.z.m128_f32[1], 0.0f);
            row.m_Buffer[row.m_Row * row.m_Width + x + 2] = Vec3Make(color.x.m128_f32[2], color.y.m128_f32[2], color.z.m128_f32[2], 0.0f);
            row.m_Buffer[row.m_Row * row.m_Width + x + 3] = Vec3Make(color.x.m128_f32[3], color.y.m128_f32[3], color.z.m128_f32[3], 0.0f);
        }
    }
    else
    {

        float yNorm = row.m_Row / (row.m_Height - 1.0f);
        for (int x = 0; x < row.m_Width; ++x)
        {
            Ray ray;
            ray.m_Pos = cameraPosition;

            float xNorm = x / (row.m_Width - 1.0f);


            Vec3 rayPoint = Lerp(Lerp(topLeft, topRight, xNorm), Lerp(bottomLeft, bottomRight, xNorm), yNorm);
            ray.m_Dir = rayPoint - cameraPosition;
            ray.m_Dir = Vec3Normalise(ray.m_Dir);
            Color result = TraceRay(ray, *row.spheres, 1);
            row.m_Buffer[row.m_Row * row.m_Width + x] = result;
        }
    }

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

void Trace(void*& worldData, Color* buffer, int width, int height, int workerThreads)
{
    std::vector<Vec3> sphereCentre;

    Vec3 sphereBottomLeft = Vec3Make(-3.0f, -3.0f, 0.0f, 0.0f);
    Vec3 sphereBottomRight = Vec3Make(3.0f, -3.0f, 0.0f, 0.0f);
    Vec3 sphereTopLeft = Vec3Make(-3.0f, 3.0f, 0.0f, 0.0f);
    Vec3 sphereTopRight = Vec3Make(3.0f, 3.0f, 0.0f, 0.0f);

    //xRot += 0.1f;

    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            float xInterp = x / (3.0f - 1);
            float yInterp = y / (3.0f - 1);
            Vec3 sphereC = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);
            sphereC = Lerp(Lerp(sphereTopLeft, sphereTopRight, xInterp), Lerp(sphereBottomLeft, sphereBottomRight, xInterp), yInterp);
            sphereCentre.push_back(sphereC);
        }
    }
    //sphereCentre.clear();
    //sphereCentre.push_back(Vec3Make(0.0f, 0.0f, 0.0f, 0.0f));

    gRowCount = height;

    for (int y = 0; y < height; ++y)
    {
        RowTrace rowTrace;
        rowTrace.m_Height = height;
        rowTrace.m_Width = width;
        rowTrace.m_Buffer = buffer;
        rowTrace.m_Row = y;
        rowTrace.spheres = &sphereCentre;
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

void TracePixel(void*& worldData, int width, int height, int workerCount, int x, int y)
{
    Vec3 cameraPosition = { 0.0f, 0.0f, 10.0f };

    cameraPosition = Vec3Make(sinf(xRot) * 10.0f, 0.0f, cosf(xRot) * 10.0f);

    Vec3 cameraDir = Vec3Normalise(-cameraPosition);// TransformByMatrix(Vec3Make(0.0f, 0.0f, -1.0f), CreateRotationMatrixX(xRot) * CreateRotationMatrixZ(zRot));

    Vec3 worldUp = Vec3Make(0.0f, 1.0f, 0.0f, 0.0f);
    Vec3 cameraRight = Vec3Normalise(Vec3Cross(worldUp, cameraPosition));

    Vec3 cameraUp = Vec3Cross(cameraDir, cameraRight);

    Vec3 topLeft = { -0.5f, 0.5f, 9.0f, 1.0f };
    Vec3 topRight{ 0.5f, 0.5f, 9.0f, 1.0f };

    Vec3 bottomLeft = { -0.5f, -0.5f, 9.0f, 1.0f };
    Vec3 bottomRight{ 0.5f, -0.5f, 9.0f, 1.0f };


    Ray ray;
    ray.m_Pos = cameraPosition;

    std::vector<Vec3> sphereCentre;

    Vec3 sphereBottomLeft = Vec3Make(-3.0f, -3.0f, 0.0f, 0.0f);
    Vec3 sphereBottomRight = Vec3Make(3.0f, -3.0f, 0.0f, 0.0f);
    Vec3 sphereTopLeft = Vec3Make(-3.0f, 3.0f, 0.0f, 0.0f);
    Vec3 sphereTopRight = Vec3Make(3.0f, 3.0f, 0.0f, 0.0f);

    //xRot += 0.1f;

    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            float xInterp = x / (3.0f - 1);
            float yInterp = y / (3.0f - 1);
            Vec3 sphereC = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);
            sphereC = Lerp(Lerp(sphereTopLeft, sphereTopRight, xInterp), Lerp(sphereBottomLeft, sphereBottomRight, xInterp), yInterp);
            sphereCentre.push_back(sphereC);
        }
    }

    float xRounded = x / 4 * 4;

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

    Vec43 res;
    TraceRay4(rayPoint, rayDir, sphereCentre, 1, res);
}

void DebugTrace(int width, int height, int x, int y)
{
    Vec3 cameraPosition = { 0.0f, 0.0f, 10.0f };
    Vec3 cameraDir = { 0.0f, 0.0f, -1.0f };

    Vec3 topLeft = { -0.5f, 0.5f, 9.0f };
    Vec3 topRight{ 0.5f, 0.5f, 9.0f };

    Vec3 bottomLeft = { -0.5f, -0.5f, 9.0f };
    Vec3 bottomRight{ 0.5f, -0.5f, 9.0f };

    float yNorm = y / (height - 1.0f);

    Ray ray;
    ray.m_Pos = cameraPosition;

    float xNorm = x / (width - 1.0f);

    std::vector<Vec3> sphereCentre;

    Vec3 sphereBottomLeft = Vec3Make(-3.0f, -3.0f, 0.0f, 0.0f);
    Vec3 sphereBottomRight = Vec3Make(3.0f, -3.0f, 0.0f, 0.0f);
    Vec3 sphereTopLeft = Vec3Make(-3.0f, 3.0f, 0.0f, 0.0f);
    Vec3 sphereTopRight = Vec3Make(3.0f, 3.0f, 0.0f, 0.0f);

    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            float xInterp = x / (3.0f - 1);
            float yInterp = y / (3.0f - 1);
            Vec3 sphereC = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);
            sphereC = Lerp(Lerp(sphereTopLeft, sphereTopRight, xInterp), Lerp(sphereBottomLeft, sphereBottomRight, xInterp), yInterp);
            sphereCentre.push_back(sphereC);
        }
    }


    Vec3 rayPoint = Lerp(Lerp(topLeft, topRight, xNorm), Lerp(bottomLeft, bottomRight, xNorm), yNorm);
    ray.m_Dir = rayPoint - cameraPosition;
    ray.m_Dir = Vec3Normalise(ray.m_Dir);
    DebugBreak();
    Color result = TraceRay(ray, sphereCentre, 1);
}

class Core
{

};