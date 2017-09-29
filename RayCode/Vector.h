#pragma once

#include "float.h"
#include <math.h>

float const PI = 3.14159265358979f;

typedef unsigned int u32;
typedef float f32;


#define MIN(a, b) a < b ? a : b
#define MAX(a, b) a > b ? a : b

#define SIMD

#define AVXx

#ifdef SIMD

#include <smmintrin.h>
#include <immintrin.h>

typedef __m128 Vec3;

struct AABB
{
    __m128 m_Min;
    __m128 m_Max;
};

struct Matrix
{
    Vec3 c0, c1, c2, c3;
};

struct Vec43
{
    Vec3 x, y, z;
};

#define VECTOR_CALL __vectorcall

inline Vec3 Vec3Make(float a)
{
    return _mm_set_ps1(a);
}

inline Vec3 Vec3Make(float a, float b, float c, float d)
{
    return _mm_set_ps(d, c, b, a);
}

inline Vec3 Vec3Make(float a, float b, float c)
{
    return _mm_set_ps(1.0f, c, b, a);
}

inline Vec3 operator - (Vec3 a, Vec3 b)
{
    return _mm_sub_ps(a, b);
}

inline Vec3 operator + (Vec3 a, Vec3 b)
{
    return _mm_add_ps(a, b);
}

inline Vec3 operator * (Vec3 a, Vec3 b)
{
    return _mm_mul_ps(a, b);
}

inline Vec3 operator * (Vec3 a, float b)
{
    Vec3 expanded = _mm_set1_ps(b);
    return _mm_mul_ps(a, expanded);
}

inline Vec3 operator / (Vec3 a, Vec3 b)
{
    return _mm_div_ps(a, b);
}

inline Vec3 Dot(Vec3 a, Vec3 b)
{
    int const mask = 0x77;
    return _mm_dp_ps(a, b, mask);
}

inline Vec3 Vec3Cross(Vec3 a, Vec3 b)
{
    return _mm_sub_ps(
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2))),
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)))
    );
}

inline Vec3 operator -(Vec3 a)
{
    return _mm_mul_ps(a, _mm_set1_ps(-1));
}

inline Vec3 VecLengthSq(Vec3 a)
{
    return Dot(a, a);
}

inline Vec3 Vec3Min(Vec3 a, Vec3 b)
{
    return _mm_min_ps(a, b);
}

inline Vec3 Vec3Max(Vec3 a, Vec3 b)
{
    return _mm_max_ps(a, b);
}

inline Vec3 Vec3LT(Vec3 a, Vec3 b)
{
    return _mm_cmplt_ps(a, b);
}

inline Vec3 Vec3LE(Vec3 a, Vec3 b)
{
    return _mm_cmple_ps(a, b);
}

inline Vec3 Vec3GT(Vec3 a, Vec3 b)
{
    return _mm_cmpgt_ps(a, b);
}

inline Vec3 Vec3GE(Vec3 a, Vec3 b)
{
    return _mm_cmpge_ps(a, b);
}

inline Vec3 Vec3EQ(Vec3 a, Vec3 b)
{
    return _mm_cmpeq_ps(a, b);
}

inline Vec3 Vec3And(Vec3 a, Vec3 b)
{
    return _mm_and_ps(a, b);
}

inline Vec3 Vec3Nand(Vec3 a, Vec3 b)
{
    return _mm_andnot_ps(b, a);
}

inline int Vec3MoveMask(Vec3 a)
{
    return _mm_movemask_ps(a);
}

inline Vec3 VECTOR_CALL Vec3Normalise(Vec3 a)
{
    Vec3 length = VecLengthSq(a);
    __m128 and = { 0, FLT_MAX, FLT_MIN, FLT_MIN };
    __m128 normalised = _mm_mul_ps(a, _mm_rsqrt_ps(length));
    normalised.m128_f32[3] = 1.0f;
    return normalised;
}

inline Vec3 VECTOR_CALL Vec3Sqrt(Vec3 a)
{
    return _mm_sqrt_ps(a);
}

inline Vec3 VECTOR_CALL Vec3Select(Vec3 a, Vec3 b, Vec3 mask)
{
    return Vec3And(a, mask) + Vec3Nand(b, mask);
}

inline Vec3 VECTOR_CALL Reflect(Vec3 dir, Vec3 normal)
{
    Vec3 a = Dot(dir, normal);
    Vec3 b = a * normal;
    Vec3 c = b * 2;
    Vec3 res = dir - c;
    return res;
}

inline Vec3 VECTOR_CALL Vec43Dot(Vec43 a, Vec43 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec43 VECTOR_CALL operator - (Vec43 a, Vec43 b)
{
    Vec43 res;
    res.x = a.x - b.x;
    res.y = a.y - b.y;
    res.z = a.z - b.z;
    return res;
}

inline Vec43 VECTOR_CALL operator - (Vec3 a, Vec43 b)
{
    Vec43 res;
    res.x = a - b.x;
    res.y = a - b.y;
    res.z = a - b.z;
    return res;
}

inline Vec43 VECTOR_CALL operator - (Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = a.x - b;
    res.y = a.y - b;
    res.z = a.z - b;
    return res;
}

inline Vec43 VECTOR_CALL operator * (Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = a.x * b;
    res.y = a.y * b;
    res.z = a.z * b;
    return res;
}

inline Vec43 VECTOR_CALL operator + (Vec43 a, Vec43 b)
{
    Vec43 res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;
    return res;
}

inline Vec43 VECTOR_CALL Vec43And(Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = Vec3And(a.x, b);
    res.y = Vec3And(a.y, b);
    res.z = Vec3And(a.z, b);
    return res;
}

inline Vec43 VECTOR_CALL Vec43Nand(Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = Vec3Nand(a.x, b);
    res.y = Vec3Nand(a.y, b);
    res.z = Vec3Nand(a.z, b);
    return res;
}

inline Vec43 VECTOR_CALL Vec43Select(Vec43 a, Vec43 b, Vec3 mask)
{
    return Vec43And(a, mask) + Vec43Nand(b, mask);
}


inline Vec3 VECTOR_CALL Vec43Length(Vec43 a)
{
    return Vec3Sqrt(Vec43Dot(a, a));
}

inline Vec43 VECTOR_CALL Vec43Normalise(Vec43 a)
{
    Vec3 length = Vec43Length(a);

    Vec43 res;
    res.x = a.x / length;
    res.y = a.y / length;
    res.z = a.z / length;
    return res;
}

inline Vec43 VECTOR_CALL Vec43Reflect(Vec43 dir, Vec43 normal)
{
    Vec3 a = Vec43Dot(dir, normal);
    Vec43 b = normal * a;
    Vec43 c = b * Vec3Make(2.0f);
    Vec43 res = dir - c;
    return res;
}

inline Vec43 VECTOR_CALL Vec43Make(float a, float b, float c)
{
    Vec43 vecM;
    vecM.x = Vec3Make(a);
    vecM.y = Vec3Make(b);
    vecM.z = Vec3Make(c);
    return vecM;
}


inline Vec43 VECTOR_CALL Vec43Make(float splat)
{
    return Vec43Make(splat, splat, splat);
}

inline Vec43 VECTOR_CALL Vec43Lerp(Vec43 a, Vec43 b, Vec3 t)
{
    Vec43 res;
    res.x = a.x + (b.x - a.x) * t;
    res.y = a.y + (b.y - a.y) * t;
    res.z = a.z + (b.z - a.z) * t;
    return res;
}

inline Vec3 __vectorcall TransformByMatrix(Vec3 a, Matrix m)
{
    Vec3 x = _mm_permute_ps(a, _MM_SHUFFLE(0, 0, 0, 0));
    Vec3 y = _mm_permute_ps(a, _MM_SHUFFLE(1, 1, 1, 1));
    Vec3 z = _mm_permute_ps(a, _MM_SHUFFLE(2, 2, 2, 2));
    Vec3 w = Vec3Make(1.0f);

    return _mm_add_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x, m.c0), _mm_mul_ps(y, m.c1)), _mm_mul_ps(z, m.c2)), _mm_mul_ps(w, m.c3));
}

inline Matrix __vectorcall operator * (Matrix a, Matrix b)
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

inline Vec3 __vectorcall GetTranslation(Matrix a)
{
    return a.c3;
}

inline Matrix __vectorcall CreateTranslationMatrix(Vec3 a)
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

inline Matrix __vectorcall CreateRotationMatrixX(float a)
{
    Matrix m;
    m.c0 = Vec3Make(1.0f, 0.0f, 0.0f, 0.0f);
    m.c1 = Vec3Make(0.0f, cosf(a), -sinf(a), 0.0f);
    m.c2 = Vec3Make(0.0f, sinf(a), cosf(a), 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}


inline Matrix __vectorcall CreateRotationMatrixY(float a)
{
    Matrix m;
    m.c0 = Vec3Make(cosf(a), 0.0f, sinf(a), 0.0f);
    m.c1 = Vec3Make(0.0f, 1.0f, 0.0f, 0.0f);
    m.c2 = Vec3Make(-sinf(a), 0.0f, cosf(a), 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}


inline Matrix __vectorcall CreateRotationMatrixZ(float a)
{
    Matrix m;
    m.c0 = Vec3Make(cosf(a), -sinf(a), 0.0f, 0.0f);
    m.c1 = Vec3Make(sinf(a), cosf(a), 0.0f, 0.0f);
    m.c2 = Vec3Make(0.0f, 0.0f, 1.0f, 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}

#else
struct Vec3
{
    union
    {
        float m128_f32[3];

        struct
        {
            float x;
            float y;
            float z;
        };
    };
    
};

struct AABB
{
    Vec3 m_Min;
    Vec3 m_Max;
};

struct Matrix
{
    Vec3 c0, c1, c2, c3;
};

struct Vec43
{
    Vec3 x, y, z;
};

#define VECTOR_CALL __vectorcall

inline Vec3 Vec3Make(float a)
{
    return { a, a, a };
}

/*
inline Vec3 Vec3Make(float a, float b, float c, float d)
{
    return{ a, b, c, d };
}*/

inline Vec3 Vec3Make(float a, float b, float c)
{
    return{ a, b, c };
}

inline Vec3 operator - (Vec3 a, Vec3 b)
{
    return{ a.x - b.x, a.y - b.y, a.z - b.z };
}

inline Vec3 operator + (Vec3 a, Vec3 b)
{
    return{ a.x + b.x, a.y + b.y, a.z + b.z };
}

inline Vec3 operator * (Vec3 a, Vec3 b)
{
    return { a.x * b.x, a.y * b.y, a.z * b.z };
}

inline Vec3 operator * (Vec3 a, float b)
{
    return { a.x * b, a.y * b, a.z * b };
}

inline Vec3 operator / (Vec3 a, Vec3 b)
{
    return { a.x / b.x, a.y / b.y, a.z / b.z };
}

inline float Dot(Vec3 a, Vec3 b)
{

    return { a.x * b.x + a.y * b.y + a.z * b.z };
}

inline Vec3 Vec3Cross(Vec3 a, Vec3 b)
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z , a.x * b.y - b.x * a.y };
}

inline Vec3 operator -(Vec3 a)
{
    return { -a.x, -a.y, -a.z };
}

inline float VecLengthSq(Vec3 a)
{
    return Dot(a, a);
}

inline Vec3 Vec3Min(Vec3 a, Vec3 b)
{
    return{ a.x < b.x ? a.x : b.x,
        a.y < b.y ? a.y : b.y,
        a.z < b.z ? a.z : b.z };
}

inline Vec3 Vec3Max(Vec3 a, Vec3 b)
{
    return{ a.x > b.x ? a.x : b.x,
        a.y > b.y ? a.y : b.y,
        a.z > b.z ? a.z : b.z };
}

inline Vec3 VECTOR_CALL Vec3Normalise(Vec3 a)
{
    float lengthSq = VecLengthSq(a);
    float invLength = 1.0f / sqrtf(lengthSq);
    return a * invLength;
}

inline Vec3 VECTOR_CALL Vec3Reflect(Vec3 dir, Vec3 normal)
{
    float a = Dot(dir, normal);
    Vec3 b = normal * a;
    Vec3 c = b * 2;
    Vec3 res = dir - c;
    return res;
}

#if 0
inline Vec3 VECTOR_CALL Vec43Dot(Vec43 a, Vec43 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec43 VECTOR_CALL operator - (Vec43 a, Vec43 b)
{
    Vec43 res;
    res.x = a.x - b.x;
    res.y = a.y - b.y;
    res.z = a.z - b.z;
    return res;
}

inline Vec43 VECTOR_CALL operator - (Vec3 a, Vec43 b)
{
    Vec43 res;
    res.x = a - b.x;
    res.y = a - b.y;
    res.z = a - b.z;
    return res;
}

inline Vec43 VECTOR_CALL operator - (Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = a.x - b;
    res.y = a.y - b;
    res.z = a.z - b;
    return res;
}

inline Vec43 VECTOR_CALL operator * (Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = a.x * b;
    res.y = a.y * b;
    res.z = a.z * b;
    return res;
}

inline Vec43 VECTOR_CALL operator + (Vec43 a, Vec43 b)
{
    Vec43 res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;
    return res;
}

inline Vec43 VECTOR_CALL Vec43And(Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = Vec3And(a.x, b);
    res.y = Vec3And(a.y, b);
    res.z = Vec3And(a.z, b);
    return res;
}

inline Vec43 VECTOR_CALL Vec43Nand(Vec43 a, Vec3 b)
{
    Vec43 res;
    res.x = Vec3Nand(a.x, b);
    res.y = Vec3Nand(a.y, b);
    res.z = Vec3Nand(a.z, b);
    return res;
}

inline Vec43 VECTOR_CALL Vec43Select(Vec43 a, Vec43 b, Vec3 mask)
{
    return Vec43And(a, mask) + Vec43Nand(b, mask);
}


inline Vec3 VECTOR_CALL Vec43Length(Vec43 a)
{
    return Vec3Sqrt(Vec43Dot(a, a));
}

inline Vec43 VECTOR_CALL Vec43Normalise(Vec43 a)
{
    Vec3 length = Vec43Length(a);

    Vec43 res;
    res.x = a.x / length;
    res.y = a.y / length;
    res.z = a.z / length;
    return res;
}

inline Vec43 VECTOR_CALL Vec43Reflect(Vec43 dir, Vec43 normal)
{
    Vec3 a = Vec43Dot(dir, normal);
    Vec43 b = normal * a;
    Vec43 c = b * Vec3Make(2.0f);
    Vec43 res = dir - c;
    return res;
}

inline Vec43 VECTOR_CALL Vec43Make(Vec3 splat)
{
    Vec43 vecM;
    vecM.x = Vec3Make(splat.m128_f32[0]);
    vecM.y = Vec3Make(splat.m128_f32[1]);
    vecM.z = Vec3Make(splat.m128_f32[2]);
    return vecM;
}

inline Vec43 VECTOR_CALL Vec43Make(float splat)
{
    return Vec43Make(Vec3Make(splat));
}

inline Vec43 VECTOR_CALL Vec43Lerp(Vec43 a, Vec43 b, Vec3 t)
{
    Vec43 res;
    res.x = a.x + (b.x - a.x) * t;
    res.y = a.y + (b.y - a.y) * t;
    res.z = a.z + (b.z - a.z) * t;
    return res;
}

inline Vec3 __vectorcall TransformByMatrix(Vec3 a, Matrix m)
{
    Vec3 x = _mm_permute_ps(a, _MM_SHUFFLE(0, 0, 0, 0));
    Vec3 y = _mm_permute_ps(a, _MM_SHUFFLE(1, 1, 1, 1));
    Vec3 z = _mm_permute_ps(a, _MM_SHUFFLE(2, 2, 2, 2));
    Vec3 w = Vec3Make(1.0f);

    return _mm_add_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x, m.c0), _mm_mul_ps(y, m.c1)), _mm_mul_ps(z, m.c2)), _mm_mul_ps(w, m.c3));
}

inline Matrix __vectorcall operator * (Matrix a, Matrix b)
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

inline Vec3 __vectorcall GetTranslation(Matrix a)
{
    return a.c3;
}

inline Matrix __vectorcall CreateTranslationMatrix(Vec3 a)
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

inline Matrix __vectorcall CreateRotationMatrixX(float a)
{
    Matrix m;
    m.c0 = Vec3Make(1.0f, 0.0f, 0.0f, 0.0f);
    m.c1 = Vec3Make(0.0f, cosf(a), -sinf(a), 0.0f);
    m.c2 = Vec3Make(0.0f, sinf(a), cosf(a), 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}


inline Matrix __vectorcall CreateRotationMatrixY(float a)
{
    Matrix m;
    m.c0 = Vec3Make(cosf(a), 0.0f, sinf(a), 0.0f);
    m.c1 = Vec3Make(0.0f, 1.0f, 0.0f, 0.0f);
    m.c2 = Vec3Make(-sinf(a), 0.0f, cosf(a), 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}


inline Matrix __vectorcall CreateRotationMatrixZ(float a)
{
    Matrix m;
    m.c0 = Vec3Make(cosf(a), -sinf(a), 0.0f, 0.0f);
    m.c1 = Vec3Make(sinf(a), cosf(a), 0.0f, 0.0f);
    m.c2 = Vec3Make(0.0f, 0.0f, 1.0f, 0.0f);
    m.c3 = Vec3Make(0.0f, 0.0f, 0.0f, 0.0f);

    return m;
}
#endif
#endif

inline Vec3 Lerp(Vec3 a, Vec3 b, float t)
{
    return a + (b - a) * t;
}

inline Vec3 Lerp(Vec3 a, Vec3 b, Vec3 t)
{
    return a + (b - a) * t;
}

#ifdef AVX

typedef __m256 Vec8;

struct Vec83
{
    __m256 x;
    __m256 y;
    __m256 z;
};


inline Vec8 Vec8Make(float a)
{
    return _mm256_set1_ps(a);
}

inline Vec8 Vec8Make(float a, float b, float c, float d, float e, float f, float g, float h)
{
    return _mm256_set_ps(h, g, f, e, d, c, b, a);
}

inline Vec8 operator - (Vec8 a, Vec8 b)
{
    return _mm256_sub_ps(a, b);
}

inline Vec8 operator + (Vec8 a, Vec8 b)
{
    return _mm256_add_ps(a, b);
}

inline Vec8 operator * (Vec8 a, Vec8 b)
{
    return _mm256_mul_ps(a, b);
}

inline Vec8 operator * (Vec8 a, float b)
{
    Vec8 expanded = _mm256_set1_ps(b);
    return _mm256_mul_ps(a, expanded);
}

inline Vec8 operator / (Vec8 a, Vec8 b)
{
    return _mm256_div_ps(a, b);
}

inline Vec8 Dot(Vec8 a, Vec8 b)
{
    int const mask = 0x77;
    return _mm256_dp_ps(a, b, mask);
}

inline Vec8 operator -(Vec8 a)
{
    return _mm256_mul_ps(a, _mm256_set1_ps(-1));
}

inline Vec8 VecLengthSq(Vec8 a)
{
    return Dot(a, a);
}

inline Vec8 Vec8Min(Vec8 a, Vec8 b)
{
    return _mm256_min_ps(a, b);
}

inline Vec8 Vec8Max(Vec8 a, Vec8 b)
{
    return _mm256_max_ps(a, b);
}

inline Vec8 Vec8LT(Vec8 a, Vec8 b)
{
    return _mm256_cmp_ps(a, b, _CMP_LT_OQ);
}

inline Vec8 Vec8GT(Vec8 a, Vec8 b)
{
    return _mm256_cmp_ps(a, b, _CMP_GT_OQ);
}

inline Vec8 Vec8EQ(Vec8 a, Vec8 b)
{
    return _mm256_cmp_ps(a, b, _CMP_EQ_OQ);
}

inline Vec8 Vec8And(Vec8 a, Vec8 b)
{
    return _mm256_and_ps(a, b);
}

inline Vec8 Vec8Nand(Vec8 a, Vec8 b)
{
    return _mm256_andnot_ps(b, a);
}

inline int Vec8MoveMask(Vec8 a)
{
    return _mm256_movemask_ps(a);
}

inline Vec8 VECTOR_CALL Vec8Sqrt(Vec8 a)
{
    return _mm256_sqrt_ps(a);
}

inline Vec8 VECTOR_CALL Vec8Select(Vec8 a, Vec8 b, Vec8 mask)
{
    return Vec8And(a, mask) + Vec8Nand(b, mask);
}

inline Vec8 VECTOR_CALL Reflect(Vec8 dir, Vec8 normal)
{
    Vec8 a = Dot(dir, normal);
    Vec8 b = a * normal;
    Vec8 c = b * 2;
    Vec8 res = dir - c;
    return res;
}

inline Vec8 VECTOR_CALL Vec83Dot(Vec83 a, Vec83 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec83 VECTOR_CALL operator - (Vec83 a, Vec83 b)
{
    Vec83 res;
    res.x = a.x - b.x;
    res.y = a.y - b.y;
    res.z = a.z - b.z;
    return res;
}

inline Vec83 VECTOR_CALL operator - (Vec8 a, Vec83 b)
{
    Vec83 res;
    res.x = a - b.x;
    res.y = a - b.y;
    res.z = a - b.z;
    return res;
}

inline Vec83 VECTOR_CALL operator - (Vec83 a, Vec8 b)
{
    Vec83 res;
    res.x = a.x - b;
    res.y = a.y - b;
    res.z = a.z - b;
    return res;
}

inline Vec83 VECTOR_CALL operator * (Vec83 a, Vec8 b)
{
    Vec83 res;
    res.x = a.x * b;
    res.y = a.y * b;
    res.z = a.z * b;
    return res;
}

inline Vec83 VECTOR_CALL operator + (Vec83 a, Vec83 b)
{
    Vec83 res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;
    return res;
}

inline Vec83 VECTOR_CALL Vec83And(Vec83 a, Vec8 b)
{
    Vec83 res;
    res.x = Vec8And(a.x, b);
    res.y = Vec8And(a.y, b);
    res.z = Vec8And(a.z, b);
    return res;
}

inline Vec83 VECTOR_CALL Vec83Nand(Vec83 a, Vec8 b)
{
    Vec83 res;
    res.x = Vec8Nand(a.x, b);
    res.y = Vec8Nand(a.y, b);
    res.z = Vec8Nand(a.z, b);
    return res;
}

inline Vec83 VECTOR_CALL Vec83Select(Vec83 a, Vec83 b, Vec8 mask)
{
    return Vec83And(a, mask) + Vec83Nand(b, mask);
}

inline Vec8 VECTOR_CALL Vec83Length(Vec83 a)
{
    return Vec8Sqrt(Vec83Dot(a, a));
}

inline Vec83 VECTOR_CALL Vec83Normalise(Vec83 a)
{
    Vec8 length = Vec83Length(a);

    Vec83 res;
    res.x = a.x / length;
    res.y = a.y / length;
    res.z = a.z / length;
    return res;
}

inline Vec83 VECTOR_CALL Vec83Reflect(Vec83 dir, Vec83 normal)
{
    Vec8 a = Vec83Dot(dir, normal);
    Vec83 b = normal * a;
    Vec83 c = b * Vec8Make(2.0f);
    Vec83 res = dir - c;
    return res;
}

inline Vec83 VECTOR_CALL Vec83Make(float a, float b, float c)
{
    Vec83 vecM;
    vecM.x = Vec8Make(a);
    vecM.y = Vec8Make(b);
    vecM.z = Vec8Make(c);
    return vecM;
}


inline Vec83 VECTOR_CALL Vec83Make(float splat)
{
    return Vec83Make(splat, splat, splat);
}

inline Vec83 VECTOR_CALL Vec83Lerp(Vec83 a, Vec83 b, Vec8 t)
{
    Vec83 res;
    res.x = a.x + (b.x - a.x) * t;
    res.y = a.y + (b.y - a.y) * t;
    res.z = a.z + (b.z - a.z) * t;
    return res;
}

#endif

#ifdef AVX

typedef Vec83 WVec;
typedef Vec8 NVec;

#define NVecMake(a) Vec8Make(a)
#define NVecNormalise(a) Vec8Normalise(a)
#define NVecLT(a, b) Vec8LT(a, b)
#define NVecGT(a, b) Vec8GT(a, b)
#define NVecEQ(a, b) Vec8EQ(a, b)
#define NVecSqrt(a) Vec8Sqrt(a)
#define NVecSelect(a, b, mask) Vec8Select(a, b, mask)
#define NVecMoveMask(a) Vec8MoveMask(a)

#define NVecMax(a, b) Vec8Max(a, b)
#define NVecMin(a, b) Vec8Min(a, b)

#define WVecMake(a) Vec83Make(a)
#define WVecMake2(a, b, c) Vec83Make(a, b, c)

#define WVecSelect(a, b, mask) Vec83Select(a, b, mask)

#define WVecDot(a, b) Vec83Dot(a, b)
#define WVecReflect(a, norm) Vec83Reflect(a, norm)

#define WVecLerp(a, b, t) Vec83Lerp(a, b, t)

#define WVecNormalise(a) Vec83Normalise(a)

#else

typedef Vec43 WVec;
typedef Vec3 NVec;

#define NVecMake(a) Vec3Make(a)
#define NVecNormalise(a) Vec3Normalise(a)
#define NVecLT(a, b) Vec3LT(a, b)
#define NVecGT(a, b) Vec3GT(a, b)
#define NVecEQ(a, b) Vec3EQ(a, b)
#define NVecSqrt(a) Vec3Sqrt(a)
#define NVecSelect(a, b, mask) Vec3Select(a, b, mask)
#define NVecMoveMask(a) Vec3MoveMask(a)

#define NVecMax(a, b) Vec3Max(a, b)
#define NVecMin(a, b) Vec3Min(a, b)

#define WVecMake(a) Vec43Make(a)
#define WVecMake2(a, b, c) Vec43Make(a, b, c)

#define WVecSelect(a, b, mask) Vec43Select(a, b, mask)

#define WVecDot(a, b) Vec43Dot(a, b)
#define WVecReflect(a, norm) Vec43Reflect(a, norm)

#define WVecLerp(a, b, t) Vec43Lerp(a, b, t)

#define WVecNormalise(a) Vec43Normalise(a)

#endif

struct ABB
{
    Vec3 min, max;
};