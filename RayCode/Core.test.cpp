#ifdef _DEBUG
#include "Core.h"

#include "gtest\gtest.h"
#include "Vector.h"
#include "Intersection.h"

inline void EXPECT_VECTOR_EQ(Vec3 a, Vec3 b)
{
    EXPECT_FLOAT_EQ(a.m128_f32[0], b.m128_f32[0]);
    EXPECT_FLOAT_EQ(a.m128_f32[1], b.m128_f32[1]);
    EXPECT_FLOAT_EQ(a.m128_f32[2], b.m128_f32[2]);
}
#if 0
TEST(Matrix, CreateTranslationMatrix)
{
    Vec3 a = Vec3Make(0.0f, 0.0f, 3.0f);
    Matrix translationMatrix = CreateTranslationMatrix(a);
    Vec3 transformedVec = TransformByMatrix(a, translationMatrix);

    EXPECT_EQ(transformedVec.m128_f32[2], 6.0f);
}

TEST(Matrix, CreateRotationMatrixX)
{
    Vec3 a = Vec3Make(0.0f, 0.0f, 1.0f);
    Matrix translationMatrix = CreateRotationMatrixX(PI);
    Vec3 transformedVec = TransformByMatrix(a, translationMatrix);

    EXPECT_VECTOR_EQ(transformedVec, Vec3Make(0.0f, 0.0f, -1.0f));

}

TEST(Matrix, CreateRotationMatrixY)
{
    Vec3 a = Vec3Make(0.0f, 0.0f, 1.0f);
    Matrix translationMatrix = CreateRotationMatrixY(PI * 0.5f);
    Vec3 transformedVec = TransformByMatrix(a, translationMatrix);

    EXPECT_VECTOR_EQ(transformedVec, Vec3Make(-1.0f, 0.0f, 0.0f));
}

TEST(Matrix, CreateRotationMatrixZ)
{
    Matrix translationMatrixA = CreateTranslationMatrix(Vec3Make(0.0f, 0.0f, 1.0f));
    Matrix translationMatrixB = CreateTranslationMatrix(Vec3Make(0.0f, 3.0f, 0.0f));
    Matrix composedMatrix = translationMatrixA * translationMatrixB;
    Vec3 translation = GetTranslation(composedMatrix);
    EXPECT_VECTOR_EQ(translation, Vec3Make(0.0f, 3.0f, 1.0f));

    Vec3 transformedVec = TransformByMatrix(Vec3Make(5.0f, 0.0f, 0.0f), composedMatrix);

    EXPECT_VECTOR_EQ(transformedVec, Vec3Make(5.0f, 3.0f, 1.0f));
}
#endif

#ifdef SIMD
TEST(Vec43, Vec43Reflect)
{
    Vec43 tes = {
        Vec3Make(-1.0f, 0.0f, 1.0f, 0.0f),
        Vec3Make(0.0f, 1.0f, 0.0f, -1.0f),
        Vec3Make(0.0f) };
    Vec43 refNorm = { Vec3Make(0.0f), Vec3Make(1.0f), Vec3Make(0.0f) };
    Vec43 reflectedVal = Vec43Reflect(tes, refNorm);
}

TEST(AABB, AABBIntersect)
{
    AABB aabb;
    aabb.m_Min = Vec3Make(-1.0f, -1.0f, -1.0f);
    aabb.m_Max = Vec3Make(1.0f, 1.0f, 1.0f);

    Vec43 rayP = {
        Vec3Make(-10.0f, 10.0f, 0.0f, 0.0f),
        Vec3Make(0.0f, 10.0f, 0.0f, 0.0f),
        Vec3Make(0.0f, 0.0f, 0.0f, 0.0f) };

    Vec43 rayD = {
        Vec3Make(1.0f,-0.7071f, 1.0f, 1.0f),
        Vec3Make(0.0f,-0.7071f, 0.0f, 0.0f),
        Vec3Make(0.0f, 0.0f, 0.0f, 0.0f) };


    Vec43 rayInvD = {
        Vec3Make(1.0f) / rayD.x,
        Vec3Make(1.0f) / rayD.y,
        Vec3Make(1.0f) / rayD.z };


    Vec3 dist = Vec3Make(FLT_MAX);
    Vec43 normal;

    Vec43 rayDSign = {
        Vec3Select(_mm_set1_ps(-1.0f), _mm_set1_ps(1.0f), _mm_cmpgt_ps(rayD.x, _mm_set1_ps(0.0f))),
        Vec3Select(_mm_set1_ps(-1.0f), _mm_set1_ps(1.0f), _mm_cmpgt_ps(rayD.y, _mm_set1_ps(0.0f))),
        Vec3Select(_mm_set1_ps(-1.0f), _mm_set1_ps(1.0f), _mm_cmpgt_ps(rayD.z, _mm_set1_ps(0.0f))),
    };

    RayAABB(rayP, rayInvD, rayDSign, aabb, dist, normal);

    EXPECT_VECTOR_EQ(dist, Vec3Make(9.0f, FLT_MAX, FLT_MAX, FLT_MAX));
}
#endif

#endif