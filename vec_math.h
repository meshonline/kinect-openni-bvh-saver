/** @file vec_math.h
 *  @brief 3D Linear math library
 *  @copyright Copyright (c) 2013 Kyle Weicht. All rights reserved.
 */
#ifndef __vec_math_h__
#define __vec_math_h__
#include <math.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4204) /* Nonstandard extension: non-constant \
aggregate initializer */

/* MSVC doens't define fminf and fmaxf */
#if _MSC_VER < 1800
static float fminf(float a, float b) {
    return (a < b ? a : b);
}
static float fmaxf(float a, float b) {
    return (a > b ? a : b);
}
#endif
#elif defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc99-extensions"
#endif

#ifdef __cplusplus
namespace Vec_Math {
#endif
    /**
     * Types
     */
    typedef struct Vec2 { float x, y; } Vec2;
    typedef struct Vec3 { float x, y, z; } Vec3;
    typedef struct Vec4 { float x, y, z, w; } Vec4;
    typedef struct Mat3 { Vec3 r0, r1, r2; } Mat3;
    typedef struct Mat4 { Vec4 r0, r1, r2, r3; } Mat4;
    typedef Vec4 Quaternion;
    typedef struct Transform {
        Quaternion orientation;
        Vec3 position;
        float scale;
    } Transform;
    typedef Vec4 Plane;
    typedef struct Sphere {
        Vec3 center;
        float radius;
    } Sphere;
    
    /**
     * Constants
     */
    static const float kEpsilon = 0.00001f;
    static const float kPi = 3.141592653589793238462643383280f;
    static const float k2Pi = 6.283185307179586476925286766559f;
    static const float kPiDiv2 = 1.570796326794896619231321691640f;
    static const float kInvPi = 0.318309886183790671537767526745f;
    static const float kDegToRad = 0.017453292519943295769236907685f;
    static const float kRadToDeg = 57.29577951308232087679815481410f;
    
#ifdef __cplusplus
    extern "C" {  // C linkage
        typedef const Vec2& VEC2_INPUT;
        typedef const Vec3& VEC3_INPUT;
        typedef const Vec4& VEC4_INPUT;
        typedef const Mat3& MAT3_INPUT;
        typedef const Mat4& MAT4_INPUT;
        typedef const Quaternion& QUAT_INPUT;
        typedef const Plane& PLANE_INPUT;
        typedef const Sphere& SPHERE_INPUT;
        typedef const Transform& TRANSFORM_INPUT;
#define INLINE inline
#else
        typedef Vec2 VEC2_INPUT;
        typedef Vec3 VEC3_INPUT;
        typedef Vec4 VEC4_INPUT;
        typedef Mat3 MAT3_INPUT;
        typedef Mat4 MAT4_INPUT;
        typedef Quaternion QUAT_INPUT;
        typedef Plane PLANE_INPUT;
        typedef Sphere SPHERE_INPUT;
        typedef Transform TRANSFORM_INPUT;
#define INLINE static __inline
#endif
        
        INLINE float rad_to_deg(float r) {
            return kRadToDeg * r;
        }
        INLINE float deg_to_rad(float d) {
            return kDegToRad * d;
        }
        
        INLINE void swapf(float& a, float& b) {
            float t = a;
            a = b;
            b = t;
        }
        
        INLINE float lerp(float a, float b, float t) {
            float d = b - a;
            float tt = d * t;
            return a + tt;
        }
        INLINE float saturate(float f) {
            if (f < 0)
                return 0.0f;
            if (f > 1.0f)
                return 1.0f;
            return f;
        }
        /******************************************************************************\
         * Vec2                                                                       *
         \******************************************************************************/
        static const Vec2 vec2_zero = {0.0f, 0.0f};
        INLINE Vec2 vec2_create(float x, float y) {
            Vec2 r;
            r.x = x;
            r.y = y;
            return r;
        }
        /* Basic aritmatic */
        INLINE Vec2 vec2_add(VEC2_INPUT a, VEC2_INPUT b) {
            Vec2 res;
            res.x = a.x + b.x;
            res.y = a.y + b.y;
            return res;
        }
        INLINE Vec2 vec2_sub(VEC2_INPUT a, VEC2_INPUT b) {
            Vec2 res;
            res.x = a.x - b.x;
            res.y = a.y - b.y;
            return res;
        }
        INLINE Vec2 vec2_mul(VEC2_INPUT a, VEC2_INPUT b) {
            Vec2 res;
            res.x = a.x * b.x;
            res.y = a.y * b.y;
            return res;
        }
        INLINE Vec2 vec2_div(VEC2_INPUT a, VEC2_INPUT b) {
            Vec2 res;
            res.x = a.x / b.x;
            res.y = a.y / b.y;
            return res;
        }
        
        /* Scalar math */
        INLINE Vec2 vec2_add_scalar(VEC2_INPUT v, float f) {
            Vec2 res;
            res.x = v.x + f;
            res.y = v.y + f;
            return res;
        }
        INLINE Vec2 vec2_sub_scalar(VEC2_INPUT v, float f) {
            Vec2 res;
            res.x = v.x - f;
            res.y = v.y - f;
            return res;
        }
        INLINE Vec2 vec2_mul_scalar(VEC2_INPUT v, float f) {
            Vec2 res;
            res.x = v.x * f;
            res.y = v.y * f;
            return res;
        }
        INLINE Vec2 vec2_div_scalar(VEC2_INPUT v, float f) {
            Vec2 res;
            res.x = v.x / f;
            res.y = v.y / f;
            return res;
        }
        /* Misc */
        INLINE float vec2_hadd(VEC2_INPUT v) {
            return v.x + v.y;
        }
        INLINE int vec2_equal(VEC2_INPUT a, VEC2_INPUT b) {
            return fabsf(a.x - b.x) < kEpsilon && fabsf(a.y - b.y) < kEpsilon;
        }
        INLINE int vec2_equal_scalar(VEC2_INPUT v, float f) {
            return fabsf(v.x - f) < kEpsilon && fabsf(v.y - f) < kEpsilon;
        }
        INLINE float vec2_length_sq(VEC2_INPUT v) {
            return v.x * v.x + v.y * v.y;
        }
        INLINE float vec2_length(VEC2_INPUT v) {
            return sqrtf(vec2_length_sq(v));
        }
        INLINE float vec2_distance_sq(VEC2_INPUT a, VEC2_INPUT b) {
            return vec2_length_sq(vec2_sub(a, b));
        }
        INLINE float vec2_distance(VEC2_INPUT a, VEC2_INPUT b) {
            return sqrtf(vec2_distance_sq(a, b));
        }
        INLINE Vec2 vec2_normalize(VEC2_INPUT v) {
            return vec2_div_scalar(v, vec2_length(v));
        }
        INLINE Vec2 vec2_min(VEC2_INPUT a, VEC2_INPUT b) {
            Vec2 v;
            v.x = fminf(a.x, b.x);
            v.y = fminf(a.y, b.y);
            return v;
        }
        INLINE Vec2 vec2_max(VEC2_INPUT a, VEC2_INPUT b) {
            Vec2 v;
            v.x = fmaxf(a.x, b.x);
            v.y = fmaxf(a.y, b.y);
            return v;
        }
        INLINE Vec2 vec2_lerp(VEC2_INPUT a, VEC2_INPUT b, float t) {
            Vec2 d = vec2_sub(b, a);
            Vec2 tt = vec2_mul_scalar(d, t);
            return vec2_add(a, tt);
        }
        INLINE Vec2 vec2_negate(VEC2_INPUT v) {
            Vec2 r;
            r.x = -v.x;
            r.y = -v.y;
            return r;
        }
        
        /******************************************************************************\
         * Vec3                                                                       *
         \******************************************************************************/
        static const Vec3 vec3_zero = {0.0f, 0.0f, 0.0f};
        INLINE Vec3 vec3_from_vec4(VEC4_INPUT v) {
            Vec3 r;
            r.x = v.x;
            r.y = v.y;
            r.z = v.z;
            return r;
        }
        INLINE Vec3 vec3_create(float x, float y, float z) {
            Vec3 r;
            r.x = x;
            r.y = y;
            r.z = z;
            return r;
        }
        /* Basic aritmatic */
        INLINE Vec3 vec3_add(VEC3_INPUT a, VEC3_INPUT b) {
            Vec3 res;
            res.x = a.x + b.x;
            res.y = a.y + b.y;
            res.z = a.z + b.z;
            return res;
        }
        INLINE Vec3 vec3_sub(VEC3_INPUT a, VEC3_INPUT b) {
            Vec3 res;
            res.x = a.x - b.x;
            res.y = a.y - b.y;
            res.z = a.z - b.z;
            return res;
        }
        INLINE Vec3 vec3_mul(VEC3_INPUT a, VEC3_INPUT b) {
            Vec3 res;
            res.x = a.x * b.x;
            res.y = a.y * b.y;
            res.z = a.z * b.z;
            return res;
        }
        INLINE Vec3 vec3_div(VEC3_INPUT a, VEC3_INPUT b) {
            Vec3 res;
            res.x = a.x / b.x;
            res.y = a.y / b.y;
            res.z = a.z / b.z;
            return res;
        }
        
        /* Scalar math */
        INLINE Vec3 vec3_add_scalar(VEC3_INPUT v, float f) {
            Vec3 res;
            res.x = v.x + f;
            res.y = v.y + f;
            res.z = v.z + f;
            return res;
        }
        INLINE Vec3 vec3_sub_scalar(VEC3_INPUT v, float f) {
            Vec3 res;
            res.x = v.x - f;
            res.y = v.y - f;
            res.z = v.z - f;
            return res;
        }
        INLINE Vec3 vec3_mul_scalar(VEC3_INPUT v, float f) {
            Vec3 res;
            res.x = v.x * f;
            res.y = v.y * f;
            res.z = v.z * f;
            return res;
        }
        INLINE Vec3 vec3_div_scalar(VEC3_INPUT v, float f) {
            Vec3 res;
            res.x = v.x / f;
            res.y = v.y / f;
            res.z = v.z / f;
            return res;
        }
        /* Misc */
        INLINE float vec3_hadd(VEC3_INPUT v) {
            return v.x + v.y + v.z;
        }
        INLINE int vec3_equal(VEC3_INPUT a, VEC3_INPUT b) {
            return fabsf(a.x - b.x) < kEpsilon && fabsf(a.y - b.y) < kEpsilon &&
            fabsf(a.z - b.z) < kEpsilon;
        }
        INLINE int vec3_equal_scalar(VEC3_INPUT v, float f) {
            return fabsf(v.x - f) < kEpsilon && fabsf(v.y - f) < kEpsilon &&
            fabsf(v.z - f) < kEpsilon;
        }
        INLINE float vec3_length_sq(VEC3_INPUT v) {
            return v.x * v.x + v.y * v.y + v.z * v.z;
        }
        INLINE float vec3_length(VEC3_INPUT v) {
            return sqrtf(vec3_length_sq(v));
        }
        INLINE float vec3_distance_sq(VEC3_INPUT a, VEC3_INPUT b) {
            return vec3_length_sq(vec3_sub(a, b));
        }
        INLINE float vec3_distance(VEC3_INPUT a, VEC3_INPUT b) {
            return sqrtf(vec3_distance_sq(a, b));
        }
        INLINE Vec3 vec3_normalize(VEC3_INPUT v) {
            return vec3_div_scalar(v, vec3_length(v));
        }
        INLINE Vec3 vec3_min(VEC3_INPUT a, VEC3_INPUT b) {
            Vec3 v;
            v.x = fminf(a.x, b.x);
            v.y = fminf(a.y, b.y);
            v.z = fminf(a.z, b.z);
            return v;
        }
        INLINE Vec3 vec3_max(VEC3_INPUT a, VEC3_INPUT b) {
            Vec3 v;
            v.x = fmaxf(a.x, b.x);
            v.y = fmaxf(a.y, b.y);
            v.z = fmaxf(a.z, b.z);
            return v;
        }
        INLINE Vec3 vec3_lerp(VEC3_INPUT a, VEC3_INPUT b, float t) {
            Vec3 d = vec3_sub(b, a);
            Vec3 tt = vec3_mul_scalar(d, t);
            return vec3_add(a, tt);
        }
        INLINE Vec3 vec3_negate(VEC3_INPUT v) {
            Vec3 r;
            r.x = -v.x;
            r.y = -v.y;
            r.z = -v.z;
            return r;
        }
        INLINE float vec3_dot(VEC3_INPUT a, VEC3_INPUT b) {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }
        INLINE Vec3 vec3_cross(VEC3_INPUT a, VEC3_INPUT b) {
            Vec3 r;
            r.x = a.y * b.z - a.z * b.y;
            r.y = a.z * b.x - a.x * b.z;
            r.z = a.x * b.y - a.y * b.x;
            return r;
        }
        
        /******************************************************************************\
         * Vec4                                                                       *
         \******************************************************************************/
        static const Vec4 vec4_zero = {0.0f, 0.0f, 0.0f, 0.0f};
        INLINE Vec4 vec4_from_vec3(VEC3_INPUT v, float w) {
            Vec4 r;
            r.x = v.x;
            r.y = v.y;
            r.z = v.z;
            r.w = w;
            return r;
        }
        INLINE Vec4 vec4_create(float x, float y, float z, float w) {
            Vec4 r;
            r.x = x;
            r.y = y;
            r.z = z;
            r.w = w;
            return r;
        }
        /* Basic aritmatic */
        INLINE Vec4 vec4_add(VEC4_INPUT a, VEC4_INPUT b) {
            Vec4 res;
            res.x = a.x + b.x;
            res.y = a.y + b.y;
            res.z = a.z + b.z;
            res.w = a.w + b.w;
            return res;
        }
        INLINE Vec4 vec4_sub(VEC4_INPUT a, VEC4_INPUT b) {
            Vec4 res;
            res.x = a.x - b.x;
            res.y = a.y - b.y;
            res.z = a.z - b.z;
            res.w = a.w - b.w;
            return res;
        }
        INLINE Vec4 vec4_mul(VEC4_INPUT a, VEC4_INPUT b) {
            Vec4 res;
            res.x = a.x * b.x;
            res.y = a.y * b.y;
            res.z = a.z * b.z;
            res.w = a.w * b.w;
            return res;
        }
        INLINE Vec4 vec4_div(VEC4_INPUT a, VEC4_INPUT b) {
            Vec4 res;
            res.x = a.x / b.x;
            res.y = a.y / b.y;
            res.z = a.z / b.z;
            res.w = a.w / b.w;
            return res;
        }
        
        /* Scalar math */
        INLINE Vec4 vec4_add_scalar(VEC4_INPUT v, float f) {
            Vec4 res;
            res.x = v.x + f;
            res.y = v.y + f;
            res.z = v.z + f;
            res.w = v.w + f;
            return res;
        }
        INLINE Vec4 vec4_sub_scalar(VEC4_INPUT v, float f) {
            Vec4 res;
            res.x = v.x - f;
            res.y = v.y - f;
            res.z = v.z - f;
            res.w = v.w - f;
            return res;
        }
        INLINE Vec4 vec4_mul_scalar(VEC4_INPUT v, float f) {
            Vec4 res;
            res.x = v.x * f;
            res.y = v.y * f;
            res.z = v.z * f;
            res.w = v.w * f;
            return res;
        }
        INLINE Vec4 vec4_div_scalar(VEC4_INPUT v, float f) {
            Vec4 res;
            res.x = v.x / f;
            res.y = v.y / f;
            res.z = v.z / f;
            res.w = v.w / f;
            return res;
        }
        /* Misc */
        INLINE float vec4_hadd(VEC4_INPUT v) {
            return v.x + v.y + v.z + v.w;
        }
        INLINE int vec4_equal(VEC4_INPUT a, VEC4_INPUT b) {
            return fabsf(a.x - b.x) < kEpsilon && fabsf(a.y - b.y) < kEpsilon &&
            fabsf(a.z - b.z) < kEpsilon && fabsf(a.w - b.w) < kEpsilon;
        }
        INLINE int vec4_equal_scalar(VEC4_INPUT v, float f) {
            return fabsf(v.x - f) < kEpsilon && fabsf(v.y - f) < kEpsilon &&
            fabsf(v.z - f) < kEpsilon && fabsf(v.w - f) < kEpsilon;
        }
        INLINE float vec4_length_sq(VEC4_INPUT v) {
            return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
        }
        INLINE float vec4_length(VEC4_INPUT v) {
            return sqrtf(vec4_length_sq(v));
        }
        INLINE float vec4_distance_sq(VEC4_INPUT a, VEC4_INPUT b) {
            return vec4_length_sq(vec4_sub(a, b));
        }
        INLINE float vec4_distance(VEC4_INPUT a, VEC4_INPUT b) {
            return sqrtf(vec4_distance_sq(a, b));
        }
        INLINE Vec4 vec4_normalize(VEC4_INPUT v) {
            return vec4_div_scalar(v, vec4_length(v));
        }
        INLINE Vec4 vec4_min(VEC4_INPUT a, VEC4_INPUT b) {
            Vec4 v;
            v.x = fminf(a.x, b.x);
            v.y = fminf(a.y, b.y);
            v.z = fminf(a.z, b.z);
            v.w = fminf(a.w, b.w);
            return v;
        }
        INLINE Vec4 vec4_max(VEC4_INPUT a, VEC4_INPUT b) {
            Vec4 v;
            v.x = fmaxf(a.x, b.x);
            v.y = fmaxf(a.y, b.y);
            v.z = fmaxf(a.z, b.z);
            v.w = fmaxf(a.w, b.w);
            return v;
        }
        INLINE Vec4 vec4_lerp(VEC4_INPUT a, VEC4_INPUT b, float t) {
            Vec4 d = vec4_sub(b, a);
            Vec4 tt = vec4_mul_scalar(d, t);
            return vec4_add(a, tt);
        }
        INLINE Vec4 vec4_negate(VEC4_INPUT v) {
            Vec4 r;
            r.x = -v.x;
            r.y = -v.y;
            r.z = -v.z;
            r.w = -v.w;
            return r;
        }
        
        /******************************************************************************\
         * Mat3                                                                       *
         \******************************************************************************/
        static const Mat3 mat3_identity = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        };
        INLINE Mat4 mat4_from_mat3(MAT3_INPUT m) {
            Mat4 r = {
                {m.r0.x, m.r0.y, m.r0.z, 0.0f},
                {m.r1.x, m.r1.y, m.r1.z, 0.0f},
                {m.r2.x, m.r2.y, m.r2.z, 0.0f},
                {0.0f, 0.0f, 0.0f, 1.0f},
            };
            return r;
        }
        INLINE Mat3 mat3_create(float f00,
                                float f01,
                                float f02,
                                float f10,
                                float f11,
                                float f12,
                                float f20,
                                float f21,
                                float f22) {
            Mat3 m = {
                {f00, f01, f02}, {f10, f11, f12}, {f20, f21, f22},
            };
            return m;
        }
        INLINE Mat3 mat3_scalef(float x, float y, float z) {
            Mat3 r = mat3_identity;
            r.r0.x = x;
            r.r1.y = y;
            r.r2.z = z;
            return r;
        }
        INLINE Mat3 mat3_scale(VEC3_INPUT v) {
            return mat3_scalef(v.x, v.y, v.z);
        }
        INLINE Mat3 mat3_rotation_x(float rad) {
            float c = cosf(rad);
            float s = sinf(rad);
            Mat3 r = mat3_identity;
            r.r1.y = c;
            r.r1.z = s;
            r.r2.y = -s;
            r.r2.z = c;
            return r;
        }
        INLINE Mat3 mat3_rotation_y(float rad) {
            float c = cosf(rad);
            float s = sinf(rad);
            Mat3 r = mat3_identity;
            r.r0.x = c;
            r.r0.z = -s;
            r.r2.x = s;
            r.r2.z = c;
            return r;
        }
        INLINE Mat3 mat3_rotation_z(float rad) {
            float c = cosf(rad);
            float s = sinf(rad);
            Mat3 r = mat3_identity;
            r.r0.x = c;
            r.r0.y = s;
            r.r1.x = -s;
            r.r1.y = c;
            return r;
        }
        INLINE Mat3 mat3_rotation_axis(VEC3_INPUT axis, float rad) {
            Vec3 normAxis = vec3_normalize(axis);
            float c = cosf(rad);
            float s = sinf(rad);
            float t = 1 - c;
            
            float x = normAxis.x;
            float y = normAxis.y;
            float z = normAxis.z;
            
            Mat3 m;
            
            m.r0.x = (t * x * x) + c;
            m.r0.y = (t * x * y) + s * z;
            m.r0.z = (t * x * z) - s * y;
            
            m.r1.x = (t * x * y) - (s * z);
            m.r1.y = (t * y * y) + c;
            m.r1.z = (t * y * z) + (s * x);
            
            m.r2.x = (t * x * z) + (s * y);
            m.r2.y = (t * y * z) - (s * x);
            m.r2.z = (t * z * z) + c;
            
            return m;
        }
        INLINE Mat3 mat3_mul_scalar(MAT3_INPUT m, float f) {
            Mat3 result;
            result.r0 = vec3_mul_scalar(m.r0, f);
            result.r1 = vec3_mul_scalar(m.r1, f);
            result.r2 = vec3_mul_scalar(m.r2, f);
            return result;
        }
#define MTX3_INDEX(f, r, c) ((f)[(r * 3) + c])
        INLINE Mat3 mat3_multiply(MAT3_INPUT a, MAT3_INPUT b) {
            Mat3 m = mat3_identity;
            
            const float* left = &a.r0.x;
            const float* right = &b.r0.x;
            float* result = (float*)&m;
            
            int ii, jj, kk;
            for (ii = 0; ii < 3; ++ii) /* row */
            {
                for (jj = 0; jj < 3; ++jj) /* column */
                {
                    float sum = MTX3_INDEX(left, ii, 0) * MTX3_INDEX(right, 0, jj);
                    for (kk = 1; kk < 3; ++kk) {
                        sum += (MTX3_INDEX(left, ii, kk) * MTX3_INDEX(right, kk, jj));
                    }
                    MTX3_INDEX(result, ii, jj) = sum;
                }
            }
            return m;
        }
#undef MTX3_INDEX
        INLINE float mat3_determinant(MAT3_INPUT m) {
            float f0 = m.r0.x * (m.r1.y * m.r2.z - m.r2.y * m.r1.z);
            float f1 = m.r0.y * -(m.r1.x * m.r2.z - m.r2.x * m.r1.z);
            float f2 = m.r0.z * (m.r1.x * m.r2.y - m.r2.x * m.r1.y);
            
            return f0 + f1 + f2;
        }
        INLINE Mat3 mat3_transpose(MAT3_INPUT m) {
            Mat3 result = m;
            swapf(result.r0.y, result.r1.x);
            swapf(result.r0.z, result.r2.x);
            swapf(result.r1.z, result.r2.y);
            return result;
        }
        INLINE Mat3 mat3_from_axis(VEC3_INPUT vx, VEC3_INPUT vy, VEC3_INPUT vz) {
            Mat3 m = mat3_identity;
            // YZ plane
            if (vx.x == 0 && vx.y == 0 && vx.z == 0) {
                // use y axis
                m.r1 = vec3_normalize(vy);
                m.r0 = vec3_normalize(vec3_cross(vy, vz));
                m.r2 = vec3_normalize(vec3_cross(m.r0, m.r1));
            }
            // XZ plane
            if (vy.x == 0 && vy.y == 0 && vy.z == 0) {
                // use x axis
                m.r0 = vec3_normalize(vx);
                m.r1 = vec3_normalize(vec3_cross(vz, vx));
                m.r2 = vec3_normalize(vec3_cross(m.r0, m.r1));
            }
            // XY plane
            if (vz.x == 0 && vz.y == 0 && vz.z == 0) {
                // use y axis
                m.r1 = vec3_normalize(vy);
                m.r2 = vec3_normalize(vec3_cross(vec3_normalize(vx), m.r1));
                m.r0 = vec3_normalize(vec3_cross(m.r1, m.r2));
            }
            return m;
        }
        INLINE Mat3 mat3_inverse(MAT3_INPUT m) {
            float det = mat3_determinant(m);
            Mat3 inv;
            
            inv.r0.x = (m.r1.y * m.r2.z) - (m.r1.z * m.r2.y);
            inv.r0.y = -((m.r1.x * m.r2.z) - (m.r1.z * m.r2.x));
            inv.r0.z = (m.r1.x * m.r2.y) - (m.r1.y * m.r2.x);
            
            inv.r1.x = -((m.r0.y * m.r2.z) - (m.r0.z * m.r2.y));
            inv.r1.y = (m.r0.x * m.r2.z) - (m.r0.z * m.r2.x);
            inv.r1.z = -((m.r0.x * m.r2.y) - (m.r0.y * m.r2.x));
            
            inv.r2.x = (m.r0.y * m.r1.z) - (m.r0.z * m.r1.y);
            inv.r2.y = -((m.r0.x * m.r1.z) - (m.r0.z * m.r1.x));
            inv.r2.z = (m.r0.x * m.r1.y) - (m.r0.y * m.r1.x);
            
            inv = mat3_transpose(inv);
            inv = mat3_mul_scalar(inv, 1.0f / det);
            
            return inv;
        }
        INLINE Vec3 mat3_mul_vector(VEC3_INPUT v, MAT3_INPUT m) {
            Mat3 transpose = mat3_transpose(m);
            Vec3 res, t;
            
            t = vec3_mul(transpose.r0, v);
            res.x = vec3_hadd(t);
            
            t = vec3_mul(transpose.r1, v);
            res.y = vec3_hadd(t);
            
            t = vec3_mul(transpose.r2, v);
            res.z = vec3_hadd(t);
            
            return res;
        }
        
        /******************************************************************************\
         * Mat4                                                                       *
         \******************************************************************************/
        static const Mat4 mat4_identity = {
            {1.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 1.0f},
        };
        INLINE Mat3 mat3_from_mat4(MAT4_INPUT m) {
            Mat3 r = {
                {m.r0.x, m.r0.y, m.r0.z},
                {m.r1.x, m.r1.y, m.r1.z},
                {m.r2.x, m.r2.y, m.r2.z},
            };
            return r;
        }
        INLINE Mat4 mat4_scalef(float x, float y, float z) {
            Mat4 r = mat4_identity;
            r.r0.x = x;
            r.r1.y = y;
            r.r2.z = z;
            return r;
        }
        INLINE Mat4 mat4_scale(VEC3_INPUT v) {
            return mat4_scalef(v.x, v.y, v.z);
        }
        INLINE Mat4 mat4_translatef(float x, float y, float z) {
            Mat4 r = mat4_identity;
            r.r3.x = x;
            r.r3.y = y;
            r.r3.z = z;
            return r;
        }
        INLINE Mat4 mat4_translate(VEC3_INPUT v) {
            return mat4_translatef(v.x, v.y, v.z);
        }
        INLINE Mat4 mat4_rotation_x(float rad) {
            float c = cosf(rad);
            float s = sinf(rad);
            Mat4 r = mat4_identity;
            r.r1.y = c;
            r.r1.z = s;
            r.r2.y = -s;
            r.r2.z = c;
            return r;
        }
        INLINE Mat4 mat4_rotation_y(float rad) {
            float c = cosf(rad);
            float s = sinf(rad);
            Mat4 r = mat4_identity;
            r.r0.x = c;
            r.r0.z = -s;
            r.r2.x = s;
            r.r2.z = c;
            return r;
        }
        INLINE Mat4 mat4_rotation_z(float rad) {
            float c = cosf(rad);
            float s = sinf(rad);
            Mat4 r = mat4_identity;
            r.r0.x = c;
            r.r0.y = s;
            r.r1.x = -s;
            r.r1.y = c;
            return r;
        }
        INLINE Mat4 mat4_rotation_axis(VEC3_INPUT axis, float rad) {
            Vec3 normAxis = vec3_normalize(axis);
            float c = cosf(rad);
            float s = sinf(rad);
            float t = 1 - c;
            
            float x = normAxis.x;
            float y = normAxis.y;
            float z = normAxis.z;
            
            Mat4 m = mat4_identity;
            
            m.r0.x = (t * x * x) + c;
            m.r0.y = (t * x * y) + s * z;
            m.r0.z = (t * x * z) - s * y;
            
            m.r1.x = (t * x * y) - (s * z);
            m.r1.y = (t * y * y) + c;
            m.r1.z = (t * y * z) + (s * x);
            
            m.r2.x = (t * x * z) + (s * y);
            m.r2.y = (t * y * z) - (s * x);
            m.r2.z = (t * z * z) + c;
            
            return m;
        }
        INLINE Mat4 mat4_mul_scalar(MAT4_INPUT m, float f) {
            Mat4 result;
            result.r0 = vec4_mul_scalar(m.r0, f);
            result.r1 = vec4_mul_scalar(m.r1, f);
            result.r2 = vec4_mul_scalar(m.r2, f);
            result.r3 = vec4_mul_scalar(m.r3, f);
            return result;
        }
#define MTX4_INDEX(f, r, c) ((f)[(r * 4) + c])
        INLINE Mat4 mat4_multiply(MAT4_INPUT a, MAT4_INPUT b) {
            Mat4 m = mat4_identity;
            
            const float* left = &a.r0.x;
            const float* right = &b.r0.x;
            float* result = (float*)&m;
            
            int ii, jj, kk;
            for (ii = 0; ii < 4; ++ii) /* row */
            {
                for (jj = 0; jj < 4; ++jj) /* column */
                {
                    float sum = MTX4_INDEX(left, ii, 0) * MTX4_INDEX(right, 0, jj);
                    for (kk = 1; kk < 4; ++kk) {
                        sum += (MTX4_INDEX(left, ii, kk) * MTX4_INDEX(right, kk, jj));
                    }
                    MTX4_INDEX(result, ii, jj) = sum;
                }
            }
            return m;
        }
#undef MTX4_INDEX
        INLINE float mat4_determinant(MAT4_INPUT m) {
            float det = 0.0f;
            
            Mat3 a = {{m.r1.y, m.r1.z, m.r1.w},
                {m.r2.y, m.r2.z, m.r2.w},
                {m.r3.y, m.r3.z, m.r3.w}};
            
            Mat3 b = {{m.r1.x, m.r1.z, m.r1.w},
                {m.r2.x, m.r2.z, m.r2.w},
                {m.r3.x, m.r3.z, m.r3.w}};
            
            Mat3 c = {{m.r1.x, m.r1.y, m.r1.w},
                {m.r2.x, m.r2.y, m.r2.w},
                {m.r3.x, m.r3.y, m.r3.w}};
            
            Mat3 d = {{m.r1.x, m.r1.y, m.r1.z},
                {m.r2.x, m.r2.y, m.r2.z},
                {m.r3.x, m.r3.y, m.r3.z}};
            
            det += m.r0.x * mat3_determinant(a);
            
            det -= m.r0.y * mat3_determinant(b);
            
            det += m.r0.z * mat3_determinant(c);
            
            det -= m.r0.w * mat3_determinant(d);
            
            return det;
        }
        INLINE Mat4 mat4_transpose(MAT4_INPUT m) {
            Mat4 result = m;
            swapf(result.r0.y, result.r1.x);
            swapf(result.r0.z, result.r2.x);
            swapf(result.r0.w, result.r3.x);
            swapf(result.r1.z, result.r2.y);
            swapf(result.r1.w, result.r3.y);
            swapf(result.r2.w, result.r3.z);
            return result;
        }
        INLINE Mat4 mat4_inverse(MAT4_INPUT mat) {
            Mat4 ret;
            float recip;
            
            /* temp matrices */
            
            /* row 1 */
            Mat3 a = mat3_create(mat.r1.y, mat.r1.z, mat.r1.w, mat.r2.y, mat.r2.z,
                                 mat.r2.w, mat.r3.y, mat.r3.z, mat.r3.w);
            
            Mat3 b = mat3_create(mat.r1.x, mat.r1.z, mat.r1.w, mat.r2.x, mat.r2.z,
                                 mat.r2.w, mat.r3.x, mat.r3.z, mat.r3.w);
            
            Mat3 c = mat3_create(mat.r1.x, mat.r1.y, mat.r1.w, mat.r2.x, mat.r2.y,
                                 mat.r2.w, mat.r3.x, mat.r3.y, mat.r3.w);
            
            Mat3 d = mat3_create(mat.r1.x, mat.r1.y, mat.r1.z, mat.r2.x, mat.r2.y,
                                 mat.r2.z, mat.r3.x, mat.r3.y, mat.r3.z);
            
            /* row 2 */
            Mat3 e = mat3_create(mat.r0.y, mat.r0.z, mat.r0.w, mat.r2.y, mat.r2.z,
                                 mat.r2.w, mat.r3.y, mat.r3.z, mat.r3.w);
            
            Mat3 f = mat3_create(mat.r0.x, mat.r0.z, mat.r0.w, mat.r2.x, mat.r2.z,
                                 mat.r2.w, mat.r3.x, mat.r3.z, mat.r3.w);
            
            Mat3 g = mat3_create(mat.r0.x, mat.r0.y, mat.r0.w, mat.r2.x, mat.r2.y,
                                 mat.r2.w, mat.r3.x, mat.r3.y, mat.r3.w);
            
            Mat3 h = mat3_create(mat.r0.x, mat.r0.y, mat.r0.z, mat.r2.x, mat.r2.y,
                                 mat.r2.z, mat.r3.x, mat.r3.y, mat.r3.z);
            
            /* row 3 */
            Mat3 i = mat3_create(mat.r0.y, mat.r0.z, mat.r0.w, mat.r1.y, mat.r1.z,
                                 mat.r1.w, mat.r3.y, mat.r3.z, mat.r3.w);
            
            Mat3 j = mat3_create(mat.r0.x, mat.r0.z, mat.r0.w, mat.r1.x, mat.r1.z,
                                 mat.r1.w, mat.r3.x, mat.r3.z, mat.r3.w);
            
            Mat3 k = mat3_create(mat.r0.x, mat.r0.y, mat.r0.w, mat.r1.x, mat.r1.y,
                                 mat.r1.w, mat.r3.x, mat.r3.y, mat.r3.w);
            
            Mat3 l = mat3_create(mat.r0.x, mat.r0.y, mat.r0.z, mat.r1.x, mat.r1.y,
                                 mat.r1.z, mat.r3.x, mat.r3.y, mat.r3.z);
            
            /* row 4 */
            Mat3 m = mat3_create(mat.r0.y, mat.r0.z, mat.r0.w, mat.r1.y, mat.r1.z,
                                 mat.r1.w, mat.r2.y, mat.r2.z, mat.r2.w);
            
            Mat3 n = mat3_create(mat.r0.x, mat.r0.z, mat.r0.w, mat.r1.x, mat.r1.z,
                                 mat.r1.w, mat.r2.x, mat.r2.z, mat.r2.w);
            
            Mat3 o = mat3_create(mat.r0.x, mat.r0.y, mat.r0.w, mat.r1.x, mat.r1.y,
                                 mat.r1.w, mat.r2.x, mat.r2.y, mat.r2.w);
            
            Mat3 p = mat3_create(mat.r0.x, mat.r0.y, mat.r0.z, mat.r1.x, mat.r1.y,
                                 mat.r1.z, mat.r2.x, mat.r2.y, mat.r2.z);
            
            /* row 1 */
            ret.r0.x = mat3_determinant(a);
            
            ret.r0.y = -mat3_determinant(b);
            
            ret.r0.z = mat3_determinant(c);
            
            ret.r0.w = -mat3_determinant(d);
            
            /* row 2 */
            ret.r1.x = -mat3_determinant(e);
            
            ret.r1.y = mat3_determinant(f);
            
            ret.r1.z = -mat3_determinant(g);
            
            ret.r1.w = mat3_determinant(h);
            
            /* row 3 */
            ret.r2.x = mat3_determinant(i);
            
            ret.r2.y = -mat3_determinant(j);
            
            ret.r2.z = mat3_determinant(k);
            
            ret.r2.w = -mat3_determinant(l);
            
            /* row 4 */
            ret.r3.x = -mat3_determinant(m);
            
            ret.r3.y = mat3_determinant(n);
            
            ret.r3.z = -mat3_determinant(o);
            
            ret.r3.w = mat3_determinant(p);
            
            ret = mat4_transpose(ret);
            recip = 1.0f / mat4_determinant(mat);
            ret = mat4_mul_scalar(ret, recip);
            return ret;
        }
        INLINE Vec4 mat4_mul_vector(VEC4_INPUT v, MAT4_INPUT m) {
            Mat4 transpose = mat4_transpose(m);
            Vec4 res, t;
            
            t = vec4_mul(transpose.r0, v);
            res.x = vec4_hadd(t);
            
            t = vec4_mul(transpose.r1, v);
            res.y = vec4_hadd(t);
            
            t = vec4_mul(transpose.r2, v);
            res.z = vec4_hadd(t);
            
            t = vec4_mul(transpose.r3, v);
            res.w = vec4_hadd(t);
            
            return res;
        }
        INLINE Mat4 mat4_ortho_off_center(float left,
                                          float right,
                                          float bottom,
                                          float top,
                                          float nearPlane,
                                          float farPlane) {
            Mat4 m = mat4_identity;
            
            float diff = farPlane - nearPlane;
            
            m.r0.x = 2.0f / (right - left);
            m.r1.y = 2.0f / (top - bottom);
            m.r2.z = 1.0f / diff;
            m.r3.x = -((left + right) / (right - left));
            m.r3.y = -((top + bottom) / (top - bottom));
            m.r3.z = -nearPlane / diff;
            
            return m;
        }
        INLINE Mat4 mat4_ortho(float width,
                               float height,
                               float nearPlane,
                               float farPlane) {
            float halfWidth = width / 2.0f;
            float halfHeight = height / 2.0f;
            
            return mat4_ortho_off_center(-halfWidth, halfWidth, -halfHeight, halfHeight,
                                         nearPlane, farPlane);
        }
        /** LH perspective
         */
        INLINE Mat4 mat4_perspective(float width,
                                     float height,
                                     float nearPlane,
                                     float farPlane) {
            Mat4 m = mat4_identity;
            
            m.r0.x = 2 * nearPlane / width;
            m.r1.y = 2 * nearPlane / height;
            m.r2.z = farPlane / (farPlane - nearPlane);
            m.r2.w = 1;
            m.r3.z = nearPlane * farPlane / (nearPlane - farPlane);
            m.r3.w = 0;
            return m;
        }
        /** LH perspective
         */
        INLINE Mat4 mat4_perspective_fov(float fov,
                                         float aspect,
                                         float nearPlane,
                                         float farPlane) {
            Mat4 m = mat4_identity;
            float y = 1.0f / tanf(fov / 2);
            float x = y / aspect;
            
            m.r0.x = x;
            m.r1.y = y;
            m.r2.z = farPlane / (farPlane - nearPlane);
            m.r2.w = 1;
            m.r3.z = (-nearPlane * farPlane) / (farPlane - nearPlane);
            m.r3.w = 0;
            return m;
        }
        
        /******************************************************************************\
         * Quaternion                                                                  *
         \******************************************************************************/
        static const Quaternion quat_identity = {0.0f, 0.0f, 0.0f, 1.0f};
        
        INLINE Quaternion quat_from_axis_angle(Vec3 axis, float rad) {
            Quaternion q;
            Vec3 norm = vec3_normalize(axis);
            float a = rad * 0.5f;
            float s = sinf(a);
            q.x = norm.x * s;
            q.y = norm.y * s;
            q.z = norm.z * s;
            q.w = cosf(a);
            
            return q;
        }
        INLINE Quaternion quat_from_axis_anglef(float x, float y, float z, float rad) {
            return quat_from_axis_angle(vec3_create(x, y, z), rad);
        }
        INLINE Quaternion quat_normalize(QUAT_INPUT q) {
            return vec4_normalize(q);
        }
        INLINE Mat3 quat_to_mat3(QUAT_INPUT q) {
            Quaternion norm = quat_normalize(q);
            float xx = norm.x * norm.x;
            float yy = norm.y * norm.y;
            float zz = norm.z * norm.z;
            
            float xy = norm.x * norm.y;
            float zw = norm.z * norm.w;
            float xz = norm.x * norm.z;
            float yw = norm.y * norm.w;
            float yz = norm.y * norm.z;
            float xw = norm.x * norm.w;
            
            Mat3 ret = mat3_create(1 - 2 * (yy + zz), 2 * (xy + zw), 2 * (xz - yw),
                                   2 * (xy - zw), 1 - 2 * (xx + zz), 2 * (yz + xw),
                                   2 * (xz + yw), 2 * (yz - xw), 1 - 2 * (xx + yy));
            return ret;
        }
        INLINE Quaternion quat_from_mat3(MAT3_INPUT m) {
            // this function needs colume order
            Mat3 a = mat3_transpose(m);
            
            Quaternion q;
            float trace = a.r0.x + a.r1.y + a.r2.z;
            if (trace > 0) {
                float s = 0.5f / sqrtf(trace + 1.0f);
                q.w = 0.25f / s;
                q.x = (a.r2.y - a.r1.z) * s;
                q.y = (a.r0.z - a.r2.x) * s;
                q.z = (a.r1.x - a.r0.y) * s;
            } else {
                if (a.r0.x > a.r1.y && a.r0.x > a.r2.z) {
                    float s = 2.0f * sqrtf(1.0f + a.r0.x - a.r1.y - a.r2.z);
                    q.w = (a.r2.y - a.r1.z) / s;
                    q.x = 0.25f * s;
                    q.y = (a.r0.y + a.r1.x) / s;
                    q.z = (a.r0.z + a.r2.x) / s;
                } else if (a.r1.y > a.r2.z) {
                    float s = 2.0f * sqrtf(1.0f + a.r1.y - a.r0.x - a.r2.z);
                    q.w = (a.r0.z - a.r2.x) / s;
                    q.x = (a.r0.y + a.r1.x) / s;
                    q.y = 0.25f * s;
                    q.z = (a.r1.z + a.r2.y) / s;
                } else {
                    float s = 2.0f * sqrtf(1.0f + a.r2.z - a.r0.x - a.r1.y);
                    q.w = (a.r1.x - a.r0.y) / s;
                    q.x = (a.r0.z + a.r2.x) / s;
                    q.y = (a.r1.z + a.r2.y) / s;
                    q.z = 0.25f * s;
                }
            }
            return q;
        }
        INLINE Quaternion quat_conjugate(QUAT_INPUT q) {
            Quaternion ret = {-q.x, -q.y, -q.z, q.w};
            return ret;
        }
        INLINE Quaternion quat_inverse(QUAT_INPUT q) {
            Quaternion norm = quat_normalize(q); /* Only normalized supported now */
            return quat_conjugate(norm);
        }
        INLINE Quaternion quat_left_multiply(QUAT_INPUT l, QUAT_INPUT r) {
            Quaternion q = {r.w * l.x + r.x * l.w + r.y * l.z - r.z * l.y,
                r.w * l.y + r.y * l.w + r.z * l.x - r.x * l.z,
                r.w * l.z + r.z * l.w + r.x * l.y - r.y * l.x,
                r.w * l.w - r.x * l.x - r.y * l.y - r.z * l.z};
            return q;
        }
        INLINE Quaternion quat_right_multiply(QUAT_INPUT l, QUAT_INPUT r) {
            Quaternion q = {r.w * l.x + r.x * l.w - r.y * l.z + r.z * l.y,
                r.w * l.y + r.y * l.w - r.z * l.x + r.x * l.z,
                r.w * l.z + r.z * l.w - r.x * l.y + r.y * l.x,
                r.w * l.w - r.x * l.x - r.y * l.y - r.z * l.z};
            return q;
        }
        // roll->yaw->pitch order, which roll is outer, pitch is inner.
        INLINE Quaternion quat_from_euler(float pitch, float yaw, float roll) {
            float x = pitch / 2;
            float y = yaw / 2;
            float z = roll / 2;
            
            float cx = cosf(x);
            float cy = cosf(y);
            float cz = cosf(z);
            float sx = sinf(x);
            float sy = sinf(y);
            float sz = sinf(z);
            
            Quaternion q = {
                (sx * cy * cz) - (cx * sy * sz), (cx * sy * cz) + (sx * cy * sz),
                (cx * cy * sz) - (sx * sy * cz), (cx * cy * cz) + (sx * sy * sz),
            };
            q = quat_normalize(q);
            
            return q;
        }
        // roll->yaw->pitch order, which roll is outer, pitch is inner.
        INLINE Vec3 euler_from_quat(QUAT_INPUT q) {
            Vec3 ret;
            ret.x = atan2f(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
            ret.y = asinf(2 * (q.w * q.y - q.z * q.x));
            ret.z = atan2f(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
            // fix nan value
            if (isnan(ret.x))
                ret.x = 0.f;
            if (isnan(ret.y))
                ret.y = 0.f;
            if (isnan(ret.z))
                ret.z = 0.f;
            return ret;
        }
        INLINE Vec3 quat_get_x_axis(QUAT_INPUT q) {
            Vec3 ret = {1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z),
                2 * (q.x * q.z - q.y * q.w)};
            ret = vec3_normalize(ret);
            return ret;
        }
        INLINE Vec3 quat_get_y_axis(QUAT_INPUT q) {
            Vec3 ret = {2 * (q.x * q.y - q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z),
                2 * (q.y * q.z + q.x * q.w)};
            ret = vec3_normalize(ret);
            return ret;
        }
        INLINE Vec3 quat_get_z_axis(QUAT_INPUT q) {
            Vec3 ret = {2 * (q.x * q.z + q.y * q.w), 2 * (q.y * q.z - q.x * q.w),
                1 - 2 * (q.x * q.x + q.y * q.y)};
            ret = vec3_normalize(ret);
            return ret;
        }
        INLINE Quaternion quat_between_vectors(VEC3_INPUT v1, VEC3_INPUT v2) {
            Vec3 a = vec3_normalize(v1);
            Vec3 b = vec3_normalize(v2);
            Vec3 xUnitVec3 = vec3_create(1, 0, 0);
            Vec3 yUnitVec3 = vec3_create(0, 1, 0);
            float dot = vec3_dot(a, b);
            if (dot < -0.999999f) {
                Vec3 tmpvec3 = vec3_cross(xUnitVec3, a);
                if (vec3_length(tmpvec3) < 0.000001f) {
                    tmpvec3 = vec3_cross(yUnitVec3, a);
                }
                tmpvec3 = vec3_normalize(tmpvec3);
                return quat_from_axis_angle(tmpvec3, kPi);
            } else if (dot > 0.999999f) {
                return vec4_create(0, 0, 0, 1);
            } else {
                Vec3 tmpvec3 = vec3_cross(a, b);
                return quat_normalize(
                                      vec4_create(tmpvec3.x, tmpvec3.y, tmpvec3.z, 1 + dot));
            }
        }
        INLINE double dot(QUAT_INPUT q1, QUAT_INPUT q2) {
            return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
        }
        INLINE Quaternion slerp(QUAT_INPUT q1, QUAT_INPUT q2, float t) {
            float w1;
            float w2;
            float theta = acosf(dot(q1, q2));
            float sinTheta = sinf(theta);
            if (sinTheta > 0.000001f) {
                w1 = (sinf(1.f - t) * theta) / sinTheta;
                w2 = (sinf(t) * theta) / sinTheta;
            } else {
                w1 = 1.f - t;
                w2 = t;
            }
            return vec4_create(q1.x * w1 + q2.x * w2, q1.y * w1 + q2.y * w2,
                               q1.z * w1 + q2.z * w2, q1.w * w1 + q2.w * w2);
        }
        
        /******************************************************************************\
         * Transform                                                                   *
         \******************************************************************************/
        static const Transform transform_zero = {{0, 0, 0, 1}, {0, 0, 0}, 1};
        INLINE Transform transform_lerp(TRANSFORM_INPUT a, TRANSFORM_INPUT b, float t) {
            Transform T;
            T.orientation = vec4_lerp(a.orientation, b.orientation, t);
            T.position = vec3_lerp(a.position, b.position, t);
            T.scale = lerp(a.scale, b.scale, t);
            return T;
        }
        INLINE Mat4 transform_get_matrix(TRANSFORM_INPUT t) {
            Quaternion q = t.orientation;
            float xx = q.x * q.x;
            float yy = q.y * q.y;
            float zz = q.z * q.z;
            
            float xy = q.x * q.y;
            float zw = q.z * q.w;
            float xz = q.x * q.z;
            float yw = q.y * q.w;
            float yz = q.y * q.z;
            float xw = q.x * q.w;
            
            float s = t.scale;
            
            Mat4 ret = {
                {(1 - 2 * (yy + zz)) * s, (2 * (xy + zw)) * s, (2 * (xz - yw)) * s, 0.0f},
                {(2 * (xy - zw)) * s, (1 - 2 * (xx + zz)) * s, (2 * (yz + xw)) * s, 0.0f},
                {(2 * (xz + yw)) * s, (2 * (yz - xw)) * s, (1 - 2 * (xx + yy)) * s, 0.0f},
                {t.position.x, t.position.y, t.position.z, 1.0f},
            };
            return ret;
        }
        
        /******************************************************************************\
         * Plane                                                                      *
         \******************************************************************************/
        INLINE Plane plane_from_points(VEC3_INPUT a, VEC3_INPUT b, VEC3_INPUT c) {
            Plane p;
            Vec3 side1 = vec3_sub(a, b);
            Vec3 side2 = vec3_sub(b, c);
            
            p = vec4_from_vec3(vec3_normalize(vec3_cross(side1, side2)), 0.0f);
            
            p.w = -(p.x * a.x + p.y * a.y + p.z * a.z);
            
            return p;
        }
        INLINE Plane plane_from_point_normal(VEC3_INPUT pt, VEC3_INPUT norm) {
            Vec3 N = vec3_normalize(norm);
            float D = -(N.x * pt.x + N.y * pt.y + N.z * pt.z);
            return vec4_from_vec3(N, D);
        }
        INLINE Plane plane_normalize(PLANE_INPUT p) {
            float dist = vec3_length(vec3_from_vec4(p));
            Plane r = {
                p.x / dist, p.y / dist, p.z / dist, p.w / dist,
            };
            return r;
        }
        INLINE float plane_distance_point(PLANE_INPUT p, VEC3_INPUT pt) {
            float dot = vec3_dot(pt, vec3_from_vec4(p));
            return dot + p.w;
        }
        
        /******************************************************************************\
         * Sphere                                                                     *
         \******************************************************************************/
        INLINE int sphere_plane_intersect(PLANE_INPUT p, SPHERE_INPUT s) {
            float dist = plane_distance_point(p, s.center);
            if (fabsf(dist) < s.radius)
                return 1;
            return 0;
        }
        
#ifdef __cplusplus
    }  // extern "C" {
#endif
    
#ifdef __cplusplus
}  // end of namespace Vec_Math {
#endif

#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif

#endif /* include guard */
