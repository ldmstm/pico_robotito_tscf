// derived from https://github.com/arkanis/single-header-file-c-libs/

#ifndef __VECTOR_MATH_H_
#define __VECTOR_MATH_H_

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct vec3_t {
  float value[3]; 
} vec3_t;

static inline vec3_t vec3(float x, float y, float z)        { return (vec3_t){ .value = {x, y, z} }; }

static inline vec3_t v3_add   (vec3_t a, vec3_t b)          { return (vec3_t){ .value = {a.value[0] + b.value[0], a.value[1] + b.value[1], a.value[2] + b.value[2]} }; }
static inline vec3_t v3_adds  (vec3_t a, float s)           { return (vec3_t){ .value = {a.value[0] + s,   a.value[1] + s,   a.value[2] + s}   }; }
static inline vec3_t v3_sub   (vec3_t a, vec3_t b)          { return (vec3_t){ .value = {a.value[0] - b.value[0], a.value[1] - b.value[1], a.value[2] - b.value[2]} }; }
static inline vec3_t v3_subs  (vec3_t a, float s)           { return (vec3_t){ .value = {a.value[0] - s,   a.value[1] - s,   a.value[2] - s}   }; }
static inline vec3_t v3_mul   (vec3_t a, vec3_t b)          { return (vec3_t){ .value = {a.value[0] * b.value[0], a.value[1] * b.value[1], a.value[2] * b.value[2]} }; }
static inline vec3_t v3_muls  (vec3_t a, float s)           { return (vec3_t){ .value = {a.value[0] * s,   a.value[1] * s,   a.value[2] * s}   }; }
static inline vec3_t v3_div   (vec3_t a, vec3_t b)          { return (vec3_t){ .value = {a.value[0] / b.value[0], a.value[1] / b.value[1], a.value[2] / b.value[2]} }; }
static inline vec3_t v3_divs  (vec3_t a, float s)           { return (vec3_t){ .value = {a.value[0] / s,   a.value[1] / s,   a.value[2] / s}   }; }
static inline float  v3_length(vec3_t v)                    { return sqrtf(v.value[0]*v.value[0] + v.value[1]*v.value[1] + v.value[2]*v.value[2]);          }
static inline vec3_t v3_norm  (vec3_t v);
static inline float  v3_dot   (vec3_t a, vec3_t b)          { return a.value[0]*b.value[0] + a.value[1]*b.value[1] + a.value[2]*b.value[2];                 }
static inline vec3_t v3_proj  (vec3_t v, vec3_t onto);
static inline vec3_t v3_cross (vec3_t a, vec3_t b);

//
// 3D vector functions header implementation
//

static inline vec3_t v3_norm(vec3_t v) {
       float len = v3_length(v);
       if (len > 0)
               return (vec3_t){ .value = {v.value[0] / len, v.value[1] / len, v.value[2] / len} };
       else
               return (vec3_t){ .value = {0, 0, 0}};
}

static inline vec3_t v3_proj(vec3_t v, vec3_t onto) {
       return v3_muls(onto, v3_dot(v, onto) / v3_dot(onto, onto));
}

static inline vec3_t v3_cross(vec3_t a, vec3_t b) {
       return (vec3_t){ .value = {
               a.value[1] * b.value[2] - a.value[2] * b.value[1],
               a.value[2] * b.value[0] - a.value[0] * b.value[2],
               a.value[0] * b.value[1] - a.value[1] * b.value[0]
       }};
}

//
// Matrix functions header implementation
//

typedef union {
       // The first index is the column index, the second the row index. The memory
       // layout of nested arrays in C matches the memory layout expected by OpenGL.
       float m[3][3];
       // OpenGL expects the first 4 floats to be the first column of the matrix.
       // So we need to define the named members column by column for the names to
       // match the memory locations of the array elements.
       struct {
               float m00, m01, m02;
               float m10, m11, m12;
               float m20, m21, m22;
       };
} mat3_t;


static inline mat3_t mat3(
       float m00, float m10, float m20,
       float m01, float m11, float m21,
       float m02, float m12, float m22
);

static inline vec3_t mat3_mul_vec3(mat3_t matrix, vec3_t position) {
       vec3_t result = vec3(
               matrix.m00 * position.value[0] + matrix.m10 * position.value[1] + matrix.m20 * position.value[2],
               matrix.m01 * position.value[0] + matrix.m11 * position.value[1] + matrix.m21 * position.value[2],
               matrix.m02 * position.value[0] + matrix.m12 * position.value[1] + matrix.m22 * position.value[2]
       );


       return result;
}

static inline mat3_t mat3(
       float m00, float m10, float m20,
       float m01, float m11, float m21,
       float m02, float m12, float m22
) 
{
  mat3_t m = { .m = {{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} };
       return m;
}


#endif //__VECTOR_MATH_H_
