#ifndef __MMD_MATH_H__
#define __MMD_MATH_H__

#include <math.h>

namespace mmd {

struct Vector3 {
  float x, y, z;
};

// quat has double precision
struct Quaternion {
  double x, y, z, w;
};

static inline void VSub(Vector3 &c, const Vector3 &a, const Vector3 &b) {
  c.x = a.x - b.x;
  c.y = a.y - b.y;
  c.z = a.z - b.z;
}

static inline float VDot(const Vector3 &a, const Vector3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline float VLength(const Vector3 &a) {
  return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

static inline void VCross(Vector3 &c, const Vector3 &a, const Vector3 &b) {
  c.x = a.y * b.z - b.y * a.z;
  c.y = a.z * b.x - b.z * a.x;
  c.z = a.x * b.y - b.x * a.y;
}

static inline void VNormalize(Vector3 &v) {
  float len = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

  if (len > 1.0e-5f) {
    float inv_len = 1.0f / len;
    v.x = v.x * inv_len;
    v.y = v.y * inv_len;
    v.z = v.z * inv_len;
  }
}

static void M44Invert(float m[16]) {
  /*
   * codes from intel web
   * cramer's rule version
   */
  int i, j;
  float tmp[12];  /* tmp array for pairs */
  float tsrc[16]; /* array of transpose source matrix */
  float det;      /* determinant */

  /* transpose matrix */
  for (i = 0; i < 4; i++) {
    tsrc[i] = m[4 * i + 0];
    tsrc[i + 4] = m[4 * i + 1];
    tsrc[i + 8] = m[4 * i + 2];
    tsrc[i + 12] = m[4 * i + 3];
  }

  /* calculate pair for first 8 elements(cofactors) */
  tmp[0] = tsrc[10] * tsrc[15];
  tmp[1] = tsrc[11] * tsrc[14];
  tmp[2] = tsrc[9] * tsrc[15];
  tmp[3] = tsrc[11] * tsrc[13];
  tmp[4] = tsrc[9] * tsrc[14];
  tmp[5] = tsrc[10] * tsrc[13];
  tmp[6] = tsrc[8] * tsrc[15];
  tmp[7] = tsrc[11] * tsrc[12];
  tmp[8] = tsrc[8] * tsrc[14];
  tmp[9] = tsrc[10] * tsrc[12];
  tmp[10] = tsrc[8] * tsrc[13];
  tmp[11] = tsrc[9] * tsrc[12];

  /* calculate first 8 elements(cofactors) */
  m[0] = tmp[0] * tsrc[5] + tmp[3] * tsrc[6] + tmp[4] * tsrc[7];
  m[0] -= tmp[1] * tsrc[5] + tmp[2] * tsrc[6] + tmp[5] * tsrc[7];
  m[1] = tmp[1] * tsrc[4] + tmp[6] * tsrc[6] + tmp[9] * tsrc[7];
  m[1] -= tmp[0] * tsrc[4] + tmp[7] * tsrc[6] + tmp[8] * tsrc[7];
  m[2] = tmp[2] * tsrc[4] + tmp[7] * tsrc[5] + tmp[10] * tsrc[7];
  m[2] -= tmp[3] * tsrc[4] + tmp[6] * tsrc[5] + tmp[11] * tsrc[7];
  m[3] = tmp[5] * tsrc[4] + tmp[8] * tsrc[5] + tmp[11] * tsrc[6];
  m[3] -= tmp[4] * tsrc[4] + tmp[9] * tsrc[5] + tmp[10] * tsrc[6];
  m[4] = tmp[1] * tsrc[1] + tmp[2] * tsrc[2] + tmp[5] * tsrc[3];
  m[4] -= tmp[0] * tsrc[1] + tmp[3] * tsrc[2] + tmp[4] * tsrc[3];
  m[5] = tmp[0] * tsrc[0] + tmp[7] * tsrc[2] + tmp[8] * tsrc[3];
  m[5] -= tmp[1] * tsrc[0] + tmp[6] * tsrc[2] + tmp[9] * tsrc[3];
  m[6] = tmp[3] * tsrc[0] + tmp[6] * tsrc[1] + tmp[11] * tsrc[3];
  m[6] -= tmp[2] * tsrc[0] + tmp[7] * tsrc[1] + tmp[10] * tsrc[3];
  m[7] = tmp[4] * tsrc[0] + tmp[9] * tsrc[1] + tmp[10] * tsrc[2];
  m[7] -= tmp[5] * tsrc[0] + tmp[8] * tsrc[1] + tmp[11] * tsrc[2];

  /* calculate pairs for second 8 elements(cofactors) */
  tmp[0] = tsrc[2] * tsrc[7];
  tmp[1] = tsrc[3] * tsrc[6];
  tmp[2] = tsrc[1] * tsrc[7];
  tmp[3] = tsrc[3] * tsrc[5];
  tmp[4] = tsrc[1] * tsrc[6];
  tmp[5] = tsrc[2] * tsrc[5];
  tmp[6] = tsrc[0] * tsrc[7];
  tmp[7] = tsrc[3] * tsrc[4];
  tmp[8] = tsrc[0] * tsrc[6];
  tmp[9] = tsrc[2] * tsrc[4];
  tmp[10] = tsrc[0] * tsrc[5];
  tmp[11] = tsrc[1] * tsrc[4];

  /* calculate second 8 elements(cofactors) */
  m[8] = tmp[0] * tsrc[13] + tmp[3] * tsrc[14] + tmp[4] * tsrc[15];
  m[8] -= tmp[1] * tsrc[13] + tmp[2] * tsrc[14] + tmp[5] * tsrc[15];
  m[9] = tmp[1] * tsrc[12] + tmp[6] * tsrc[14] + tmp[9] * tsrc[15];
  m[9] -= tmp[0] * tsrc[12] + tmp[7] * tsrc[14] + tmp[8] * tsrc[15];
  m[10] = tmp[2] * tsrc[12] + tmp[7] * tsrc[13] + tmp[10] * tsrc[15];
  m[10] -= tmp[3] * tsrc[12] + tmp[6] * tsrc[13] + tmp[11] * tsrc[15];
  m[11] = tmp[5] * tsrc[12] + tmp[8] * tsrc[13] + tmp[11] * tsrc[14];
  m[11] -= tmp[4] * tsrc[12] + tmp[9] * tsrc[13] + tmp[10] * tsrc[14];
  m[12] = tmp[2] * tsrc[10] + tmp[5] * tsrc[11] + tmp[1] * tsrc[9];
  m[12] -= tmp[4] * tsrc[11] + tmp[0] * tsrc[9] + tmp[3] * tsrc[10];
  m[13] = tmp[8] * tsrc[11] + tmp[0] * tsrc[8] + tmp[7] * tsrc[10];
  m[13] -= tmp[6] * tsrc[10] + tmp[9] * tsrc[11] + tmp[1] * tsrc[8];
  m[14] = tmp[6] * tsrc[9] + tmp[11] * tsrc[11] + tmp[3] * tsrc[8];
  m[14] -= tmp[10] * tsrc[11] + tmp[2] * tsrc[8] + tmp[7] * tsrc[9];
  m[15] = tmp[10] * tsrc[10] + tmp[4] * tsrc[8] + tmp[9] * tsrc[9];
  m[15] -= tmp[8] * tsrc[9] + tmp[11] * tsrc[0] + tmp[5] * tsrc[8];

  /* calculate determinant */
  det = tsrc[0] * m[0] + tsrc[1] * m[1] + tsrc[2] * m[2] + tsrc[3] * m[3];

  /* calculate matrix inverse */
  det = 1.0f / det;

  for (j = 0; j < 4; j++) {
    for (i = 0; i < 4; i++) {
      m[4 * j + i] *= det;
    }
  }
}

static inline void AxisToQuat(Quaternion &q,
                              const Vector3 &a, // Assume normalized
                              float phi) {
  float s = sin(phi / 2.0);

  q.x = a.x * s;
  q.y = a.y * s;
  q.z = a.z * s;

  q.w = cos(phi / 2.0);
}

static inline void QNormalize(Quaternion &q) {
  float mag;
  double tmp;
  tmp = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

  mag = (float)sqrt(tmp);

  if (mag > 1.0e-5f) {
    mag = 1.0f / mag;
    q.x *= mag;
    q.y *= mag;
    q.z *= mag;
    q.w *= mag;
  }
}

static inline void QMult(Quaternion &q, const Quaternion &a,
                         const Quaternion &b) {
  Vector3 p;

  // scalar component
  q.w = (a.w * b.w) - (a.x * b.x + a.y * b.y + a.z * b.z);

  // cross product
  p.x = a.y * b.z - a.z * b.y;
  p.y = a.z * b.x - a.x * b.z;
  p.z = a.x * b.y - a.y * b.x;

  q.x = (a.w * b.x) + (b.w * a.x) + p.x;
  q.y = (a.w * b.y) + (b.w * a.y) + p.y;
  q.z = (a.w * b.z) + (b.w * a.z) + p.z;

  QNormalize(q);
}

static inline void QSlerp(Quaternion &p, const Quaternion &q,
                          const Quaternion &r, double t) {
  double qdotr = q.x * r.x + q.y * r.y + q.z * r.z + q.w * r.w;
  // Clamp to prevent NaN. But is this OK?
  if (qdotr > 1.0)
    qdotr = 1.0;
  if (qdotr < -1.0)
    qdotr = -1.0;

  if (qdotr < 0.0) {
    double theta = acos(-qdotr);

    if (fabs(theta) < 1.0e-10) {
      p = q;
      return;
    }

    double st = sin(theta);
    double inv_st = 1.0 / st;
    double c0 = sin((1.0 - t) * theta) * inv_st;
    double c1 = sin(t * theta) * inv_st;

    p.x = c0 * q.x - c1 * r.x;
    p.y = c0 * q.y - c1 * r.y;
    p.z = c0 * q.z - c1 * r.z;
    p.w = c0 * q.w - c1 * r.w;

  } else {

    double theta = acos(qdotr);

    if (fabs(theta) < 1.0e-10) {
      p = q;
      return;
    }

    double st = sin(theta);
    double inv_st = 1.0 / st;
    double c0 = sin((1.0 - t) * theta) * inv_st;
    double c1 = sin(t * theta) * inv_st;

    p.x = c0 * q.x + c1 * r.x;
    p.y = c0 * q.y + c1 * r.y;
    p.z = c0 * q.z + c1 * r.z;
    p.w = c0 * q.w + c1 * r.w;
  }
}

static inline void MatMul(float dst[16], float m0[16], float m1[16]) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      dst[4 * i + j] = 0;
      for (int k = 0; k < 4; ++k) {
        dst[4 * i + j] += m0[4 * k + j] * m1[4 * i + k];
      }
    }
  }
}

static inline void MatVMul(Vector3 &w, float m[16], const Vector3 &v) {
  w.x = m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12];
  w.y = m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13];
  w.z = m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14];
};

static inline void QuatToMatrix(float m[16], const Quaternion &q) {
  double x2 = q.x * q.x * 2.0f;
  double y2 = q.y * q.y * 2.0f;
  double z2 = q.z * q.z * 2.0f;
  double xy = q.x * q.y * 2.0f;
  double yz = q.y * q.z * 2.0f;
  double zx = q.z * q.x * 2.0f;
  double xw = q.x * q.w * 2.0f;
  double yw = q.y * q.w * 2.0f;
  double zw = q.z * q.w * 2.0f;

  m[0] = (float)(1.0f - y2 - z2);
  m[1] = (float)(xy + zw);
  m[2] = (float)(zx - yw);
  m[3] = 0.0f;

  m[4] = (float)(xy - zw);
  m[5] = (float)(1.0f - z2 - x2);
  m[6] = (float)(yz + xw);
  m[7] = 0.0f;

  m[8] = (float)(zx + yw);
  m[9] = (float)(yz - xw);
  m[10] = (float)(1.0f - x2 - y2);
  m[11] = 0.0f;

  m[12] = 0.0f;
  m[13] = 0.0f;
  m[14] = 0.0f;
  m[15] = 1.0f;
}
};

#endif // __MMD_MATH_H__
