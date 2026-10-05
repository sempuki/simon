// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/articulated_convex.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <vector>

// A port of MuJoCo 3.14.0's native convex collision, engine_collision_gjk.c
// and engine_collision_convex.c (Apache-2.0), for primitives, in its order
// of operations. It keeps MuJoCo's arrays of three numbers and its names,
// so the two read side by side.
namespace simon::model {

namespace {

constexpr double MINVAL = 1e-15;             // mjMINVAL.
constexpr double MINVAL2 = MINVAL * MINVAL;  // mjMINVAL2.
constexpr double MAXVAL = 1e10;              // mjMAXVAL.
constexpr double MAXVAL2 = MAXVAL * MAXVAL;  // mjMAXVAL2.
constexpr double MAX_LIMIT = DBL_MAX;        // mjMAX_LIMIT.
constexpr double FACE_TOL = 0.996;           // mjFACE_TOL, about 5.1 deg.
constexpr double EDGE_TOL = 0.0888;          // mjEDGE_TOL.
constexpr double MINDIST2 = MINVAL2;         // mjMINDIST2.
constexpr double MINDIST3 = MINVAL2;         // mjMINDIST3.
constexpr double MINDIST4 = MINVAL2;         // mjMINDIST4.
constexpr double MINEPATOL = MINVAL;         // mjMINEPATOL.
constexpr int CCD_ITERATIONS = 35;           // mjOption's ccd_iterations.
constexpr double CCD_TOLERANCE = 1e-6;       // mjOption's ccd_tolerance.
constexpr int MAXCONPAIR = 50;               // mjMAXCONPAIR.

//-- Vectors -------------------------------------------------------------------

inline void add3(double res[3], const double v1[3], const double v2[3]) {
  res[0] = v1[0] + v2[0], res[1] = v1[1] + v2[1], res[2] = v1[2] + v2[2];
}
inline void sub3(double res[3], const double v1[3], const double v2[3]) {
  res[0] = v1[0] - v2[0], res[1] = v1[1] - v2[1], res[2] = v1[2] - v2[2];
}
inline auto dot3(const double v1[3], const double v2[3]) -> double {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
inline auto norm3(const double v[3]) -> double { return std::sqrt(dot3(v, v)); }
inline void copy3(double res[3], const double v[3]) {
  res[0] = v[0], res[1] = v[1], res[2] = v[2];
}
inline void scl3(double res[3], const double v[3], double s) {
  res[0] = s * v[0], res[1] = s * v[1], res[2] = s * v[2];
}
inline void addScl3(double res[3], const double v1[3], const double v2[3],
                    double s) {
  res[0] = v1[0] + s * v2[0], res[1] = v1[1] + s * v2[1],
  res[2] = v1[2] + s * v2[2];
}
inline void cross3(double res[3], const double v1[3], const double v2[3]) {
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}
inline auto det3(const double v1[3], const double v2[3], const double v3[3])
    -> double {
  return v1[0] * (v2[1] * v3[2] - v2[2] * v3[1]) +
         v1[1] * (v2[2] * v3[0] - v2[0] * v3[2]) +
         v1[2] * (v2[0] * v3[1] - v2[1] * v3[0]);
}
// mju_normalize3.
inline auto normalize3(double v[3]) -> double {
  double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (norm < MINVAL) {
    v[0] = 1, v[1] = 0, v[2] = 0;
  } else {
    double inverse = 1 / norm;
    v[0] *= inverse, v[1] *= inverse, v[2] *= inverse;
  }
  return norm;
}
inline void mulMatVec3(double res[3], const double m[9], const double v[3]) {
  res[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
  res[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
  res[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}
inline void mulMatTVec3(double res[3], const double m[9], const double v[3]) {
  res[0] = m[0] * v[0] + m[3] * v[1] + m[6] * v[2];
  res[1] = m[1] * v[0] + m[4] * v[1] + m[7] * v[2];
  res[2] = m[2] * v[0] + m[5] * v[1] + m[8] * v[2];
}
inline void localToGlobal(double res[3], const double mat[9],
                          const double dir[3], const double pos[3]) {
  res[0] = mat[0] * dir[0] + mat[1] * dir[1] + mat[2] * dir[2];
  res[1] = mat[3] * dir[0] + mat[4] * dir[1] + mat[5] * dir[2];
  res[2] = mat[6] * dir[0] + mat[7] * dir[1] + mat[8] * dir[2];
  res[0] += pos[0];
  res[1] += pos[1];
  res[2] += pos[2];
}
inline void globalcoord(double res[3], const double mat[9], const double* pos,
                        double l1, double l2, double l3) {
  res[0] = mat[0] * l1 + mat[1] * l2 + mat[2] * l3;
  res[1] = mat[3] * l1 + mat[4] * l2 + mat[5] * l3;
  res[2] = mat[6] * l1 + mat[7] * l2 + mat[8] * l3;
  if (pos != nullptr) {
    res[0] += pos[0];
    res[1] += pos[1];
    res[2] += pos[2];
  }
}

//-- Objects and their support functions (engine_collision_convex.c) ----------

struct Object;
using Support = void (*)(double res[3], Object* obj, const double dir[3]);

// A geom as the convex collider sees it (mjCCDObj).
struct Object final {
  GeomType type = GeomType::SPHERE;
  double size[3]{};
  double pos[3]{};
  double mat[9]{};
  double margin = 0.0;
  int vertindex = -1;
  Support support = nullptr;
};

void pointSupport(double res[3], Object* obj, const double[3]) {
  copy3(res, obj->pos);
}

void sphereSupport(double res[3], Object* obj, const double dir[3]) {
  double radius = obj->size[0];
  res[0] = radius * dir[0] + obj->pos[0];
  res[1] = radius * dir[1] + obj->pos[1];
  res[2] = radius * dir[2] + obj->pos[2];
}

void lineSupport(double res[3], Object* obj, const double dir[3]) {
  const double* mat = obj->mat;
  double length = obj->size[1];
  double dot = mat[2] * dir[0] + mat[5] * dir[1] + mat[8] * dir[2];
  double scl = dot >= 0 ? length : -length;
  res[0] = mat[2] * scl + obj->pos[0];
  res[1] = mat[5] * scl + obj->pos[1];
  res[2] = mat[8] * scl + obj->pos[2];
}

void capsuleSupport(double res[3], Object* obj, const double dir[3]) {
  double radius = obj->size[0];
  double length = obj->size[1];
  double local_dir[3];
  double local_supp[3];
  mulMatTVec3(local_dir, obj->mat, dir);
  local_supp[0] = local_dir[0] * radius;
  local_supp[1] = local_dir[1] * radius;
  local_supp[2] = local_dir[2] * radius;
  local_supp[2] += (local_dir[2] >= 0 ? length : -length);
  localToGlobal(res, obj->mat, local_supp, obj->pos);
}

void ellipsoidSupport(double res[3], Object* obj, const double dir[3]) {
  const double* mat = obj->mat;
  const double* size = obj->size;
  double local_dir[3];
  double local_supp[3];
  mulMatTVec3(local_dir, mat, dir);
  local_supp[0] = local_dir[0] * size[0];
  local_supp[1] = local_dir[1] * size[1];
  local_supp[2] = local_dir[2] * size[2];
  double norm2 = local_supp[0] * local_supp[0] + local_supp[1] * local_supp[1] +
                 local_supp[2] * local_supp[2];
  if (norm2 < MINVAL2) {
    res[0] = mat[0] * size[0] + obj->pos[0];
    res[1] = mat[3] * size[0] + obj->pos[1];
    res[2] = mat[6] * size[0] + obj->pos[2];
    return;
  }
  double norm_inv = 1 / std::sqrt(norm2);
  local_supp[0] *= norm_inv * size[0];
  local_supp[1] *= norm_inv * size[1];
  local_supp[2] *= norm_inv * size[2];
  localToGlobal(res, mat, local_supp, obj->pos);
}

void cylinderSupport(double res[3], Object* obj, const double dir[3]) {
  const double* size = obj->size;
  double local_dir[3];
  double local_supp[3];
  mulMatTVec3(local_dir, obj->mat, dir);
  double n2 = local_dir[0] * local_dir[0] + local_dir[1] * local_dir[1];
  double scl = n2 >= MINVAL2 ? size[0] / std::sqrt(n2) : 0;
  local_supp[0] = scl * local_dir[0];
  local_supp[1] = scl * local_dir[1];
  local_supp[2] = local_dir[2] >= 0 ? size[1] : -size[1];
  obj->vertindex = local_dir[2] >= 0 ? 0 : 1;
  localToGlobal(res, obj->mat, local_supp, obj->pos);
}

void boxSupport(double res[3], Object* obj, const double dir[3]) {
  const double* size = obj->size;
  double local_dir[3];
  double local_supp[3];
  mulMatTVec3(local_dir, obj->mat, dir);
  local_supp[0] = local_dir[0] >= 0 ? size[0] : -size[0];
  local_supp[1] = local_dir[1] >= 0 ? size[1] : -size[1];
  local_supp[2] = local_dir[2] >= 0 ? size[2] : -size[2];
  obj->vertindex = (local_supp[0] > 0) ? 1 : 0;
  obj->vertindex |= (local_supp[1] > 0) ? 2 : 0;
  obj->vertindex |= (local_supp[2] > 0) ? 4 : 0;
  localToGlobal(res, obj->mat, local_supp, obj->pos);
}

// mjc_initCCDObj, for primitives.
auto make_object(const Geom& geom, const GeomFrame& frame, double margin)
    -> Object {
  Object obj;
  obj.type = geom.type;
  obj.margin = margin;
  for (int k = 0; k < 3; ++k) {
    obj.size[k] = geom.size[k];
    obj.pos[k] = frame.pos[k];
  }
  for (int k = 0; k < 9; ++k) {
    obj.mat[k] = frame.mat[k];
  }
  switch (geom.type) {
    case GeomType::ELLIPSOID:
      obj.support = ellipsoidSupport;
      break;
    case GeomType::SPHERE:
      obj.support = sphereSupport;
      break;
    case GeomType::CAPSULE:
      obj.support = capsuleSupport;
      break;
    case GeomType::CYLINDER:
      obj.support = cylinderSupport;
      break;
    case GeomType::BOX:
      obj.support = boxSupport;
      break;
    default:
      obj.support = nullptr;
      break;
  }
  return obj;
}

//-- GJK and EPA (engine_collision_gjk.c) --------------------------------------

// A vertex of the Minkowski difference, and the points of each geom it is.
struct Vertex final {
  double vert[3]{};
  double vert1[3]{};
  double vert2[3]{};
  int index1 = 0;
  int index2 = 0;
};

enum EpaStatus : int {
  EPA_NOCONTACT = -1,
  EPA_SUCCESS = 0,
  EPA_P2_INVALID_FACES,
  EPA_P2_NONCONVEX,
  EPA_P2_ORIGIN_ON_FACE,
  EPA_P3_BAD_NORMAL,
  EPA_P3_INVALID_V4,
  EPA_P3_INVALID_V5,
  EPA_P3_MISSING_ORIGIN,
  EPA_P3_ORIGIN_ON_FACE,
  EPA_P4_MISSING_ORIGIN,
};

// mjCCDStatus.
struct Status final {
  int separated = 0;
  double dist[MAXCONPAIR]{};
  double x1[3 * MAXCONPAIR]{};
  double x2[3 * MAXCONPAIR]{};
  int nx = 0;
  int max_iterations = 0;
  double tolerance = 0.0;
  int max_contacts = 0;
  double dist_cutoff = 0.0;
  int gjk_iterations = 0;
  int epa_iterations = 0;
  int epa_status = EPA_NOCONTACT;
  Vertex simplex[4];
  int nsimplex = 0;
};

struct Face final {
  int verts = 0;
  int adj[3]{};
  double v[3]{};
  double dist2 = 0.0;
  int index = 0;
};

inline void expand(int out[3], int n) {
  out[0] = n & 0x3FF;
  out[1] = (n >> 10) & 0x3FF;
  out[2] = (n >> 20) & 0x3FF;
}

struct Polytope final {
  std::vector<Vertex> verts;
  int nverts = 0;
  std::vector<Face> faces;
  int nfaces = 0;
  int maxfaces = 0;
  double center[3]{};
  std::vector<Face*> map;
  int nmap = 0;
  int horizon_indices[24]{};
  int horizon_edges[24]{};
  int nedges = 0;
  const double* horizon_w = nullptr;
};

auto discreteGeoms(const Object* obj1, const Object* obj2) -> bool {
  if (obj1->margin != 0 || obj2->margin != 0) {
    return false;
  }
  return obj1->type == GeomType::BOX && obj2->type == GeomType::BOX;
}

inline void lincomb(double res[3], const double* coef, int n,
                    const double v1[3], const double v2[3], const double v3[3],
                    const double v4[3]) {
  switch (n) {
    case 0:
      res[0] = res[1] = res[2] = 0;
      break;
    case 1:
      res[0] = coef[0] * v1[0];
      res[1] = coef[0] * v1[1];
      res[2] = coef[0] * v1[2];
      break;
    case 2:
      res[0] = coef[0] * v1[0] + coef[1] * v2[0];
      res[1] = coef[0] * v1[1] + coef[1] * v2[1];
      res[2] = coef[0] * v1[2] + coef[1] * v2[2];
      break;
    case 3:
      res[0] = coef[0] * v1[0] + coef[1] * v2[0] + coef[2] * v3[0];
      res[1] = coef[0] * v1[1] + coef[1] * v2[1] + coef[2] * v3[1];
      res[2] = coef[0] * v1[2] + coef[1] * v2[2] + coef[2] * v3[2];
      break;
    case 4:
      res[0] =
          coef[0] * v1[0] + coef[1] * v2[0] + coef[2] * v3[0] + coef[3] * v4[0];
      res[1] =
          coef[0] * v1[1] + coef[1] * v2[1] + coef[2] * v3[1] + coef[3] * v4[1];
      res[2] =
          coef[0] * v1[2] + coef[1] * v2[2] + coef[2] * v3[2] + coef[3] * v4[2];
      break;
    default:
      break;
  }
}

auto projectOriginPlane(double res[3], const double v1[3], const double v2[3],
                        const double v3[3]) -> int {
  double diff21[3];
  double diff31[3];
  double diff32[3];
  double n[3];
  sub3(diff21, v2, v1);
  sub3(diff31, v3, v1);
  sub3(diff32, v3, v2);
  cross3(n, diff32, diff21);
  double nv = dot3(n, v2);
  double nn = dot3(n, n);
  if (nn == 0) {
    return 1;
  }
  if (nv != 0 && nn > MINVAL) {
    scl3(res, n, nv / nn);
    return 0;
  }
  cross3(n, diff21, diff31);
  nv = dot3(n, v1);
  nn = dot3(n, n);
  if (nn == 0) {
    return 1;
  }
  if (nv != 0 && nn > MINVAL) {
    scl3(res, n, nv / nn);
    return 0;
  }
  cross3(n, diff31, diff32);
  nv = dot3(n, v3);
  nn = dot3(n, n);
  scl3(res, n, nv / nn);
  return 0;
}

inline void projectOriginLine(double res[3], const double v1[3],
                              const double v2[3]) {
  double diff[3];
  sub3(diff, v2, v1);
  double scl = -(dot3(v2, diff) / dot3(diff, diff));
  res[0] = v2[0] + scl * diff[0];
  res[1] = v2[1] + scl * diff[1];
  res[2] = v2[2] + scl * diff[2];
}

inline auto sameSign2(double a, double b) -> int {
  if (a > 0 && b > 0) {
    return 1;
  }
  if (a < 0 && b < 0) {
    return -1;
  }
  return 0;
}

void S1D(double lambda[2], const double s1[3], const double s2[3]) {
  double p_o[3];
  projectOriginLine(p_o, s1, s2);
  double mu = s1[0] - s2[0];
  double mu_max = mu;
  int index = 0;
  mu = s1[1] - s2[1];
  if (std::abs(mu) >= std::abs(mu_max)) {
    mu_max = mu;
    index = 1;
  }
  mu = s1[2] - s2[2];
  if (std::abs(mu) >= std::abs(mu_max)) {
    mu_max = mu;
    index = 2;
  }
  double c1 = p_o[index] - s2[index];
  double c2 = s1[index] - p_o[index];
  int same = sameSign2(mu_max, c1) && sameSign2(mu_max, c2);
  lambda[0] = same ? c1 / mu_max : 0;
  lambda[1] = same ? c2 / mu_max : 1;
}

void S2D(double lambda[3], const double s1[3], const double s2[3],
         const double s3[3]) {
  double p_o[3];
  if (projectOriginPlane(p_o, s1, s2, s3)) {
    S1D(lambda, s1, s2);
    lambda[2] = 0;
    return;
  }
  double m14 = s2[1] * s3[2] - s2[2] * s3[1] - s1[1] * s3[2] + s1[2] * s3[1] +
               s1[1] * s2[2] - s1[2] * s2[1];
  double m24 = s2[0] * s3[2] - s2[2] * s3[0] - s1[0] * s3[2] + s1[2] * s3[0] +
               s1[0] * s2[2] - s1[2] * s2[0];
  double m34 = s2[0] * s3[1] - s2[1] * s3[0] - s1[0] * s3[1] + s1[1] * s3[0] +
               s1[0] * s2[1] - s1[1] * s2[0];
  double m_max = 0;
  double a[2];
  double b[2];
  double c[2];
  double p[2];
  double mu1 = std::abs(m14);
  double mu2 = std::abs(m24);
  double mu3 = std::abs(m34);
  int x = 0;
  int y = 1;
  if (mu1 >= mu2 && mu1 >= mu3) {
    m_max = m14;
    x = 1;
    y = 2;
  } else if (mu2 >= mu3) {
    m_max = m24;
    x = 0;
    y = 2;
  } else {
    m_max = m34;
    x = 0;
    y = 1;
  }
  a[0] = s1[x], a[1] = s1[y];
  b[0] = s2[x], b[1] = s2[y];
  c[0] = s3[x], c[1] = s3[y];
  p[0] = p_o[x], p[1] = p_o[y];
  double c31 = p[0] * b[1] + p[1] * c[0] + b[0] * c[1] - p[0] * c[1] -
               p[1] * b[0] - c[0] * b[1];
  double c32 = p[0] * c[1] + p[1] * a[0] + c[0] * a[1] - p[0] * a[1] -
               p[1] * c[0] - a[0] * c[1];
  double c33 = p[0] * a[1] + p[1] * b[0] + a[0] * b[1] - p[0] * b[1] -
               p[1] * a[0] - b[0] * a[1];
  int comp1 = sameSign2(m_max, c31);
  int comp2 = sameSign2(m_max, c32);
  int comp3 = sameSign2(m_max, c33);
  if (comp1 && comp2 && comp3) {
    lambda[0] = c31 / m_max;
    lambda[1] = c32 / m_max;
    lambda[2] = c33 / m_max;
    return;
  }
  double dmin = MAX_LIMIT;
  if (!comp1) {
    double l[2];
    double v[3];
    S1D(l, s2, s3);
    lincomb(v, l, 2, s2, s3, nullptr, nullptr);
    double d = dot3(v, v);
    lambda[0] = 0;
    lambda[1] = l[0];
    lambda[2] = l[1];
    dmin = d;
  }
  if (!comp2) {
    double l[2];
    double v[3];
    S1D(l, s1, s3);
    lincomb(v, l, 2, s1, s3, nullptr, nullptr);
    double d = dot3(v, v);
    if (d < dmin) {
      lambda[0] = l[0];
      lambda[1] = 0;
      lambda[2] = l[1];
      dmin = d;
    }
  }
  if (!comp3) {
    double l[2];
    double v[3];
    S1D(l, s1, s2);
    lincomb(v, l, 2, s1, s2, nullptr, nullptr);
    double d = dot3(v, v);
    if (d < dmin) {
      lambda[0] = l[0];
      lambda[1] = l[1];
      lambda[2] = 0;
    }
  }
}

void S3D(double lambda[4], const double s1[3], const double s2[3],
         const double s3[3], const double s4[3]) {
  double c41 = -det3(s2, s3, s4);
  double c42 = det3(s1, s3, s4);
  double c43 = -det3(s1, s2, s4);
  double c44 = det3(s1, s2, s3);
  double m_det = c41 + c42 + c43 + c44;
  int comp1 = sameSign2(m_det, c41);
  int comp2 = sameSign2(m_det, c42);
  int comp3 = sameSign2(m_det, c43);
  int comp4 = sameSign2(m_det, c44);
  if (comp1 && comp2 && comp3 && comp4) {
    lambda[0] = c41 / m_det;
    lambda[1] = c42 / m_det;
    lambda[2] = c43 / m_det;
    lambda[3] = c44 / m_det;
    return;
  }
  double dmin = MAX_LIMIT;
  if (!comp1) {
    double l[3];
    double x[3];
    S2D(l, s2, s3, s4);
    lincomb(x, l, 3, s2, s3, s4, nullptr);
    double d = dot3(x, x);
    lambda[0] = 0;
    lambda[1] = l[0];
    lambda[2] = l[1];
    lambda[3] = l[2];
    dmin = d;
  }
  if (!comp2) {
    double l[3];
    double x[3];
    S2D(l, s1, s3, s4);
    lincomb(x, l, 3, s1, s3, s4, nullptr);
    double d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = l[0];
      lambda[1] = 0;
      lambda[2] = l[1];
      lambda[3] = l[2];
      dmin = d;
    }
  }
  if (!comp3) {
    double l[3];
    double x[3];
    S2D(l, s1, s2, s4);
    lincomb(x, l, 3, s1, s2, s4, nullptr);
    double d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = l[0];
      lambda[1] = l[1];
      lambda[2] = 0;
      lambda[3] = l[2];
      dmin = d;
    }
  }
  if (!comp4) {
    double l[3];
    double x[3];
    S2D(l, s1, s2, s3);
    lincomb(x, l, 3, s1, s2, s3, nullptr);
    double d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = l[0];
      lambda[1] = l[1];
      lambda[2] = l[2];
      lambda[3] = 0;
    }
  }
}

inline void subdistance(double lambda[4], int n, const Vertex simplex[4]) {
  lambda[0] = lambda[1] = lambda[2] = lambda[3] = 0;
  switch (n) {
    case 4:
      S3D(lambda, simplex[0].vert, simplex[1].vert, simplex[2].vert,
          simplex[3].vert);
      break;
    case 3:
      S2D(lambda, simplex[0].vert, simplex[1].vert, simplex[2].vert);
      break;
    case 2:
      S1D(lambda, simplex[0].vert, simplex[1].vert);
      break;
    default:
      lambda[0] = 1;
      break;
  }
}

// S_{A-B}(dir), each geom's support grown by half its margin.
inline void support(Vertex* v, Object* obj1, Object* obj2, const double dir[3],
                    const double dir_neg[3]) {
  obj1->support(v->vert1, obj1, dir);
  if (obj1->margin > 0) {
    double margin = 0.5 * obj1->margin;
    v->vert1[0] += dir[0] * margin;
    v->vert1[1] += dir[1] * margin;
    v->vert1[2] += dir[2] * margin;
  }
  obj2->support(v->vert2, obj2, dir_neg);
  if (obj2->margin > 0) {
    double margin = 0.5 * obj2->margin;
    v->vert2[0] += dir_neg[0] * margin;
    v->vert2[1] += dir_neg[1] * margin;
    v->vert2[2] += dir_neg[2] * margin;
  }
  sub3(v->vert, v->vert1, v->vert2);
  v->index1 = obj1->vertindex;
  v->index2 = obj2->vertindex;
}

inline void gjkSupport(Vertex* v, Object* obj1, Object* obj2,
                       const double x_k[3], double x_norm) {
  double dir[3];
  double dir_neg[3];
  scl3(dir_neg, x_k, 1 / x_norm);
  scl3(dir, dir_neg, -1);
  support(v, obj1, obj2, dir, dir_neg);
}

auto epaSupport(Polytope* pt, Object* obj1, Object* obj2, const double d[3],
                double dnorm) -> int {
  double dir[3] = {1, 0, 0};
  double dir_neg[3] = {-1, 0, 0};
  if (dnorm > MINVAL) {
    dir[0] = d[0] / dnorm;
    dir[1] = d[1] / dnorm;
    dir[2] = d[2] / dnorm;
    scl3(dir_neg, dir, -1);
  }
  int n = pt->nverts++;
  Vertex* v = &pt->verts[n];
  support(v, obj1, obj2, dir, dir_neg);
  return n;
}

void gjkIntersectSupport(Vertex* v, Object* obj1, Object* obj2,
                         const double dir[3]) {
  double dir_neg[3] = {-dir[0], -dir[1], -dir[2]};
  support(v, obj1, obj2, dir, dir_neg);
}

inline auto signedDistance(double normal[3], const Vertex* v1, const Vertex* v2,
                           const Vertex* v3) -> double {
  double diff1[3];
  double diff2[3];
  sub3(diff1, v3->vert, v1->vert);
  sub3(diff2, v2->vert, v1->vert);
  cross3(normal, diff1, diff2);
  double norm2 = dot3(normal, normal);
  if (norm2 > MINVAL2 && norm2 < MAXVAL2) {
    scl3(normal, normal, 1 / std::sqrt(norm2));
    return dot3(normal, v1->vert);
  }
  return MAX_LIMIT;
}

// 1 if in contact, 0 if not, -1 if inconclusive.
auto gjkIntersect(Status* status, Object* obj1, Object* obj2) -> int {
  Vertex simplex[4] = {status->simplex[0], status->simplex[1],
                       status->simplex[2], status->simplex[3]};
  int s[4] = {0, 1, 2, 3};
  int k = status->gjk_iterations;
  int kmax = status->max_iterations;
  for (; k < kmax; k++) {
    double dist[4];
    double normals[12];
    dist[0] = signedDistance(&normals[0], simplex + s[2], simplex + s[1],
                             simplex + s[3]);
    dist[1] = signedDistance(&normals[3], simplex + s[0], simplex + s[2],
                             simplex + s[3]);
    dist[2] = signedDistance(&normals[6], simplex + s[1], simplex + s[0],
                             simplex + s[3]);
    dist[3] = signedDistance(&normals[9], simplex + s[0], simplex + s[1],
                             simplex + s[2]);
    if (!dist[3] || !dist[2] || !dist[1] || !dist[0]) {
      status->gjk_iterations = k;
      return -1;
    }
    int i = (dist[0] < dist[1]) ? 0 : 1;
    int j = (dist[2] < dist[3]) ? 2 : 3;
    int index = (dist[i] < dist[j]) ? i : j;
    if (dist[index] > 0) {
      status->nsimplex = 4;
      for (int c = 0; c < 4; ++c) {
        status->simplex[c] = simplex[s[c]];
      }
      status->gjk_iterations = k;
      return 1;
    }
    gjkIntersectSupport(simplex + s[index], obj1, obj2, normals + 3 * index);
    if (dot3(&normals[3 * index], simplex[s[index]].vert) < 0) {
      status->nsimplex = 0;
      status->gjk_iterations = k;
      return 0;
    }
    i = (index + 1) & 3;
    j = (index + 2) & 3;
    std::swap(s[i], s[j]);
  }
  status->gjk_iterations = k;
  return -1;
}

void gjk(Status* status, Object* obj1, Object* obj2) {
  int get_dist = status->dist_cutoff > 0;
  int backup_gjk = !get_dist;
  Vertex* simplex = status->simplex;
  int n = 0;
  int k = 0;
  int kmax = status->max_iterations;
  double* x1_k = status->x1;
  double* x2_k = status->x2;
  double x_k[3];
  double lambda[4] = {0, 0, 0, 0};
  double tol2 = status->tolerance * status->tolerance;
  status->separated = 0;
  double epsilon = discreteGeoms(obj1, obj2) ? 0 : 0.5 * tol2;
  double min_norm = discreteGeoms(obj1, obj2) ? MINVAL : status->tolerance;
  sub3(x_k, x1_k, x2_k);
  double x_norm = norm3(x_k);
  double x_norm_prev = 0;

  for (; k < kmax; k++) {
    if (x_norm < min_norm || std::abs(x_norm_prev - x_norm) < MINVAL) {
      break;
    }
    gjkSupport(simplex + n, obj1, obj2, x_k, x_norm);
    double* s_k = simplex[n].vert;
    double diff[3] = {x_k[0] - s_k[0], x_k[1] - s_k[1], x_k[2] - s_k[2]};
    if (dot3(x_k, diff) < epsilon) {
      break;
    }
    double lower = dot3(x_k, s_k);
    if (!get_dist) {
      if (lower > 0) {
        status->separated = 1;
        status->gjk_iterations = k;
        status->nsimplex = 0;
        status->nx = 0;
        status->dist[0] = MAX_LIMIT;
        return;
      }
    } else if (status->dist_cutoff < MAX_LIMIT) {
      if (lower > 0 && lower >= status->dist_cutoff * x_norm) {
        status->separated = 1;
        status->gjk_iterations = k;
        status->nsimplex = 0;
        status->nx = 0;
        status->dist[0] = MAX_LIMIT;
        return;
      }
    }
    if (n == 3 && backup_gjk) {
      status->gjk_iterations = k;
      int ret = gjkIntersect(status, obj1, obj2);
      if (ret != -1) {
        status->nx = 0;
        status->separated = ret == 0;
        status->dist[0] = ret > 0 ? 0 : MAX_LIMIT;
        return;
      }
      k = status->gjk_iterations;
      backup_gjk = 0;
    }
    subdistance(lambda, n + 1, simplex);
    n = 0;
    for (int i = 0; i < 4; i++) {
      if (!lambda[i]) {
        continue;
      }
      simplex[n] = simplex[i];
      lambda[n++] = lambda[i];
    }
    if (n < 1) {
      status->gjk_iterations = k;
      status->nsimplex = 0;
      status->nx = 0;
      status->dist[0] = MAX_LIMIT;
      status->separated = 1;
      return;
    }
    lincomb(x_k, lambda, n, simplex[0].vert, simplex[1].vert, simplex[2].vert,
            simplex[3].vert);
    x_norm_prev = x_norm;
    x_norm = norm3(x_k);
    if (n == 4) {
      break;
    }
  }
  if (n > 0) {
    lincomb(x1_k, lambda, n, simplex[0].vert1, simplex[1].vert1,
            simplex[2].vert1, simplex[3].vert1);
    lincomb(x2_k, lambda, n, simplex[0].vert2, simplex[1].vert2,
            simplex[2].vert2, simplex[3].vert2);
  }
  Vertex tmp;
  gjkSupport(&tmp, obj1, obj2, x_k, x_norm);
  if (dot3(x_k, tmp.vert) > 0) {
    status->separated = 1;
  }
  status->nx = 1;
  status->gjk_iterations = k;
  status->nsimplex = n;
  status->dist[0] = (n == 4 && !status->separated) ? 0 : x_norm;
}

inline void replaceSimplex3(Polytope* pt, Status* status, int v1, int v2,
                            int v3) {
  status->nsimplex = 3;
  status->simplex[0] = pt->verts[v1];
  status->simplex[1] = pt->verts[v2];
  status->simplex[2] = pt->verts[v3];
  pt->nfaces = 0;
  pt->nverts = 0;
  pt->nmap = 0;
}

auto sameSide(const double p0[3], const double p1[3], const double p2[3],
              const double p3[3]) -> int {
  double diff1[3];
  double diff2[3];
  double diff3[3];
  double diff4[3];
  double n[3];
  sub3(diff1, p1, p0);
  sub3(diff2, p2, p0);
  cross3(n, diff1, diff2);
  sub3(diff3, p3, p0);
  double dot1 = dot3(n, diff3);
  scl3(diff4, p0, -1);
  double dot2 = dot3(n, diff4);
  if (dot1 > 0 && dot2 > 0) {
    return 1;
  }
  if (dot1 < 0 && dot2 < 0) {
    return 1;
  }
  return 0;
}

auto testTetra(const double p0[3], const double p1[3], const double p2[3],
               const double p3[3]) -> int {
  return sameSide(p0, p1, p2, p3) && sameSide(p1, p2, p3, p0) &&
         sameSide(p2, p3, p0, p1) && sameSide(p3, p0, p1, p2);
}

void rotmat(double r[9], const double axis[3]) {
  double n = norm3(axis);
  double u1 = axis[0] / n;
  double u2 = axis[1] / n;
  double u3 = axis[2] / n;
  const double sin = 0.86602540378;
  const double cos = -0.5;
  r[0] = cos + u1 * u1 * (1 - cos);
  r[1] = u1 * u2 * (1 - cos) - u3 * sin;
  r[2] = u1 * u3 * (1 - cos) + u2 * sin;
  r[3] = u2 * u1 * (1 - cos) + u3 * sin;
  r[4] = cos + u2 * u2 * (1 - cos);
  r[5] = u2 * u3 * (1 - cos) - u1 * sin;
  r[6] = u1 * u3 * (1 - cos) - u2 * sin;
  r[7] = u2 * u3 * (1 - cos) + u1 * sin;
  r[8] = cos + u3 * u3 * (1 - cos);
}

inline auto rayTriangle(const double v1[3], const double v2[3],
                        const double v3[3], const double v4[3],
                        const double v5[3]) -> int {
  double diff12[3];
  double diff13[3];
  double diff14[3];
  double diff15[3];
  sub3(diff12, v2, v1);
  sub3(diff13, v3, v1);
  sub3(diff14, v4, v1);
  sub3(diff15, v5, v1);
  double vol1 = det3(diff13, diff14, diff12);
  double vol2 = det3(diff14, diff15, diff12);
  double vol3 = det3(diff15, diff13, diff12);
  if (vol1 >= 0 && vol2 >= 0 && vol3 >= 0) {
    return 1;
  }
  if (vol1 <= 0 && vol2 <= 0 && vol3 <= 0) {
    return -1;
  }
  return 0;
}

inline auto insertVertex(Polytope* pt, const Vertex* v) -> int {
  int n = pt->nverts++;
  pt->verts[n] = *v;
  return n;
}

void deleteFace(Polytope* pt, Face* face) {
  if (face->index >= 0) {
    pt->map[face->index] = pt->map[--pt->nmap];
    pt->map[face->index]->index = face->index;
  }
  face->index = -2;
}

inline auto maxFaces(Polytope* pt) -> int { return pt->maxfaces - pt->nfaces; }

inline auto attachFace(Polytope* pt, int v1, int v2, int v3, int adj1, int adj2,
                       int adj3) -> double {
  Face* face = &pt->faces[pt->nfaces++];
  face->verts = v1 + (v2 << 10) + (v3 << 20);
  face->adj[0] = adj1;
  face->adj[1] = adj2;
  face->adj[2] = adj3;
  int ret = projectOriginPlane(face->v, pt->verts[v3].vert, pt->verts[v2].vert,
                               pt->verts[v1].vert);
  if (ret) {
    return 0;
  }
  double outward[3];
  sub3(outward, pt->verts[v1].vert, pt->center);
  if (dot3(face->v, outward) < 0) {
    scl3(face->v, face->v, -1);
  }
  face->dist2 = dot3(face->v, face->v);
  face->index = -1;
  return face->dist2;
}

void triAffineCoord(double lambda[3], const double v1[3], const double v2[3],
                    const double v3[3], const double p[3]) {
  double m14 = v2[1] * v3[2] - v2[2] * v3[1] - v1[1] * v3[2] + v1[2] * v3[1] +
               v1[1] * v2[2] - v1[2] * v2[1];
  double m24 = v2[0] * v3[2] - v2[2] * v3[0] - v1[0] * v3[2] + v1[2] * v3[0] +
               v1[0] * v2[2] - v1[2] * v2[0];
  double m34 = v2[0] * v3[1] - v2[1] * v3[0] - v1[0] * v3[1] + v1[1] * v3[0] +
               v1[0] * v2[1] - v1[1] * v2[0];
  double m_max = 0;
  int x = 0;
  int y = 1;
  double mu1 = std::abs(m14);
  double mu2 = std::abs(m24);
  double mu3 = std::abs(m34);
  if (mu1 >= mu2 && mu1 >= mu3) {
    m_max = m14;
    x = 1;
    y = 2;
  } else if (mu2 >= mu3) {
    m_max = m24;
    x = 0;
    y = 2;
  } else {
    m_max = m34;
    x = 0;
    y = 1;
  }
  double c31 = p[x] * v2[y] + p[y] * v3[x] + v2[x] * v3[y] - p[x] * v3[y] -
               p[y] * v2[x] - v3[x] * v2[y];
  double c32 = p[x] * v3[y] + p[y] * v1[x] + v3[x] * v1[y] - p[x] * v1[y] -
               p[y] * v3[x] - v1[x] * v3[y];
  double c33 = p[x] * v1[y] + p[y] * v2[x] + v1[x] * v2[y] - p[x] * v2[y] -
               p[y] * v1[x] - v2[x] * v1[y];
  lambda[0] = c31 / m_max;
  lambda[1] = c32 / m_max;
  lambda[2] = c33 / m_max;
}

auto triPointIntersect(const double v1[3], const double v2[3],
                       const double v3[3], const double p[3]) -> int {
  double lambda[3];
  triAffineCoord(lambda, v1, v2, v3, p);
  if (lambda[0] < 0 || lambda[1] < 0 || lambda[2] < 0) {
    return 0;
  }
  double pr[3];
  double diff[3];
  pr[0] = v1[0] * lambda[0] + v2[0] * lambda[1] + v3[0] * lambda[2];
  pr[1] = v1[1] * lambda[0] + v2[1] * lambda[1] + v3[1] * lambda[2];
  pr[2] = v1[2] * lambda[0] + v2[2] * lambda[1] + v3[2] * lambda[2];
  sub3(diff, pr, p);
  return norm3(diff) < MINVAL;
}

auto polytope3(Polytope* pt, Status* status, Object* obj1, Object* obj2)
    -> int {
  const double* v1 = status->simplex[0].vert;
  const double* v2 = status->simplex[1].vert;
  const double* v3 = status->simplex[2].vert;
  add3(pt->center, v1, v2);
  add3(pt->center, pt->center, v3);
  scl3(pt->center, pt->center, 1.0 / 3.0);
  double diff1[3];
  double diff2[3];
  double n[3];
  double n_neg[3];
  sub3(diff1, v2, v1);
  sub3(diff2, v3, v1);
  cross3(n, diff1, diff2);
  double n_norm = norm3(n);
  if (n_norm < MINVAL) {
    return EPA_P3_BAD_NORMAL;
  }
  scl3(n_neg, n, -1);
  int v1i = insertVertex(pt, status->simplex + 0);
  int v2i = insertVertex(pt, status->simplex + 1);
  int v3i = insertVertex(pt, status->simplex + 2);
  int v5i = epaSupport(pt, obj1, obj2, n_neg, n_norm);
  int v4i = epaSupport(pt, obj1, obj2, n, n_norm);
  // The vertices are read after inserting: the simplex was copied in.
  v1 = pt->verts[v1i].vert;
  v2 = pt->verts[v2i].vert;
  v3 = pt->verts[v3i].vert;
  double* v4 = pt->verts[v4i].vert;
  double* v5 = pt->verts[v5i].vert;
  if (triPointIntersect(v1, v2, v3, v4)) {
    return EPA_P3_INVALID_V4;
  }
  if (triPointIntersect(v1, v2, v3, v5)) {
    return EPA_P3_INVALID_V5;
  }
  if (status->dist[0] > 10 * MINVAL && !testTetra(v1, v2, v3, v4) &&
      !testTetra(v1, v2, v3, v5)) {
    return EPA_P3_MISSING_ORIGIN;
  }
  if (attachFace(pt, v4i, v1i, v2i, 1, 3, 2) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v4i, v3i, v1i, 2, 4, 0) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v4i, v2i, v3i, 0, 5, 1) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v5i, v2i, v1i, 5, 0, 4) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v5i, v1i, v3i, 3, 1, 5) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v5i, v3i, v2i, 4, 2, 3) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  for (int i = 0; i < 6; i++) {
    pt->map[i] = &pt->faces[i];
    pt->faces[i].index = i;
  }
  pt->nmap = 6;
  return 0;
}

auto polytope2(Polytope* pt, Status* status, Object* obj1, Object* obj2)
    -> int {
  double* v1 = status->simplex[0].vert;
  double* v2 = status->simplex[1].vert;
  add3(pt->center, v1, v2);
  scl3(pt->center, pt->center, 0.5);
  double diff[3];
  sub3(diff, v2, v1);
  double value = MAX_LIMIT;
  int index = 0;
  for (int i = 0; i < 3; i++) {
    if (std::abs(diff[i]) < value) {
      value = std::abs(diff[i]);
      index = i;
    }
  }
  double e[3] = {0, 0, 0};
  e[index] = 1;
  double d1[3];
  double d2[3];
  double d3[3];
  cross3(d1, e, diff);
  double r[9];
  rotmat(r, diff);
  mulMatVec3(d2, r, d1);
  mulMatVec3(d3, r, d2);
  int v1i = insertVertex(pt, status->simplex + 0);
  int v2i = insertVertex(pt, status->simplex + 1);
  int v3i = epaSupport(pt, obj1, obj2, d1, norm3(d1));
  int v4i = epaSupport(pt, obj1, obj2, d2, norm3(d2));
  int v5i = epaSupport(pt, obj1, obj2, d3, norm3(d3));
  double* v3 = pt->verts[v3i].vert;
  double* v4 = pt->verts[v4i].vert;
  double* v5 = pt->verts[v5i].vert;
  if (attachFace(pt, v1i, v3i, v4i, 1, 3, 2) < MINDIST2) {
    replaceSimplex3(pt, status, v1i, v3i, v4i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1i, v5i, v3i, 2, 4, 0) < MINDIST2) {
    replaceSimplex3(pt, status, v1i, v5i, v3i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1i, v4i, v5i, 0, 5, 1) < MINDIST2) {
    replaceSimplex3(pt, status, v1i, v4i, v5i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v4i, v3i, 5, 0, 4) < MINDIST2) {
    replaceSimplex3(pt, status, v2i, v4i, v3i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v3i, v5i, 3, 1, 5) < MINDIST2) {
    replaceSimplex3(pt, status, v2i, v3i, v5i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v5i, v4i, 4, 2, 3) < MINDIST2) {
    replaceSimplex3(pt, status, v2i, v5i, v4i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (!rayTriangle(pt->verts[v1i].vert, pt->verts[v2i].vert, v3, v4, v5)) {
    return EPA_P2_NONCONVEX;
  }
  for (int i = 0; i < 6; i++) {
    pt->map[i] = &pt->faces[i];
    pt->faces[i].index = i;
  }
  pt->nmap = 6;
  return 0;
}

auto polytope4(Polytope* pt, Status* status, Object* obj1, Object* obj2)
    -> int {
  int v1 = insertVertex(pt, status->simplex + 0);
  int v2 = insertVertex(pt, status->simplex + 1);
  int v3 = insertVertex(pt, status->simplex + 2);
  int v4 = insertVertex(pt, status->simplex + 3);
  add3(pt->center, pt->verts[v1].vert, pt->verts[v2].vert);
  add3(pt->center, pt->center, pt->verts[v3].vert);
  add3(pt->center, pt->center, pt->verts[v4].vert);
  scl3(pt->center, pt->center, 0.25);
  if (attachFace(pt, v1, v2, v3, 1, 3, 2) < MINDIST4) {
    replaceSimplex3(pt, status, v1, v2, v3);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1, v4, v2, 2, 3, 0) < MINDIST4) {
    replaceSimplex3(pt, status, v1, v4, v2);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1, v3, v4, 0, 3, 1) < MINDIST4) {
    replaceSimplex3(pt, status, v1, v3, v4);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v4, v3, v2, 2, 0, 1) < MINDIST4) {
    replaceSimplex3(pt, status, v4, v3, v2);
    return polytope3(pt, status, obj1, obj2);
  }
  if (!testTetra(pt->verts[v1].vert, pt->verts[v2].vert, pt->verts[v3].vert,
                 pt->verts[v4].vert)) {
    return EPA_P4_MISSING_ORIGIN;
  }
  for (int i = 0; i < 4; i++) {
    pt->map[i] = &pt->faces[i];
    pt->faces[i].index = i;
  }
  pt->nmap = 4;
  return 0;
}

inline void addEdge(Polytope* pt, int index, int edge) {
  pt->horizon_edges[pt->nedges] = edge;
  pt->horizon_indices[pt->nedges++] = index;
}

inline auto getEdge(const Face* face, int vertex) -> int {
  int verts[3];
  expand(verts, face->verts);
  if (verts[0] == vertex) {
    return 0;
  }
  if (verts[1] == vertex) {
    return 1;
  }
  return 2;
}

auto horizonRec(Polytope* pt, Face* face, int e) -> int {
  if (dot3(face->v, pt->horizon_w) - face->dist2 > MINVAL) {
    int verts[3];
    expand(verts, face->verts);
    deleteFace(pt, face);
    for (int k = 1; k < 3; k++) {
      int i = (e + k) % 3;
      Face* adj = &pt->faces[face->adj[i]];
      if (adj->index > -2) {
        int adj_edge = getEdge(adj, verts[(i + 1) % 3]);
        if (!horizonRec(pt, adj, adj_edge)) {
          addEdge(pt, face->adj[i], adj_edge);
        }
      }
    }
    return 1;
  }
  return 0;
}

void horizon(Polytope* pt, Face* face) {
  deleteFace(pt, face);
  int verts[3];
  expand(verts, face->verts);
  Face* adj = &pt->faces[face->adj[0]];
  int adj_edge = getEdge(adj, verts[1]);
  if (!horizonRec(pt, adj, adj_edge)) {
    addEdge(pt, face->adj[0], adj_edge);
  }
  adj = &pt->faces[face->adj[1]];
  adj_edge = getEdge(adj, verts[2]);
  if (adj->index > -2 && !horizonRec(pt, adj, adj_edge)) {
    addEdge(pt, face->adj[1], adj_edge);
  }
  adj = &pt->faces[face->adj[2]];
  adj_edge = getEdge(adj, verts[0]);
  if (adj->index > -2 && !horizonRec(pt, adj, adj_edge)) {
    addEdge(pt, face->adj[2], adj_edge);
  }
}

auto epaWitness(const Polytope* pt, const Face* face, double x1[3],
                double x2[3]) -> double {
  double lambda[3];
  int verts[3];
  expand(verts, face->verts);
  const Vertex* v1 = &pt->verts[verts[0]];
  const Vertex* v2 = &pt->verts[verts[1]];
  const Vertex* v3 = &pt->verts[verts[2]];
  triAffineCoord(lambda, v1->vert, v2->vert, v3->vert, face->v);
  lincomb(x1, lambda, 3, v1->vert1, v2->vert1, v3->vert1, nullptr);
  lincomb(x2, lambda, 3, v1->vert2, v2->vert2, v3->vert2, nullptr);
  return -std::sqrt(face->dist2);
}

auto epa(Status* status, Polytope* pt, Object* obj1, Object* obj2) -> Face* {
  double upper = MAX_LIMIT;
  double upper2 = MAX_LIMIT;
  double lower2 = 0.0;
  Face* face = nullptr;
  Face* pface = nullptr;
  bool discrete = discreteGeoms(obj1, obj2);
  double tolerance = discrete ? MINEPATOL : status->tolerance;
  int k = 0;
  int kmax = status->max_iterations < 1000 ? status->max_iterations : 1000;
  for (k = 0; k < kmax; k++) {
    pface = face;
    lower2 = MAX_LIMIT;
    for (int i = 0; i < pt->nmap; i++) {
      if (pt->map[i]->dist2 < lower2) {
        face = pt->map[i];
        lower2 = face->dist2;
      }
    }
    if (lower2 > upper2 || face == nullptr) {
      face = pface;
      break;
    }
    if (lower2 <= 0) {
      break;
    }
    double lower = std::sqrt(lower2);
    int wi = epaSupport(pt, obj1, obj2, face->v, lower);
    const Vertex* w = &pt->verts[wi];
    double upper_k = dot3(face->v, w->vert) / lower;
    if (upper_k < upper) {
      upper = upper_k;
      upper2 = upper * upper;
    }
    if (upper - lower < tolerance) {
      if (k == 0 && upper < lower - 1e-10) {
        face = nullptr;
      }
      break;
    }
    if (discrete) {
      int i = 0;
      int nverts = pt->nverts - 1;
      for (; i < nverts; i++) {
        if (w->index1 == pt->verts[i].index1 &&
            w->index2 == pt->verts[i].index2) {
          break;
        }
      }
      if (i != nverts) {
        break;
      }
    }
    pt->horizon_w = w->vert;
    horizon(pt, face);
    if (pt->nedges < 3) {
      face = nullptr;
      break;
    }
    int nfaces = pt->nfaces;
    int nedges = pt->nedges;
    if (nedges > maxFaces(pt)) {
      break;
    }
    int hzn_index = pt->horizon_indices[0];
    int hzn_edge = pt->horizon_edges[0];
    Face* hzn_face = &pt->faces[hzn_index];
    int hzn_verts[3];
    expand(hzn_verts, hzn_face->verts);
    int v1 = hzn_verts[hzn_edge];
    int v2 = hzn_verts[(hzn_edge + 1) % 3];
    hzn_face->adj[hzn_edge] = nfaces;
    double dist2 =
        attachFace(pt, wi, v2, v1, nfaces + nedges - 1, hzn_index, nfaces + 1);
    if (dist2 == 0) {
      face = nullptr;
      break;
    }
    if (dist2 >= lower2 && dist2 <= upper2) {
      int i = pt->nmap++;
      pt->map[i] = &pt->faces[pt->nfaces - 1];
      pt->map[i]->index = i;
    }
    for (int i = 1; i < nedges; i++) {
      int cur = nfaces + i;
      int next = nfaces + (i + 1) % nedges;
      hzn_index = pt->horizon_indices[i];
      hzn_edge = pt->horizon_edges[i];
      hzn_face = &pt->faces[hzn_index];
      int verts2[3];
      expand(verts2, hzn_face->verts);
      v1 = verts2[hzn_edge];
      v2 = verts2[(hzn_edge + 1) % 3];
      hzn_face->adj[hzn_edge] = cur;
      dist2 = attachFace(pt, wi, v2, v1, cur - 1, hzn_index, next);
      if (dist2 == 0) {
        face = nullptr;
        break;
      }
      if (dist2 >= lower2 && dist2 <= upper2) {
        int idx = pt->nmap++;
        pt->map[idx] = &pt->faces[pt->nfaces - 1];
        pt->map[idx]->index = idx;
      }
    }
    pt->nedges = 0;
    if (!pt->nmap || face == nullptr) {
      break;
    }
  }
  status->epa_iterations = k;
  if (face != nullptr) {
    status->dist[0] = epaWitness(pt, face, status->x1, status->x2);
    status->nx = 1;
  } else {
    status->nx = 0;
    status->dist[0] = 0;
  }
  return face;
}

//-- Multiple contacts
//----------------------------------------------------------

inline auto area4(const double* hull, int a, int b, int c, int d) -> double {
  double ca[3];
  double db[3];
  double cross[3];
  sub3(ca, hull + 3 * a, hull + 3 * c);
  sub3(db, hull + 3 * b, hull + 3 * d);
  cross3(cross, ca, db);
  return 0.5 * norm3(cross);
}

inline void hull4(int res[4], const double* hull, int nhull) {
  int a = 0;
  int b = 1;
  int c = 2;
  int d = 3;
  res[0] = 0, res[1] = 1, res[2] = 2, res[3] = 3;
  double m = area4(hull, a, b, c, d);
  double m_next = 0.0;
  for (; a < nhull; a++) {
    while (true) {
      int d_next = (d + 1) % nhull;
      m_next = area4(hull, a, b, c, d_next);
      if (m_next <= m) {
        break;
      }
      d = d_next, m = m_next;
      res[0] = a, res[1] = b, res[2] = c, res[3] = d;
      while (true) {
        int c_next = (c + 1) % nhull;
        m_next = area4(hull, a, b, c_next, d);
        if (m_next <= m) {
          break;
        }
        c = c_next, m = m_next;
        res[0] = a, res[1] = b, res[2] = c, res[3] = d;
      }
      while (true) {
        int b_next = (b + 1) % nhull;
        m_next = area4(hull, a, b_next, c, d);
        if (m_next <= m) {
          break;
        }
        b = b_next, m = m_next;
        res[0] = a, res[1] = b, res[2] = c, res[3] = d;
      }
    }
    if (b == a) {
      b = (b + 1) % nhull;
      if (c == b) {
        c = (c + 1) % nhull;
        if (d == c) {
          d = (d + 1) % nhull;
        }
      }
    }
  }
}

auto planeNormal(double res[3], const double v1[3], const double v2[3],
                 const double n[3]) -> double {
  double v3[3];
  double diff1[3];
  double diff2[3];
  add3(v3, v1, n);
  sub3(diff1, v2, v1);
  sub3(diff2, v3, v1);
  cross3(res, diff1, diff2);
  normalize3(res);
  return dot3(res, v1);
}

auto halfspace(const double a[3], const double n[3], const double p[3]) -> int {
  double diff[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
  return dot3(diff, n) > -MINVAL;
}

inline auto witnessOnFace(double w1[3], double w2[3], const double v[3],
                          const double* p, const double n[3],
                          const double dir[3]) -> double {
  double d[3] = {v[0] - p[0], v[1] - p[1], v[2] - p[2]};
  double dist = dot3(d, n);
  addScl3(w1, v, dir, -std::abs(dist));
  copy3(w2, v);
  return dist;
}

void polygonClip(Status* status, const double* face1, int nface1,
                 const double* face2, int nface2, const double n[3],
                 const double dir[3], int npolygonmax) {
  if (nface1 < 3) {
    return;
  }
  double* dist = status->dist;
  std::vector<double> storage(std::size_t(6 * npolygonmax) * 2 +
                              std::size_t(3 * npolygonmax) + npolygonmax);
  double* polygon = storage.data();
  double* clipped = polygon + 6 * npolygonmax;
  double* pn = clipped + 6 * npolygonmax;
  double* pd = pn + 3 * npolygonmax;
  for (int i = 0; i < nface1 - 1; i++) {
    pd[i] = planeNormal(&pn[3 * i], &face1[3 * i], &face1[3 * i + 3], n);
  }
  pd[nface1 - 1] = planeNormal(&pn[3 * (nface1 - 1)], &face1[3 * (nface1 - 1)],
                               &face1[0], n);
  int npolygon = nface2;
  int nclipped = 0;
  for (int i = 0; i < nface2; i++) {
    copy3(polygon + 3 * i, face2 + 3 * i);
  }
  for (int e = 0; e < (3 * nface1); e += 3) {
    for (int i = 0; i < npolygon; i++) {
      double* p = polygon + 3 * i;
      double* q = (i < npolygon - 1) ? polygon + 3 * (i + 1) : polygon;
      double pq[3] = {q[0] - p[0], q[1] - p[1], q[2] - p[2]};
      int inside1 = halfspace(face1 + e, pn + e, p);
      int inside2 = halfspace(face1 + e, pn + e, q);
      if (!inside1 && !inside2) {
        continue;
      }
      if (inside1 && inside2) {
        copy3(clipped + 3 * nclipped++, q);
        continue;
      }
      double tmp = dot3(pn + e, pq);
      if (tmp != 0.0) {
        double t = (pd[e / 3] - dot3(pn + e, p)) / tmp;
        if (t >= 0.0 && t <= 1.0) {
          addScl3(clipped + 3 * nclipped++, p, pq, t);
        }
      }
      if (inside2) {
        copy3(clipped + 3 * nclipped++, q);
      }
    }
    std::swap(polygon, clipped);
    npolygon = nclipped;
    nclipped = 0;
  }
  int m = npolygon;
  npolygon = 0;
  for (int i = 0; i < m; i++) {
    double diff[3];
    sub3(diff, polygon + 3 * i, face1);
    if (dot3(diff, n) <= 0) {
      if (npolygon != i) {
        copy3(polygon + 3 * npolygon, polygon + 3 * i);
      }
      npolygon++;
    }
  }
  if (npolygon < 1) {
    return;
  }
  if (status->max_contacts < 5 && npolygon > 4) {
    status->nx = 4;
    int idx[4];
    hull4(idx, polygon, npolygon);
    for (int i = 0; i < 4; i++) {
      dist[i] = witnessOnFace(status->x1 + 3 * i, status->x2 + 3 * i,
                              polygon + 3 * idx[i], face1, n, dir);
    }
    return;
  }
  if (nface2 == 2 && npolygon > 2) {
    int best1 = 0;
    int best2 = 1;
    double d = 0;
    for (int i = 0; i < npolygon; i++) {
      for (int j = i + 1; j < npolygon; j++) {
        double diff[3];
        sub3(diff, polygon + 3 * j, polygon + 3 * i);
        double d2 = dot3(diff, diff);
        if (d2 > d) {
          d = d2;
          best1 = i;
          best2 = j;
        }
      }
    }
    dist[0] = witnessOnFace(status->x1, status->x2, polygon + 3 * best1, face1,
                            n, dir);
    dist[1] = witnessOnFace(status->x1 + 3, status->x2 + 3, polygon + 3 * best2,
                            face1, n, dir);
    status->nx = 2;
    return;
  }
  npolygon = std::min(npolygon, MAXCONPAIR);
  for (int i = 0; i < npolygon; i++) {
    dist[i] = witnessOnFace(status->x1 + 3 * i, status->x2 + 3 * i,
                            polygon + 3 * i, face1, n, dir);
  }
  status->nx = npolygon;
}

auto cylinderNormals(double res[9], int resind[3], int dim, const Object* obj,
                     const int vi[3]) -> int {
  if (dim == 1) {
    globalcoord(res, obj->mat, nullptr, 0, 0, vi[0] ? -1.0 : 1.0);
    resind[0] = vi[0];
    return 1;
  }
  return 0;
}

auto boxNormals2(double res[9], int resind[3], const double mat[9],
                 const double n[3]) -> int {
  double normals[18] = {1, 0,  0, -1, 0, 0, 0, 1, 0,
                        0, -1, 0, 0,  0, 1, 0, 0, -1};
  double local_n[3];
  local_n[0] = mat[0] * n[0] + mat[3] * n[1] + mat[6] * n[2];
  local_n[1] = mat[1] * n[0] + mat[4] * n[1] + mat[7] * n[2];
  local_n[2] = mat[2] * n[0] + mat[5] * n[1] + mat[8] * n[2];
  scl3(local_n, local_n, 1 / std::sqrt(dot3(local_n, local_n)));
  for (int i = 0; i < 6; i++) {
    if (dot3(local_n, normals + 3 * i) > FACE_TOL) {
      globalcoord(res, mat, nullptr, normals[3 * i], normals[3 * i + 1],
                  normals[3 * i + 2]);
      resind[0] = i;
      return 1;
    }
  }
  return 0;
}

auto boxNormals(double res[9], int resind[3], int dim, const Object* obj,
                const int vi[3], const double dir[3]) -> int {
  int v1 = vi[0];
  int v2 = vi[1];
  int v3 = vi[2];
  const double* mat = obj->mat;
  if (dim == 3) {
    int c = 0;
    int x = ((v1 & 1) && (v2 & 1) && (v3 & 1)) -
            (!(v1 & 1) && !(v2 & 1) && !(v3 & 1));
    int y = ((v1 & 2) && (v2 & 2) && (v3 & 2)) -
            (!(v1 & 2) && !(v2 & 2) && !(v3 & 2));
    int z = ((v1 & 4) && (v2 & 4) && (v3 & 4)) -
            (!(v1 & 4) && !(v2 & 4) && !(v3 & 4));
    globalcoord(res, mat, nullptr, x, y, z);
    int sgn = x + y + z;
    if (x) {
      resind[c++] = 0;
    }
    if (y) {
      resind[c++] = 2;
    }
    if (z) {
      resind[c++] = 4;
    }
    if (sgn == -1) {
      resind[0]++;
    }
    return c == 1 ? 1 : boxNormals2(res, resind, mat, dir);
  }
  if (dim == 2) {
    int c = 0;
    int x = ((v1 & 1) && (v2 & 1)) - (!(v1 & 1) && !(v2 & 1));
    int y = ((v1 & 2) && (v2 & 2)) - (!(v1 & 2) && !(v2 & 2));
    int z = ((v1 & 4) && (v2 & 4)) - (!(v1 & 4) && !(v2 & 4));
    if (x) {
      globalcoord(res, mat, nullptr, x, 0, 0);
      resind[c++] = (x > 0) ? 0 : 1;
    }
    if (y) {
      globalcoord(res + 3 * c, mat, nullptr, 0, y, 0);
      resind[c++] = (y > 0) ? 2 : 3;
    }
    if (z) {
      globalcoord(res + 3, mat, nullptr, 0, 0, z);
      resind[c++] = (z > 0) ? 4 : 5;
    }
    return c == 2 ? 2 : boxNormals2(res, resind, mat, dir);
  }
  if (dim == 1) {
    double x = (v1 & 1) ? 1 : -1;
    double y = (v1 & 2) ? 1 : -1;
    double z = (v1 & 4) ? 1 : -1;
    globalcoord(res + 0, mat, nullptr, x, 0, 0);
    globalcoord(res + 3, mat, nullptr, 0, y, 0);
    globalcoord(res + 6, mat, nullptr, 0, 0, z);
    resind[0] = (x > 0) ? 0 : 1;
    resind[1] = (y > 0) ? 2 : 3;
    resind[2] = (z > 0) ? 4 : 5;
    return 3;
  }
  return 0;
}

auto boxEdgeNormals(double res[9], double endverts[9], int dim,
                    const Object* obj, const double v[9], int v1i) -> int {
  const double* v1 = v;
  const double* v2 = v + 3;
  const double* mat = obj->mat;
  const double* pos = obj->pos;
  const double* size = obj->size;
  if (dim == 2) {
    copy3(endverts, v2);
    sub3(res, v2, v1);
    normalize3(res);
    return 1;
  }
  if (dim == 1) {
    double x = (v1i & 1) ? size[0] : -size[0];
    double y = (v1i & 2) ? size[1] : -size[1];
    double z = (v1i & 4) ? size[2] : -size[2];
    globalcoord(endverts, mat, pos, -x, y, z);
    sub3(res, endverts, v1);
    normalize3(res);
    globalcoord(endverts + 3, mat, pos, x, -y, z);
    sub3(res + 3, endverts + 3, v1);
    normalize3(res + 3);
    globalcoord(endverts + 6, mat, pos, x, y, -z);
    sub3(res + 6, endverts + 6, v1);
    normalize3(res + 6);
    return 3;
  }
  return 0;
}

auto cylinderEdgeNormals(double res[9], double endverts[9], int dim,
                         const Object* obj, const double v[9], int v1i) -> int {
  if (dim == 1 || dim == 2) {
    double sgn = v1i ? 1.0 : -1.0;
    res[0] = sgn * obj->mat[2];
    res[1] = sgn * obj->mat[5];
    res[2] = sgn * obj->mat[8];
    addScl3(endverts, v, res, 2 * obj->size[1]);
    return 1;
  }
  return 0;
}

auto cylinderFace(double res[48], const Object* obj, int idx) -> int {
  static constexpr double COS16[16] = {
      1.000000000000000,  0.923879532511287,  0.707106781186548,
      0.382683432365090,  0.000000000000000,  -0.382683432365090,
      -0.707106781186547, -0.923879532511287, -1.000000000000000,
      -0.923879532511287, -0.707106781186548, -0.382683432365090,
      0.000000000000000,  0.382683432365090,  0.707106781186547,
      0.923879532511287};
  static constexpr double SIN16[16] = {
      0.000000000000000,  0.382683432365090,  0.707106781186547,
      0.923879532511287,  1.000000000000000,  0.923879532511287,
      0.707106781186548,  0.382683432365090,  0.000000000000000,
      -0.382683432365090, -0.707106781186547, -0.923879532511287,
      -1.000000000000000, -0.923879532511287, -0.707106781186548,
      -0.382683432365090};
  double sgn = idx ? -1.0 : 1.0;
  for (int i = 0; i < 16; i++) {
    double x = COS16[i] * obj->size[0];
    double y = -SIN16[i] * obj->size[0] * sgn;
    globalcoord(res + 3 * i, obj->mat, obj->pos, x, y, sgn * obj->size[1]);
  }
  return 16;
}

auto boxFace(double res[12], const Object* obj, int idx) -> int {
  const double* mat = obj->mat;
  const double* pos = obj->pos;
  const double* s = obj->size;
  switch (idx) {
    case 0:
      globalcoord(res + 0, mat, pos, s[0], s[1], s[2]);
      globalcoord(res + 3, mat, pos, s[0], s[1], -s[2]);
      globalcoord(res + 6, mat, pos, s[0], -s[1], -s[2]);
      globalcoord(res + 9, mat, pos, s[0], -s[1], s[2]);
      return 4;
    case 1:
      globalcoord(res + 0, mat, pos, -s[0], s[1], -s[2]);
      globalcoord(res + 3, mat, pos, -s[0], s[1], s[2]);
      globalcoord(res + 6, mat, pos, -s[0], -s[1], s[2]);
      globalcoord(res + 9, mat, pos, -s[0], -s[1], -s[2]);
      return 4;
    case 2:
      globalcoord(res + 0, mat, pos, -s[0], s[1], -s[2]);
      globalcoord(res + 3, mat, pos, s[0], s[1], -s[2]);
      globalcoord(res + 6, mat, pos, s[0], s[1], s[2]);
      globalcoord(res + 9, mat, pos, -s[0], s[1], s[2]);
      return 4;
    case 3:
      globalcoord(res + 0, mat, pos, -s[0], -s[1], s[2]);
      globalcoord(res + 3, mat, pos, s[0], -s[1], s[2]);
      globalcoord(res + 6, mat, pos, s[0], -s[1], -s[2]);
      globalcoord(res + 9, mat, pos, -s[0], -s[1], -s[2]);
      return 4;
    case 4:
      globalcoord(res + 0, mat, pos, -s[0], s[1], s[2]);
      globalcoord(res + 3, mat, pos, s[0], s[1], s[2]);
      globalcoord(res + 6, mat, pos, s[0], -s[1], s[2]);
      globalcoord(res + 9, mat, pos, -s[0], -s[1], s[2]);
      return 4;
    case 5:
      globalcoord(res + 0, mat, pos, s[0], s[1], -s[2]);
      globalcoord(res + 3, mat, pos, -s[0], s[1], -s[2]);
      globalcoord(res + 6, mat, pos, -s[0], -s[1], -s[2]);
      globalcoord(res + 9, mat, pos, s[0], -s[1], -s[2]);
      return 4;
    default:
      return 0;
  }
}

inline auto alignedFaces(int res[2], const double* v, int nv, const double* w,
                         int nw) -> int {
  for (int i = 0; i < nv; i++) {
    for (int j = 0; j < nw; j++) {
      if (dot3(v + 3 * i, w + 3 * j) < -FACE_TOL) {
        res[0] = i;
        res[1] = j;
        return 1;
      }
    }
  }
  return 0;
}

inline auto alignedFaceEdge(int res[2], const double* edge, int nedge,
                            const double* face, int nface, const double dir[3])
    -> int {
  for (int i = 0; i < nface; i++) {
    if (dot3(face + 3 * i, dir) <= MINVAL) {
      continue;
    }
    for (int j = 0; j < nedge; j++) {
      if (std::abs(dot3(edge + 3 * j, face + 3 * i)) < EDGE_TOL) {
        res[0] = j;
        res[1] = i;
        return 1;
      }
    }
  }
  return 0;
}

inline auto simplexDim(int vi[3], double v[9]) -> int {
  if (vi[0] == vi[1]) {
    if (vi[0] == vi[2]) {
      return 1;
    }
    vi[1] = vi[2];
    copy3(v + 3, v + 6);
    return 2;
  }
  return (vi[2] == vi[0] || vi[2] == vi[1]) ? 2 : 3;
}

auto normals_of(const Object* obj, double* n, int* idx, int dim,
                const int vi[3], const double dir[3]) -> int {
  if (obj->type == GeomType::BOX) {
    return boxNormals(n, idx, dim, obj, vi, dir);
  }
  if (obj->type == GeomType::CYLINDER) {
    return cylinderNormals(n, idx, dim, obj, vi);
  }
  return 0;
}

auto edge_normals_of(const Object* obj, double* n, double* endverts, int dim,
                     const double v[9], int v1i) -> int {
  if (obj->type == GeomType::BOX) {
    return boxEdgeNormals(n, endverts, dim, obj, v, v1i);
  }
  if (obj->type == GeomType::CYLINDER) {
    return cylinderEdgeNormals(n, endverts, dim, obj, v, v1i);
  }
  return 0;
}

auto face_of(const Object* obj, double* face, int index) -> int {
  if (obj->type == GeomType::BOX) {
    return boxFace(face, obj, index);
  }
  if (obj->type == GeomType::CYLINDER) {
    return cylinderFace(face, obj, index);
  }
  return 0;
}

// Where two faces, or a face and an edge, meet: the one clipped against the
// other (multicontact).
void multicontact(Polytope* pt, Face* face, Status* status, Object* obj1,
                  Object* obj2) {
  int npolygonmax = 0;
  if (obj1->type == GeomType::CYLINDER || obj2->type == GeomType::CYLINDER) {
    npolygonmax = std::max(npolygonmax, 16);
  }
  if (obj1->type == GeomType::BOX || obj2->type == GeomType::BOX) {
    npolygonmax = std::max(npolygonmax, 4);
  }
  int verts[3];
  expand(verts, face->verts);
  int v1i[3] = {pt->verts[verts[0]].index1, pt->verts[verts[1]].index1,
                pt->verts[verts[2]].index1};
  int v2i[3] = {pt->verts[verts[0]].index2, pt->verts[verts[1]].index2,
                pt->verts[verts[2]].index2};
  double v1[9];
  double v2[9];
  for (int k = 0; k < 3; ++k) {
    copy3(v1 + 3 * k, pt->verts[verts[k]].vert1);
    copy3(v2 + 3 * k, pt->verts[verts[k]].vert2);
  }
  double n1[9]{};
  double n2[9]{};
  int idx1[3]{};
  int idx2[3]{};
  double endverts[9]{};
  double face1[48]{};
  double face2[48]{};
  int nface1 = simplexDim(v1i, v1);
  int nface2 = simplexDim(v2i, v2);
  double dir[3];
  double dir_neg[3];
  sub3(dir, status->x2, status->x1);
  sub3(dir_neg, status->x1, status->x2);
  int nnorms1 = normals_of(obj1, n1, idx1, nface1, v1i, dir_neg);
  int nnorms2 = normals_of(obj2, n2, idx2, nface2, v2i, dir);
  int res[2];
  int edgecon1 = 0;
  int edgecon2 = 0;
  if (!alignedFaces(res, n1, nnorms1, n2, nnorms2)) {
    if (nface1 < 3 && nface1 <= nface2) {
      nnorms1 = edge_normals_of(obj1, n1, endverts, nface1, v1, v1i[0]);
      if (!alignedFaceEdge(res, n1, nnorms1, n2, nnorms2, dir)) {
        return;
      }
      edgecon1 = 1;
    } else if (nface2 < 3) {
      nnorms2 = edge_normals_of(obj2, n2, endverts, nface2, v2, v2i[0]);
      if (!alignedFaceEdge(res, n2, nnorms2, n1, nnorms1, dir_neg)) {
        return;
      }
      edgecon2 = 1;
    } else {
      return;
    }
  }
  int i = res[0];
  int j = res[1];
  if (edgecon1) {
    copy3(face1, v1);
    copy3(face1 + 3, endverts + 3 * i);
    nface1 = 2;
  } else {
    nface1 = face_of(obj1, face1, edgecon2 ? idx1[j] : idx1[i]);
  }
  if (edgecon2) {
    copy3(face2, v2);
    copy3(face2 + 3, endverts + 3 * i);
    nface2 = 2;
  } else {
    nface2 = face_of(obj2, face2, idx2[j]);
  }
  double wit_dir[3];
  if (edgecon1) {
    scl3(wit_dir, n2 + 3 * j, -1.0);
    polygonClip(status, face2, nface2, face1, nface1, n2 + 3 * j, wit_dir,
                npolygonmax);
    for (int k = 0; k < status->nx; k++) {
      double tmp[3];
      copy3(tmp, status->x1 + 3 * k);
      copy3(status->x1 + 3 * k, status->x2 + 3 * k);
      copy3(status->x2 + 3 * k, tmp);
    }
    return;
  }
  if (edgecon2) {
    scl3(wit_dir, n1 + 3 * j, -1.0);
    polygonClip(status, face1, nface1, face2, nface2, n1 + 3 * j, wit_dir,
                npolygonmax);
    return;
  }
  copy3(wit_dir, n2 + 3 * j);
  polygonClip(status, face1, nface1, face2, nface2, n1 + 3 * i, wit_dir,
              npolygonmax);
}

inline void inflate(Status* status, double margin1, double margin2) {
  double n[3];
  sub3(n, status->x2, status->x1);
  normalize3(n);
  if (margin1 != 0) {
    status->x1[0] += margin1 * n[0];
    status->x1[1] += margin1 * n[1];
    status->x1[2] += margin1 * n[2];
  }
  if (margin2 != 0) {
    status->x2[0] -= margin2 * n[0];
    status->x2[1] -= margin2 * n[1];
    status->x2[2] -= margin2 * n[2];
  }
  status->dist[0] -= (margin1 + margin2);
}

// Distance, or penetration where negative, and witness points (mjc_ccd).
auto ccd(int max_contacts, Status* status, Object* obj1, Object* obj2)
    -> double {
  copy3(status->x1, obj1->pos);
  copy3(status->x2, obj2->pos);
  status->gjk_iterations = 0;
  status->epa_iterations = 0;
  status->epa_status = EPA_NOCONTACT;
  status->tolerance = CCD_TOLERANCE;
  status->max_iterations = CCD_ITERATIONS;
  status->max_contacts = max_contacts;
  status->dist_cutoff = 0;

  // A sphere or capsule shrunk to its point or segment, its radius added
  // back after.
  if (obj1->type == GeomType::SPHERE || obj2->type == GeomType::SPHERE ||
      obj1->type == GeomType::CAPSULE || obj2->type == GeomType::CAPSULE) {
    Support support1 = obj1->support;
    Support support2 = obj2->support;
    double full_margin1 = 0;
    double full_margin2 = 0;
    double margin1 = obj1->margin;
    double margin2 = obj2->margin;
    if (obj1->type == GeomType::SPHERE) {
      full_margin1 = obj1->size[0] + 0.5 * margin1;
      obj1->support = pointSupport;
      obj1->margin = 0;
    } else if (obj1->type == GeomType::CAPSULE) {
      full_margin1 = obj1->size[0] + 0.5 * margin1;
      obj1->support = lineSupport;
      obj1->margin = 0;
    }
    if (obj2->type == GeomType::SPHERE) {
      full_margin2 = obj2->size[0] + 0.5 * margin2;
      obj2->support = pointSupport;
      obj2->margin = 0;
    } else if (obj2->type == GeomType::CAPSULE) {
      full_margin2 = obj2->size[0] + 0.5 * margin2;
      obj2->support = lineSupport;
      obj2->margin = 0;
    }
    status->dist_cutoff += full_margin1 + full_margin2;
    gjk(status, obj1, obj2);
    status->dist_cutoff = 0;
    obj1->margin = margin1;
    obj2->margin = margin2;
    obj1->support = support1;
    obj2->support = support2;
    if (status->dist[0] > status->tolerance) {
      inflate(status, full_margin1, full_margin2);
      if (status->dist[0] > status->dist_cutoff) {
        status->dist[0] = MAX_LIMIT;
      }
      return status->dist[0];
    }
    status->gjk_iterations = 0;
    copy3(status->x1, obj1->pos);
    copy3(status->x2, obj2->pos);
  }
  gjk(status, obj1, obj2);
  if (status->dist[0] <= CCD_TOLERANCE && status->nsimplex > 1 &&
      !status->separated) {
    status->dist[0] = 0;
    Polytope pt;
    int n = CCD_ITERATIONS;
    pt.maxfaces = 6 * n;
    pt.verts.resize(5 + std::size_t(n));
    pt.faces.resize(std::size_t(6 * n));
    pt.map.resize(std::size_t(6 * n));
    int ret = 0;
    if (status->nsimplex == 2) {
      ret = polytope2(&pt, status, obj1, obj2);
    } else if (status->nsimplex == 3) {
      ret = polytope3(&pt, status, obj1, obj2);
    } else {
      ret = polytope4(&pt, status, obj1, obj2);
    }
    status->epa_status = ret;
    if (ret == 0) {
      Face* face = epa(status, &pt, obj1, obj2);
      if (max_contacts > 1 && face != nullptr) {
        multicontact(&pt, face, status, obj1, obj2);
      }
    }
  }
  double min_dist = status->dist[0];
  for (int i = 1; i < status->nx; i++) {
    min_dist = std::min(min_dist, status->dist[i]);
  }
  return min_dist;
}

// Contacts from the witness points of a penetration (mjc_penetration).
auto penetration(Object* obj1, Object* obj2, PreContact* con, int nconmax,
                 double margin) -> int {
  Status status;
  int ncon = 0;
  if (ccd(nconmax, &status, obj1, obj2) < 0) {
    for (int i = 0; i < status.nx; i++) {
      con[i].dist = margin + status.dist[i];
      double pos[3];
      add3(pos, status.x1 + 3 * i, status.x2 + 3 * i);
      scl3(pos, pos, 0.5);
      double normal[3];
      sub3(normal, status.x1 + 3 * i, status.x2 + 3 * i);
      normalize3(normal);
      for (int k = 0; k < 3; ++k) {
        con[i].pos[k] = pos[k];
        con[i].normal[k] = normal[k];
        con[i].tangent[k] = 0;
      }
    }
    ncon = status.nx;
  }
  return ncon;
}

// How many contacts one pass may find (maxContacts).
auto max_contacts(const Object* obj1, const Object* obj2) -> int {
  if (obj1->margin > 0 || obj2->margin > 0) {
    return 1;
  }
  bool polygonal1 =
      obj1->type == GeomType::BOX || obj1->type == GeomType::CYLINDER;
  bool polygonal2 =
      obj2->type == GeomType::BOX || obj2->type == GeomType::CYLINDER;
  if (obj1->type == GeomType::BOX && obj2->type == GeomType::BOX) {
    return 8;
  }
  return polygonal1 && polygonal2 ? 4 : 1;
}

// A frame's turn by `rot` about `origin` (mju_rotateFrame).
void rotateFrame(const double origin[3], const double rot[9], double xmat[9],
                 double xpos[3]) {
  double mat[9];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mat[3 * i + j] = rot[3 * i] * xmat[j] + rot[3 * i + 1] * xmat[3 + j] +
                       rot[3 * i + 2] * xmat[6 + j];
    }
  }
  std::copy_n(mat, 9, xmat);
  double rel[3];
  double vec[3];
  sub3(rel, origin, xpos);
  mulMatVec3(vec, rot, rel);
  vec[0] -= rel[0];
  vec[1] -= rel[1];
  vec[2] -= rel[2];
  xpos[0] -= vec[0];
  xpos[1] -= vec[1];
  xpos[2] -= vec[2];
}

}  // namespace

auto collide_convex(const Geom& first, const GeomFrame& f1, double radius1,
                    const Geom& second, const GeomFrame& f2, double radius2,
                    double margin, std::span<PreContact, MAX_PAIR_CONTACTS> out)
    -> std::uint32_t {
  PreContact* con = out.data();
  // A plane against a convex geom: its support point opposite the plane's
  // normal (mjc_PlaneConvex).
  if (first.type == GeomType::PLANE) {
    Object obj = make_object(second, f2, 0);
    double normal[3] = {f1.mat[2], f1.mat[5], f1.mat[8]};
    double dir[3] = {-f1.mat[2], -f1.mat[5], -f1.mat[8]};
    double v[3];
    obj.support(v, &obj, dir);
    double diff[3];
    double pos1[3] = {f1.pos[0], f1.pos[1], f1.pos[2]};
    sub3(diff, v, pos1);
    double dist = dot3(normal, diff);
    if (dist > margin) {
      return 0;
    }
    con[0].dist = dist;
    for (int k = 0; k < 3; ++k) {
      con[0].pos[k] = v[k] + normal[k] * (-0.5 * dist);
      con[0].normal[k] = normal[k];
      con[0].tangent[k] = 0;
    }
    return 1;
  }
  Object obj1 = make_object(first, f1, margin);
  Object obj2 = make_object(second, f2, margin);
  int most = max_contacts(&obj1, &obj2);
  int ncon = penetration(&obj1, &obj2, con, most, margin);
  if (most > 1) {
    return static_cast<std::uint32_t>(ncon);
  }
  // One contact found: turn the geoms a little each way about it, and keep
  // each new contact far enough from the others (mjc_Convex's multiCCD).
  if (ncon == 1 && first.type != GeomType::ELLIPSOID &&
      first.type != GeomType::SPHERE && second.type != GeomType::ELLIPSOID &&
      second.type != GeomType::SPHERE) {
    constexpr double RELATIVE_TOLERANCE = 1e-3;
    constexpr double PERTURBATION = 1e-3;
    Matrix3 frame = {
        con[0].normal[0], con[0].normal[1], con[0].normal[2], 0, 0, 0, 0, 0, 0};
    {
      // mju_makeFrame of the normal alone.
      double x[3] = {frame[0], frame[1], frame[2]};
      normalize3(x);
      double y[3] = {0, 0, 0};
      if (x[1] < 0.5 && x[1] > -0.5) {
        y[1] = 1;
      } else {
        y[2] = 1;
      }
      double tmp[3];
      scl3(tmp, x, dot3(x, y));
      sub3(y, y, tmp);
      normalize3(y);
      double z[3];
      cross3(z, x, y);
      frame = {x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]};
    }
    double tolerance = RELATIVE_TOLERANCE * std::min(radius1, radius2);
    const double* axes[2] = {&frame[3], &frame[6]};
    double angles[2] = {-PERTURBATION, PERTURBATION};
    for (const double* axis : axes) {
      for (double angle : angles) {
        // mji_axisAngle2Quat, mju_quat2Mat.
        double s = std::sin(angle * 0.5);
        Quaternion4 quat{std::cos(angle * 0.5), axis[0] * s, axis[1] * s,
                         axis[2] * s};
        Matrix3 rot = articulated::convert_to_matrix(quat);
        double origin[3] = {con[0].pos[0], con[0].pos[1], con[0].pos[2]};
        rotateFrame(origin, rot.data(), obj1.mat, obj1.pos);
        double invrot[9] = {rot[0], rot[3], rot[6], rot[1], rot[4],
                            rot[7], rot[2], rot[5], rot[8]};
        rotateFrame(origin, invrot, obj2.mat, obj2.pos);
        int n = penetration(&obj1, &obj2, con + ncon, 1, margin);
        if (n != 0 && ncon + 1 <= static_cast<int>(MAX_PAIR_CONTACTS)) {
          bool distinct = true;
          for (int i = 0; i < ncon; i++) {
            double d[3] = {con[i].pos[0] - con[ncon].pos[0],
                           con[i].pos[1] - con[ncon].pos[1],
                           con[i].pos[2] - con[ncon].pos[2]};
            if (norm3(d) <= tolerance) {
              distinct = false;
              break;
            }
          }
          if (distinct) {
            con[ncon].dist = con[0].dist;
            ncon += 1;
          }
        }
        for (int k = 0; k < 3; ++k) {
          obj1.pos[k] = f1.pos[k];
          obj2.pos[k] = f2.pos[k];
        }
        for (int k = 0; k < 9; ++k) {
          obj1.mat[k] = f1.mat[k];
          obj2.mat[k] = f2.mat[k];
        }
      }
    }
  }
  return static_cast<std::uint32_t>(ncon);
}

}  // namespace simon::model
