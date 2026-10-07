// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#include "model/articulated/convex.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "core/vocabulary.hpp"
#include "model/articulated/arithmetic.hpp"

// A port of MuJoCo 3.14.0's native convex collision, engine_collision_gjk.c
// and engine_collision_convex.c (Apache-2.0), for primitives, in its order
// of operations. Each function names the one it ports, so the two read side
// by side.
namespace simon::model {

namespace {

using articulated::add;
using articulated::add_scaled;
using articulated::add_to_scaled;
using articulated::cross;
using articulated::dot;
using articulated::multiply;
using articulated::multiply_transposed;
using articulated::norm;
using articulated::normalize3;
using articulated::scale;
using articulated::subtract;
using articulated::transpose;

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
constexpr int MAX_POLYGON = 16;              // A cylinder's face, the most.

using Triple = std::array<Array3, 3>;
using Indices = std::array<int, 3>;
using Polygon = std::array<Array3, MAX_POLYGON>;

//-- Vectors -------------------------------------------------------------------

// The determinant of the matrix with rows `a`, `b` and `c` (det3).
inline auto compute_determinant(const Array3& a, const Array3& b,
                                const Array3& c) -> double {
  return a[0] * (b[1] * c[2] - b[2] * c[1]) +
         a[1] * (b[2] * c[0] - b[0] * c[2]) +
         a[2] * (b[0] * c[1] - b[1] * c[0]);
}

// `local` turned by `mat` and moved to `pos` (localToGlobal, globalcoord).
inline auto transform(const Matrix3& mat, const Array3& pos,
                      const Array3& local) -> Array3 {
  return add(multiply(mat, local), pos);
}

//-- Objects and their support functions (engine_collision_convex.c) ----------

struct Object;
using Support = auto (*)(InOut<Object> object, const Array3& dir) -> Array3;

// A geom as the convex collider sees it: a support function sets the index
// of the vertex it found, for a box or cylinder (mjCCDObj).
struct Object final {
  Array3 size{};
  Array3 pos{};
  Matrix3 mat{};
  double margin = 0.0;
  Support support = nullptr;
  GeomType type = GeomType::SPHERE;
  int vertex_index = -1;
};

// The point itself (mjc_pointSupport).
auto support_point(InOut<Object> object, const Array3& /*dir*/) -> Array3 {
  return object->pos;
}

// mjc_sphereSupport.
auto support_sphere(InOut<Object> object, const Array3& dir) -> Array3 {
  double radius = object->size[0];
  return {radius * dir[0] + object->pos[0], radius * dir[1] + object->pos[1],
          radius * dir[2] + object->pos[2]};
}

// A capsule's segment (mjc_lineSupport).
auto support_line(InOut<Object> object, const Array3& dir) -> Array3 {
  const Matrix3& mat = object->mat;
  double length = object->size[1];
  double along = mat[2] * dir[0] + mat[5] * dir[1] + mat[8] * dir[2];
  double scl = along >= 0 ? length : -length;
  return {mat[2] * scl + object->pos[0], mat[5] * scl + object->pos[1],
          mat[8] * scl + object->pos[2]};
}

// mjc_capsuleSupport.
auto support_capsule(InOut<Object> object, const Array3& dir) -> Array3 {
  double radius = object->size[0];
  double length = object->size[1];
  Array3 local_dir = multiply_transposed(object->mat, dir);
  Array3 local_supp{local_dir[0] * radius, local_dir[1] * radius,
                    local_dir[2] * radius};
  local_supp[2] += (local_dir[2] >= 0 ? length : -length);
  return transform(object->mat, object->pos, local_supp);
}

// mjc_ellipsoidSupport.
auto support_ellipsoid(InOut<Object> object, const Array3& dir) -> Array3 {
  const Matrix3& mat = object->mat;
  const Array3& size = object->size;
  Array3 local_dir = multiply_transposed(mat, dir);
  Array3 local_supp{local_dir[0] * size[0], local_dir[1] * size[1],
                    local_dir[2] * size[2]};
  double norm2 = local_supp[0] * local_supp[0] + local_supp[1] * local_supp[1] +
                 local_supp[2] * local_supp[2];
  if (norm2 < MINVAL2) {
    return {mat[0] * size[0] + object->pos[0],
            mat[3] * size[0] + object->pos[1],
            mat[6] * size[0] + object->pos[2]};
  }
  double norm_inv = 1 / std::sqrt(norm2);
  local_supp[0] *= norm_inv * size[0];
  local_supp[1] *= norm_inv * size[1];
  local_supp[2] *= norm_inv * size[2];
  return transform(mat, object->pos, local_supp);
}

// mjc_cylinderSupport.
auto support_cylinder(InOut<Object> object, const Array3& dir) -> Array3 {
  const Array3& size = object->size;
  Array3 local_dir = multiply_transposed(object->mat, dir);
  double n2 = local_dir[0] * local_dir[0] + local_dir[1] * local_dir[1];
  double scl = n2 >= MINVAL2 ? size[0] / std::sqrt(n2) : 0;
  Array3 local_supp{scl * local_dir[0], scl * local_dir[1],
                    local_dir[2] >= 0 ? size[1] : -size[1]};
  object->vertex_index = local_dir[2] >= 0 ? 0 : 1;
  return transform(object->mat, object->pos, local_supp);
}

// mjc_boxSupport.
auto support_box(InOut<Object> object, const Array3& dir) -> Array3 {
  const Array3& size = object->size;
  Array3 local_dir = multiply_transposed(object->mat, dir);
  Array3 local_supp{local_dir[0] >= 0 ? size[0] : -size[0],
                    local_dir[1] >= 0 ? size[1] : -size[1],
                    local_dir[2] >= 0 ? size[2] : -size[2]};
  object->vertex_index = (local_supp[0] > 0) ? 1 : 0;
  object->vertex_index |= (local_supp[1] > 0) ? 2 : 0;
  object->vertex_index |= (local_supp[2] > 0) ? 4 : 0;
  return transform(object->mat, object->pos, local_supp);
}

// mjc_initCCDObj, for primitives.
auto make_object(const Geom& geom, const GeomFrame& frame, double margin)
    -> Object {
  Object object{.size = geom.size,
                .pos = frame.pos,
                .mat = frame.mat,
                .margin = margin,
                .type = geom.type};
  switch (geom.type) {
    case GeomType::ELLIPSOID:
      object.support = support_ellipsoid;
      break;
    case GeomType::SPHERE:
      object.support = support_sphere;
      break;
    case GeomType::CAPSULE:
      object.support = support_capsule;
      break;
    case GeomType::CYLINDER:
      object.support = support_cylinder;
      break;
    case GeomType::BOX:
      object.support = support_box;
      break;
    default:
      object.support = nullptr;
      break;
  }
  return object;
}

//-- GJK and EPA (engine_collision_gjk.c) --------------------------------------

// A vertex of the Minkowski difference, the points on each geom it is the
// difference of, and those points' vertex indices (mjtVertex).
struct Vertex final {
  Array3 point{};
  Array3 first{};
  Array3 second{};
  int first_index = 0;
  int second_index = 0;
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

// The distances and witness points found, and the search's settings and
// progress (mjCCDStatus).
struct CcdStatus final {
  std::array<double, MAXCONPAIR> dist{};
  std::array<Array3, MAXCONPAIR> x1{};
  std::array<Array3, MAXCONPAIR> x2{};
  std::array<Vertex, 4> simplex{};
  double tolerance = 0.0;
  double dist_cutoff = 0.0;
  int witness_count = 0;
  int simplex_size = 0;
  int max_iterations = 0;
  int max_contacts = 0;
  int gjk_iterations = 0;
  int epa_iterations = 0;
  int epa_status = EPA_NOCONTACT;
  bool separated = false;
};

// A face of the expanding polytope: the origin's projection onto its plane
// and that point's squared distance, its neighbors across each edge, its
// vertices packed ten bits each, and its place in the map, -1 outside it and
// -2 deleted (mjtFace).
struct Face final {
  Array3 closest{};
  double dist2 = 0.0;
  std::array<int, 3> adj{};
  int verts = 0;
  int index = 0;
};

// A face's three vertices, unpacked.
inline auto unpack_vertices(int packed) -> Indices {
  return {packed & 0x3FF, (packed >> 10) & 0x3FF, (packed >> 20) & 0x3FF};
}

// The expanding polytope: its vertices and faces, the faces that may yet be
// nearest the origin, and the horizon seen from a new vertex (mjPolytope).
struct Polytope final {
  std::vector<Vertex> vertices;
  std::vector<Face> faces;
  std::vector<Face*> map;
  Array3 center{};
  Array3 horizon_point{};
  std::array<int, 24> horizon_indices{};
  std::array<int, 24> horizon_edges{};
  int vertex_count = 0;
  int face_count = 0;
  int max_faces = 0;
  int map_size = 0;
  int edge_count = 0;
};

// Whether two geoms have finitely many vertices: boxes without margins
// (discreteGeoms).
auto are_discrete(const Object& first, const Object& second) -> bool {
  if (first.margin != 0 || second.margin != 0) {
    return false;
  }
  return first.type == GeomType::BOX && second.type == GeomType::BOX;
}

// The sum of the first `n` points weighted by `coef` (lincomb).
inline auto combine_points(std::span<const double> coef, int n,
                           const Array3& v1, const Array3& v2,
                           const Array3& v3 = {}, const Array3& v4 = {})
    -> Array3 {
  switch (n) {
    case 1:
      return {coef[0] * v1[0], coef[0] * v1[1], coef[0] * v1[2]};
    case 2:
      return {coef[0] * v1[0] + coef[1] * v2[0],
              coef[0] * v1[1] + coef[1] * v2[1],
              coef[0] * v1[2] + coef[1] * v2[2]};
    case 3:
      return {coef[0] * v1[0] + coef[1] * v2[0] + coef[2] * v3[0],
              coef[0] * v1[1] + coef[1] * v2[1] + coef[2] * v3[1],
              coef[0] * v1[2] + coef[1] * v2[2] + coef[2] * v3[2]};
    case 4:
      return {
          coef[0] * v1[0] + coef[1] * v2[0] + coef[2] * v3[0] + coef[3] * v4[0],
          coef[0] * v1[1] + coef[1] * v2[1] + coef[2] * v3[1] + coef[3] * v4[1],
          coef[0] * v1[2] + coef[1] * v2[2] + coef[2] * v3[2] +
              coef[3] * v4[2]};
    default:
      return {0, 0, 0};
  }
}

// The origin projected onto the plane through three points, or nothing if
// they lie on a line (projectOriginPlane).
auto project_origin_plane(const Array3& v1, const Array3& v2, const Array3& v3)
    -> std::optional<Array3> {
  Array3 diff21 = subtract(v2, v1);
  Array3 diff31 = subtract(v3, v1);
  Array3 diff32 = subtract(v3, v2);
  Array3 n = cross(diff32, diff21);
  double nv = dot(n, v2);
  double nn = dot(n, n);
  if (nn == 0) {
    return std::nullopt;
  }
  if (nv != 0 && nn > MINVAL) {
    return scale(n, nv / nn);
  }
  n = cross(diff21, diff31);
  nv = dot(n, v1);
  nn = dot(n, n);
  if (nn == 0) {
    return std::nullopt;
  }
  if (nv != 0 && nn > MINVAL) {
    return scale(n, nv / nn);
  }
  n = cross(diff31, diff32);
  nv = dot(n, v3);
  nn = dot(n, n);
  return scale(n, nv / nn);
}

// The origin projected onto the line through two points (projectOriginLine).
inline auto project_origin_line(const Array3& v1, const Array3& v2) -> Array3 {
  Array3 diff = subtract(v2, v1);
  double scl = -(dot(v2, diff) / dot(diff, diff));
  return add_scaled(v2, diff, scl);
}

// 1 if both are positive, -1 if both are negative, else 0 (sameSign2).
inline auto compare_signs(double a, double b) -> int {
  if (a > 0 && b > 0) {
    return 1;
  }
  if (a < 0 && b < 0) {
    return -1;
  }
  return 0;
}

// The weights of a segment's ends at its point nearest the origin, by signed
// volumes (S1D).
auto weigh_segment(const Array3& s1, const Array3& s2)
    -> std::array<double, 2> {
  Array3 p_o = project_origin_line(s1, s2);
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
  bool same = compare_signs(mu_max, c1) && compare_signs(mu_max, c2);
  return {same ? c1 / mu_max : 0, same ? c2 / mu_max : 1};
}

// The weights of a triangle's corners at its point nearest the origin (S2D).
auto weigh_triangle(const Array3& s1, const Array3& s2, const Array3& s3)
    -> std::array<double, 3> {
  std::optional<Array3> projected = project_origin_plane(s1, s2, s3);
  if (!projected) {
    std::array<double, 2> l = weigh_segment(s1, s2);
    return {l[0], l[1], 0};
  }
  const Array3& p_o = *projected;
  double m14 = s2[1] * s3[2] - s2[2] * s3[1] - s1[1] * s3[2] + s1[2] * s3[1] +
               s1[1] * s2[2] - s1[2] * s2[1];
  double m24 = s2[0] * s3[2] - s2[2] * s3[0] - s1[0] * s3[2] + s1[2] * s3[0] +
               s1[0] * s2[2] - s1[2] * s2[0];
  double m34 = s2[0] * s3[1] - s2[1] * s3[0] - s1[0] * s3[1] + s1[1] * s3[0] +
               s1[0] * s2[1] - s1[1] * s2[0];
  double m_max = 0;
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
  std::array<double, 2> a{s1[x], s1[y]};
  std::array<double, 2> b{s2[x], s2[y]};
  std::array<double, 2> c{s3[x], s3[y]};
  std::array<double, 2> p{p_o[x], p_o[y]};
  double c31 = p[0] * b[1] + p[1] * c[0] + b[0] * c[1] - p[0] * c[1] -
               p[1] * b[0] - c[0] * b[1];
  double c32 = p[0] * c[1] + p[1] * a[0] + c[0] * a[1] - p[0] * a[1] -
               p[1] * c[0] - a[0] * c[1];
  double c33 = p[0] * a[1] + p[1] * b[0] + a[0] * b[1] - p[0] * b[1] -
               p[1] * a[0] - b[0] * a[1];
  int comp1 = compare_signs(m_max, c31);
  int comp2 = compare_signs(m_max, c32);
  int comp3 = compare_signs(m_max, c33);
  if (comp1 && comp2 && comp3) {
    return {c31 / m_max, c32 / m_max, c33 / m_max};
  }

  // The origin projects outside: the nearest of the edges it lies beyond.
  std::array<double, 3> lambda{};
  double dmin = MAX_LIMIT;
  if (!comp1) {
    std::array<double, 2> l = weigh_segment(s2, s3);
    Array3 v = combine_points(l, 2, s2, s3);
    double d = dot(v, v);
    lambda = {0, l[0], l[1]};
    dmin = d;
  }
  if (!comp2) {
    std::array<double, 2> l = weigh_segment(s1, s3);
    Array3 v = combine_points(l, 2, s1, s3);
    double d = dot(v, v);
    if (d < dmin) {
      lambda = {l[0], 0, l[1]};
      dmin = d;
    }
  }
  if (!comp3) {
    std::array<double, 2> l = weigh_segment(s1, s2);
    Array3 v = combine_points(l, 2, s1, s2);
    double d = dot(v, v);
    if (d < dmin) {
      lambda = {l[0], l[1], 0};
    }
  }
  return lambda;
}

// The weights of a tetrahedron's corners at its point nearest the origin
// (S3D).
auto weigh_tetrahedron(const Array3& s1, const Array3& s2, const Array3& s3,
                       const Array3& s4) -> std::array<double, 4> {
  double c41 = -compute_determinant(s2, s3, s4);
  double c42 = compute_determinant(s1, s3, s4);
  double c43 = -compute_determinant(s1, s2, s4);
  double c44 = compute_determinant(s1, s2, s3);
  double m_det = c41 + c42 + c43 + c44;
  int comp1 = compare_signs(m_det, c41);
  int comp2 = compare_signs(m_det, c42);
  int comp3 = compare_signs(m_det, c43);
  int comp4 = compare_signs(m_det, c44);
  if (comp1 && comp2 && comp3 && comp4) {
    return {c41 / m_det, c42 / m_det, c43 / m_det, c44 / m_det};
  }

  // The origin lies outside: the nearest of the faces it lies beyond.
  std::array<double, 4> lambda{};
  double dmin = MAX_LIMIT;
  if (!comp1) {
    std::array<double, 3> l = weigh_triangle(s2, s3, s4);
    Array3 x = combine_points(l, 3, s2, s3, s4);
    double d = dot(x, x);
    lambda = {0, l[0], l[1], l[2]};
    dmin = d;
  }
  if (!comp2) {
    std::array<double, 3> l = weigh_triangle(s1, s3, s4);
    Array3 x = combine_points(l, 3, s1, s3, s4);
    double d = dot(x, x);
    if (d < dmin) {
      lambda = {l[0], 0, l[1], l[2]};
      dmin = d;
    }
  }
  if (!comp3) {
    std::array<double, 3> l = weigh_triangle(s1, s2, s4);
    Array3 x = combine_points(l, 3, s1, s2, s4);
    double d = dot(x, x);
    if (d < dmin) {
      lambda = {l[0], l[1], 0, l[2]};
      dmin = d;
    }
  }
  if (!comp4) {
    std::array<double, 3> l = weigh_triangle(s1, s2, s3);
    Array3 x = combine_points(l, 3, s1, s2, s3);
    double d = dot(x, x);
    if (d < dmin) {
      lambda = {l[0], l[1], l[2], 0};
    }
  }
  return lambda;
}

// The weights of the first `n` vertices of `simplex` at its point nearest
// the origin (subdistance).
inline auto weigh_simplex(int n, const std::array<Vertex, 4>& simplex)
    -> std::array<double, 4> {
  switch (n) {
    case 4:
      return weigh_tetrahedron(simplex[0].point, simplex[1].point,
                               simplex[2].point, simplex[3].point);
    case 3: {
      std::array<double, 3> l =
          weigh_triangle(simplex[0].point, simplex[1].point, simplex[2].point);
      return {l[0], l[1], l[2], 0};
    }
    case 2: {
      std::array<double, 2> l =
          weigh_segment(simplex[0].point, simplex[1].point);
      return {l[0], l[1], 0, 0};
    }
    default:
      return {1, 0, 0, 0};
  }
}

// S_{A-B}(dir), each geom's support grown by half its margin (gjk_support's
// mjc_support pair).
inline auto find_support(InOut<Object> first, InOut<Object> second,
                         const Array3& dir, const Array3& dir_neg) -> Vertex {
  Vertex v;
  v.first = first->support(first, dir);
  if (first->margin > 0) {
    add_to_scaled(InOut(v.first), dir, 0.5 * first->margin);
  }
  v.second = second->support(second, dir_neg);
  if (second->margin > 0) {
    add_to_scaled(InOut(v.second), dir_neg, 0.5 * second->margin);
  }
  v.point = subtract(v.first, v.second);
  v.first_index = first->vertex_index;
  v.second_index = second->vertex_index;
  return v;
}

// The support toward the origin from `x_k` (gjkSupport).
inline auto find_gjk_support(InOut<Object> first, InOut<Object> second,
                             const Array3& x_k, double x_norm) -> Vertex {
  Array3 dir_neg = scale(x_k, 1 / x_norm);
  Array3 dir = scale(dir_neg, -1);
  return find_support(first, second, dir, dir_neg);
}

// Appends the support along `d` to the polytope and returns its index
// (epaSupport).
auto append_epa_support(InOut<Polytope> polytope, InOut<Object> first,
                        InOut<Object> second, const Array3& d, double dnorm)
    -> int {
  Array3 dir{1, 0, 0};
  Array3 dir_neg{-1, 0, 0};
  if (dnorm > MINVAL) {
    dir = {d[0] / dnorm, d[1] / dnorm, d[2] / dnorm};
    dir_neg = scale(dir, -1);
  }
  int n = polytope->vertex_count++;
  polytope->vertices[n] = find_support(first, second, dir, dir_neg);
  return n;
}

// The origin's signed distance from the plane of three vertices, and the
// plane's normal, or MAX_LIMIT if they lie on a line (signedDistance).
inline auto compute_signed_distance(Out<Array3> normal, const Vertex& v1,
                                    const Vertex& v2, const Vertex& v3)
    -> double {
  Array3 diff1 = subtract(v3.point, v1.point);
  Array3 diff2 = subtract(v2.point, v1.point);
  *normal = cross(diff1, diff2);
  double norm2 = dot(*normal, *normal);
  if (norm2 > MINVAL2 && norm2 < MAXVAL2) {
    *normal = scale(*normal, 1 / std::sqrt(norm2));
    return dot(*normal, v1.point);
  }
  return MAX_LIMIT;
}

// Whether the geoms intersect, from a tetrahedron GJK found: 1 if so, 0 if
// not, -1 if inconclusive (gjkIntersect).
auto intersect_gjk(InOut<CcdStatus> status, InOut<Object> first,
                   InOut<Object> second) -> int {
  std::array<Vertex, 4> simplex = status->simplex;
  std::array<int, 4> s{0, 1, 2, 3};
  int k = status->gjk_iterations;
  int kmax = status->max_iterations;
  for (; k < kmax; k++) {
    std::array<double, 4> dist{};
    std::array<Array3, 4> normals{};
    dist[0] = compute_signed_distance(Out(normals[0]), simplex[s[2]],
                                      simplex[s[1]], simplex[s[3]]);
    dist[1] = compute_signed_distance(Out(normals[1]), simplex[s[0]],
                                      simplex[s[2]], simplex[s[3]]);
    dist[2] = compute_signed_distance(Out(normals[2]), simplex[s[1]],
                                      simplex[s[0]], simplex[s[3]]);
    dist[3] = compute_signed_distance(Out(normals[3]), simplex[s[0]],
                                      simplex[s[1]], simplex[s[2]]);
    if (dist[3] == 0 || dist[2] == 0 || dist[1] == 0 || dist[0] == 0) {
      status->gjk_iterations = k;
      return -1;
    }
    int i = (dist[0] < dist[1]) ? 0 : 1;
    int j = (dist[2] < dist[3]) ? 2 : 3;
    int index = (dist[i] < dist[j]) ? i : j;
    if (dist[index] > 0) {
      status->simplex_size = 4;
      for (int c = 0; c < 4; ++c) {
        status->simplex[c] = simplex[s[c]];
      }
      status->gjk_iterations = k;
      return 1;
    }
    const Array3& dir = normals[index];
    simplex[s[index]] =
        find_support(first, second, dir, {-dir[0], -dir[1], -dir[2]});
    if (dot(dir, simplex[s[index]].point) < 0) {
      status->simplex_size = 0;
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

// The distance between the geoms by Gilbert–Johnson–Keerthi, its witness
// points, and the simplex it ended on (gjk).
auto run_gjk(InOut<CcdStatus> status, InOut<Object> first, InOut<Object> second)
    -> void {
  bool get_dist = status->dist_cutoff > 0;
  bool backup_gjk = !get_dist;
  std::array<Vertex, 4>& simplex = status->simplex;
  int n = 0;
  int k = 0;
  int kmax = status->max_iterations;
  Array3& x1_k = status->x1[0];
  Array3& x2_k = status->x2[0];
  std::array<double, 4> lambda{};
  double tol2 = status->tolerance * status->tolerance;
  status->separated = false;
  bool discrete = are_discrete(*first, *second);
  double epsilon = discrete ? 0 : 0.5 * tol2;
  double min_norm = discrete ? MINVAL : status->tolerance;
  Array3 x_k = subtract(x1_k, x2_k);
  double x_norm = norm(x_k);
  double x_norm_prev = 0;

  for (; k < kmax; k++) {
    if (x_norm < min_norm || std::abs(x_norm_prev - x_norm) < MINVAL) {
      break;
    }
    simplex[n] = find_gjk_support(first, second, x_k, x_norm);
    const Array3& s_k = simplex[n].point;
    if (dot(x_k, subtract(x_k, s_k)) < epsilon) {
      break;
    }
    double lower = dot(x_k, s_k);
    if (!get_dist) {
      if (lower > 0) {
        status->separated = true;
        status->gjk_iterations = k;
        status->simplex_size = 0;
        status->witness_count = 0;
        status->dist[0] = MAX_LIMIT;
        return;
      }
    } else if (status->dist_cutoff < MAX_LIMIT) {
      if (lower > 0 && lower >= status->dist_cutoff * x_norm) {
        status->separated = true;
        status->gjk_iterations = k;
        status->simplex_size = 0;
        status->witness_count = 0;
        status->dist[0] = MAX_LIMIT;
        return;
      }
    }
    if (n == 3 && backup_gjk) {
      status->gjk_iterations = k;
      int ret = intersect_gjk(status, first, second);
      if (ret != -1) {
        status->witness_count = 0;
        status->separated = ret == 0;
        status->dist[0] = ret > 0 ? 0 : MAX_LIMIT;
        return;
      }
      k = status->gjk_iterations;
      backup_gjk = false;
    }
    lambda = weigh_simplex(n + 1, simplex);
    n = 0;
    for (int i = 0; i < 4; i++) {
      if (lambda[i] == 0) {
        continue;
      }
      simplex[n] = simplex[i];
      lambda[n++] = lambda[i];
    }
    if (n < 1) {
      status->gjk_iterations = k;
      status->simplex_size = 0;
      status->witness_count = 0;
      status->dist[0] = MAX_LIMIT;
      status->separated = true;
      return;
    }
    x_k = combine_points(lambda, n, simplex[0].point, simplex[1].point,
                         simplex[2].point, simplex[3].point);
    x_norm_prev = x_norm;
    x_norm = norm(x_k);
    if (n == 4) {
      break;
    }
  }
  if (n > 0) {
    x1_k = combine_points(lambda, n, simplex[0].first, simplex[1].first,
                          simplex[2].first, simplex[3].first);
    x2_k = combine_points(lambda, n, simplex[0].second, simplex[1].second,
                          simplex[2].second, simplex[3].second);
  }
  Vertex tmp = find_gjk_support(first, second, x_k, x_norm);
  if (dot(x_k, tmp.point) > 0) {
    status->separated = true;
  }
  status->witness_count = 1;
  status->gjk_iterations = k;
  status->simplex_size = n;
  status->dist[0] = (n == 4 && !status->separated) ? 0 : x_norm;
}

// Starts over from three of the polytope's vertices (replaceSimplex3).
inline auto replace_simplex(InOut<Polytope> polytope, InOut<CcdStatus> status,
                            int v1, int v2, int v3) -> void {
  status->simplex_size = 3;
  status->simplex[0] = polytope->vertices[v1];
  status->simplex[1] = polytope->vertices[v2];
  status->simplex[2] = polytope->vertices[v3];
  polytope->face_count = 0;
  polytope->vertex_count = 0;
  polytope->map_size = 0;
}

// Whether `p3` and the origin lie on the same side of the plane through the
// other three (sameSide).
auto lie_on_same_side(const Array3& p0, const Array3& p1, const Array3& p2,
                      const Array3& p3) -> bool {
  Array3 diff1 = subtract(p1, p0);
  Array3 diff2 = subtract(p2, p0);
  Array3 n = cross(diff1, diff2);
  Array3 diff3 = subtract(p3, p0);
  double dot1 = dot(n, diff3);
  Array3 diff4 = scale(p0, -1);
  double dot2 = dot(n, diff4);
  return (dot1 > 0 && dot2 > 0) || (dot1 < 0 && dot2 < 0);
}

// Whether a tetrahedron holds the origin (testTetra).
auto contains_origin(const Array3& p0, const Array3& p1, const Array3& p2,
                     const Array3& p3) -> bool {
  return lie_on_same_side(p0, p1, p2, p3) && lie_on_same_side(p1, p2, p3, p0) &&
         lie_on_same_side(p2, p3, p0, p1) && lie_on_same_side(p3, p0, p1, p2);
}

// A turn by a third of a circle about `axis` (rotmat).
auto compute_third_turn(const Array3& axis) -> Matrix3 {
  double n = norm(axis);
  double u1 = axis[0] / n;
  double u2 = axis[1] / n;
  double u3 = axis[2] / n;
  const double s = 0.86602540378;
  const double c = -0.5;
  return {c + u1 * u1 * (1 - c),      u1 * u2 * (1 - c) - u3 * s,
          u1 * u3 * (1 - c) + u2 * s, u2 * u1 * (1 - c) + u3 * s,
          c + u2 * u2 * (1 - c),      u2 * u3 * (1 - c) - u1 * s,
          u1 * u3 * (1 - c) - u2 * s, u2 * u3 * (1 - c) + u1 * s,
          c + u3 * u3 * (1 - c)};
}

// Which way the segment from `v1` to `v2` passes the triangle of the other
// three: 1 or -1 through it, 0 beside it (rayTriangle).
inline auto cross_triangle(const Array3& v1, const Array3& v2, const Array3& v3,
                           const Array3& v4, const Array3& v5) -> int {
  Array3 diff12 = subtract(v2, v1);
  Array3 diff13 = subtract(v3, v1);
  Array3 diff14 = subtract(v4, v1);
  Array3 diff15 = subtract(v5, v1);
  double vol1 = compute_determinant(diff13, diff14, diff12);
  double vol2 = compute_determinant(diff14, diff15, diff12);
  double vol3 = compute_determinant(diff15, diff13, diff12);
  if (vol1 >= 0 && vol2 >= 0 && vol3 >= 0) {
    return 1;
  }
  if (vol1 <= 0 && vol2 <= 0 && vol3 <= 0) {
    return -1;
  }
  return 0;
}

// Appends `v` to the polytope and returns its index (insertVertex).
inline auto append_vertex(InOut<Polytope> polytope, const Vertex& v) -> int {
  int n = polytope->vertex_count++;
  polytope->vertices[n] = v;
  return n;
}

// deleteFace.
auto delete_face(InOut<Polytope> polytope, InOut<Face> face) -> void {
  if (face->index >= 0) {
    polytope->map[face->index] = polytope->map[--polytope->map_size];
    polytope->map[face->index]->index = face->index;
  }
  face->index = -2;
}

// maxFaces.
inline auto count_free_faces(const Polytope& polytope) -> int {
  return polytope.max_faces - polytope.face_count;
}

// Appends a face on three vertices and returns its squared distance from the
// origin, 0 if they lie on a line (attachFace).
inline auto attach_face(InOut<Polytope> polytope, int v1, int v2, int v3,
                        int adj1, int adj2, int adj3) -> double {
  Face& face = polytope->faces[polytope->face_count++];
  face.verts = v1 + (v2 << 10) + (v3 << 20);
  face.adj = {adj1, adj2, adj3};
  std::optional<Array3> projected = project_origin_plane(
      polytope->vertices[v3].point, polytope->vertices[v2].point,
      polytope->vertices[v1].point);
  if (!projected) {
    return 0;
  }
  face.closest = *projected;
  Array3 outward = subtract(polytope->vertices[v1].point, polytope->center);
  if (dot(face.closest, outward) < 0) {
    face.closest = scale(face.closest, -1);
  }
  face.dist2 = dot(face.closest, face.closest);
  face.index = -1;
  return face.dist2;
}

// The affine coordinates of `p` in a triangle (triAffineCoord).
auto compute_affine_coordinates(const Array3& v1, const Array3& v2,
                                const Array3& v3, const Array3& p) -> Array3 {
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
  return {c31 / m_max, c32 / m_max, c33 / m_max};
}

// Whether `p` lies in a triangle (triPointIntersect).
auto lies_in_triangle(const Array3& v1, const Array3& v2, const Array3& v3,
                      const Array3& p) -> bool {
  Array3 lambda = compute_affine_coordinates(v1, v2, v3, p);
  if (lambda[0] < 0 || lambda[1] < 0 || lambda[2] < 0) {
    return false;
  }
  Array3 pr{v1[0] * lambda[0] + v2[0] * lambda[1] + v3[0] * lambda[2],
            v1[1] * lambda[0] + v2[1] * lambda[1] + v3[1] * lambda[2],
            v1[2] * lambda[0] + v2[2] * lambda[1] + v3[2] * lambda[2]};
  return norm(subtract(pr, p)) < MINVAL;
}

// The first polytope from GJK's triangle: a bipyramid on it (polytope3).
auto build_triangle_polytope(InOut<Polytope> polytope, InOut<CcdStatus> status,
                             InOut<Object> first, InOut<Object> second) -> int {
  Array3 n{};
  double n_norm = 0.0;
  {
    const Array3& s1 = status->simplex[0].point;
    const Array3& s2 = status->simplex[1].point;
    const Array3& s3 = status->simplex[2].point;
    polytope->center = add(s1, s2);
    polytope->center = add(polytope->center, s3);
    polytope->center = scale(polytope->center, 1.0 / 3.0);
    Array3 diff1 = subtract(s2, s1);
    Array3 diff2 = subtract(s3, s1);
    n = cross(diff1, diff2);
    n_norm = norm(n);
  }
  if (n_norm < MINVAL) {
    return EPA_P3_BAD_NORMAL;
  }
  Array3 n_neg = scale(n, -1);
  int v1i = append_vertex(polytope, status->simplex[0]);
  int v2i = append_vertex(polytope, status->simplex[1]);
  int v3i = append_vertex(polytope, status->simplex[2]);
  int v5i = append_epa_support(polytope, first, second, n_neg, n_norm);
  int v4i = append_epa_support(polytope, first, second, n, n_norm);
  const Array3& v1 = polytope->vertices[v1i].point;
  const Array3& v2 = polytope->vertices[v2i].point;
  const Array3& v3 = polytope->vertices[v3i].point;
  const Array3& v4 = polytope->vertices[v4i].point;
  const Array3& v5 = polytope->vertices[v5i].point;
  if (lies_in_triangle(v1, v2, v3, v4)) {
    return EPA_P3_INVALID_V4;
  }
  if (lies_in_triangle(v1, v2, v3, v5)) {
    return EPA_P3_INVALID_V5;
  }
  if (status->dist[0] > 10 * MINVAL && !contains_origin(v1, v2, v3, v4) &&
      !contains_origin(v1, v2, v3, v5)) {
    return EPA_P3_MISSING_ORIGIN;
  }
  if (attach_face(polytope, v4i, v1i, v2i, 1, 3, 2) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attach_face(polytope, v4i, v3i, v1i, 2, 4, 0) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attach_face(polytope, v4i, v2i, v3i, 0, 5, 1) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attach_face(polytope, v5i, v2i, v1i, 5, 0, 4) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attach_face(polytope, v5i, v1i, v3i, 3, 1, 5) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  if (attach_face(polytope, v5i, v3i, v2i, 4, 2, 3) < MINDIST3) {
    return EPA_P3_ORIGIN_ON_FACE;
  }
  for (int i = 0; i < 6; i++) {
    polytope->map[i] = &polytope->faces[i];
    polytope->faces[i].index = i;
  }
  polytope->map_size = 6;
  return 0;
}

// The first polytope from GJK's segment: a bipyramid on three supports
// around it (polytope2).
auto build_segment_polytope(InOut<Polytope> polytope, InOut<CcdStatus> status,
                            InOut<Object> first, InOut<Object> second) -> int {
  Array3 diff{};
  {
    const Array3& s1 = status->simplex[0].point;
    const Array3& s2 = status->simplex[1].point;
    polytope->center = add(s1, s2);
    polytope->center = scale(polytope->center, 0.5);
    diff = subtract(s2, s1);
  }
  double value = MAX_LIMIT;
  int index = 0;
  for (int i = 0; i < 3; i++) {
    if (std::abs(diff[i]) < value) {
      value = std::abs(diff[i]);
      index = i;
    }
  }
  Array3 e{0, 0, 0};
  e[index] = 1;
  Array3 d1 = cross(e, diff);
  Matrix3 r = compute_third_turn(diff);
  Array3 d2 = multiply(r, d1);
  Array3 d3 = multiply(r, d2);
  int v1i = append_vertex(polytope, status->simplex[0]);
  int v2i = append_vertex(polytope, status->simplex[1]);
  int v3i = append_epa_support(polytope, first, second, d1, norm(d1));
  int v4i = append_epa_support(polytope, first, second, d2, norm(d2));
  int v5i = append_epa_support(polytope, first, second, d3, norm(d3));
  if (attach_face(polytope, v1i, v3i, v4i, 1, 3, 2) < MINDIST2) {
    replace_simplex(polytope, status, v1i, v3i, v4i);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (attach_face(polytope, v1i, v5i, v3i, 2, 4, 0) < MINDIST2) {
    replace_simplex(polytope, status, v1i, v5i, v3i);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (attach_face(polytope, v1i, v4i, v5i, 0, 5, 1) < MINDIST2) {
    replace_simplex(polytope, status, v1i, v4i, v5i);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (attach_face(polytope, v2i, v4i, v3i, 5, 0, 4) < MINDIST2) {
    replace_simplex(polytope, status, v2i, v4i, v3i);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (attach_face(polytope, v2i, v3i, v5i, 3, 1, 5) < MINDIST2) {
    replace_simplex(polytope, status, v2i, v3i, v5i);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (attach_face(polytope, v2i, v5i, v4i, 4, 2, 3) < MINDIST2) {
    replace_simplex(polytope, status, v2i, v5i, v4i);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (!cross_triangle(
          polytope->vertices[v1i].point, polytope->vertices[v2i].point,
          polytope->vertices[v3i].point, polytope->vertices[v4i].point,
          polytope->vertices[v5i].point)) {
    return EPA_P2_NONCONVEX;
  }
  for (int i = 0; i < 6; i++) {
    polytope->map[i] = &polytope->faces[i];
    polytope->faces[i].index = i;
  }
  polytope->map_size = 6;
  return 0;
}

// The first polytope from GJK's tetrahedron: itself (polytope4).
auto build_tetrahedron_polytope(InOut<Polytope> polytope,
                                InOut<CcdStatus> status, InOut<Object> first,
                                InOut<Object> second) -> int {
  int v1 = append_vertex(polytope, status->simplex[0]);
  int v2 = append_vertex(polytope, status->simplex[1]);
  int v3 = append_vertex(polytope, status->simplex[2]);
  int v4 = append_vertex(polytope, status->simplex[3]);
  polytope->center =
      add(polytope->vertices[v1].point, polytope->vertices[v2].point);
  polytope->center = add(polytope->center, polytope->vertices[v3].point);
  polytope->center = add(polytope->center, polytope->vertices[v4].point);
  polytope->center = scale(polytope->center, 0.25);
  if (attach_face(polytope, v1, v2, v3, 1, 3, 2) < MINDIST4) {
    replace_simplex(polytope, status, v1, v2, v3);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (attach_face(polytope, v1, v4, v2, 2, 3, 0) < MINDIST4) {
    replace_simplex(polytope, status, v1, v4, v2);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (attach_face(polytope, v1, v3, v4, 0, 3, 1) < MINDIST4) {
    replace_simplex(polytope, status, v1, v3, v4);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (attach_face(polytope, v4, v3, v2, 2, 0, 1) < MINDIST4) {
    replace_simplex(polytope, status, v4, v3, v2);
    return build_triangle_polytope(polytope, status, first, second);
  }
  if (!contains_origin(
          polytope->vertices[v1].point, polytope->vertices[v2].point,
          polytope->vertices[v3].point, polytope->vertices[v4].point)) {
    return EPA_P4_MISSING_ORIGIN;
  }
  for (int i = 0; i < 4; i++) {
    polytope->map[i] = &polytope->faces[i];
    polytope->faces[i].index = i;
  }
  polytope->map_size = 4;
  return 0;
}

// addEdge.
inline auto append_horizon_edge(InOut<Polytope> polytope, int index, int edge)
    -> void {
  polytope->horizon_edges[polytope->edge_count] = edge;
  polytope->horizon_indices[polytope->edge_count++] = index;
}

// The edge of `face` that starts at `vertex` (getEdge).
inline auto find_edge(const Face& face, int vertex) -> int {
  Indices verts = unpack_vertices(face.verts);
  if (verts[0] == vertex) {
    return 0;
  }
  if (verts[1] == vertex) {
    return 1;
  }
  return 2;
}

// Deletes `face` if the horizon point sees it, and walks on across its other
// edges, keeping those whose far face it does not see (horizonRec).
auto cut_horizon(InOut<Polytope> polytope, InOut<Face> face, int e) -> bool {
  if (dot(face->closest, polytope->horizon_point) - face->dist2 > MINVAL) {
    Indices verts = unpack_vertices(face->verts);
    delete_face(polytope, face);
    for (int k = 1; k < 3; k++) {
      int i = (e + k) % 3;
      Face& adj = polytope->faces[face->adj[i]];
      if (adj.index > -2) {
        int adj_edge = find_edge(adj, verts[(i + 1) % 3]);
        if (!cut_horizon(polytope, InOut(adj), adj_edge)) {
          append_horizon_edge(polytope, face->adj[i], adj_edge);
        }
      }
    }
    return true;
  }
  return false;
}

// The horizon the horizon point sees, starting from `face` (horizon).
auto find_horizon(InOut<Polytope> polytope, InOut<Face> face) -> void {
  delete_face(polytope, face);
  Indices verts = unpack_vertices(face->verts);
  Face* adj = &polytope->faces[face->adj[0]];
  int adj_edge = find_edge(*adj, verts[1]);
  if (!cut_horizon(polytope, InOut(*adj), adj_edge)) {
    append_horizon_edge(polytope, face->adj[0], adj_edge);
  }
  adj = &polytope->faces[face->adj[1]];
  adj_edge = find_edge(*adj, verts[2]);
  if (adj->index > -2 && !cut_horizon(polytope, InOut(*adj), adj_edge)) {
    append_horizon_edge(polytope, face->adj[1], adj_edge);
  }
  adj = &polytope->faces[face->adj[2]];
  adj_edge = find_edge(*adj, verts[0]);
  if (adj->index > -2 && !cut_horizon(polytope, InOut(*adj), adj_edge)) {
    append_horizon_edge(polytope, face->adj[2], adj_edge);
  }
}

// The witness points on each geom of `face`'s nearest point, and the
// penetration, negative (epaWitness).
auto compute_epa_witness(const Polytope& polytope, const Face& face,
                         Out<Array3> x1, Out<Array3> x2) -> double {
  Indices verts = unpack_vertices(face.verts);
  const Vertex& v1 = polytope.vertices[verts[0]];
  const Vertex& v2 = polytope.vertices[verts[1]];
  const Vertex& v3 = polytope.vertices[verts[2]];
  Array3 lambda =
      compute_affine_coordinates(v1.point, v2.point, v3.point, face.closest);
  *x1 = combine_points(lambda, 3, v1.first, v2.first, v3.first);
  *x2 = combine_points(lambda, 3, v1.second, v2.second, v3.second);
  return -std::sqrt(face.dist2);
}

// The penetration by the expanding polytope, and the face nearest the
// origin, or nothing (epa).
auto run_epa(InOut<CcdStatus> status, InOut<Polytope> polytope,
             InOut<Object> first, InOut<Object> second) -> Face* {
  double upper = MAX_LIMIT;
  double upper2 = MAX_LIMIT;
  double lower2 = 0.0;
  Face* face = nullptr;
  Face* pface = nullptr;
  bool discrete = are_discrete(*first, *second);
  double tolerance = discrete ? MINEPATOL : status->tolerance;
  int k = 0;
  int kmax = status->max_iterations < 1000 ? status->max_iterations : 1000;
  for (k = 0; k < kmax; k++) {
    pface = face;
    lower2 = MAX_LIMIT;
    for (int i = 0; i < polytope->map_size; i++) {
      if (polytope->map[i]->dist2 < lower2) {
        face = polytope->map[i];
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
    int wi = append_epa_support(polytope, first, second, face->closest, lower);
    const Vertex& w = polytope->vertices[wi];
    double upper_k = dot(face->closest, w.point) / lower;
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
      int nverts = polytope->vertex_count - 1;
      for (; i < nverts; i++) {
        if (w.first_index == polytope->vertices[i].first_index &&
            w.second_index == polytope->vertices[i].second_index) {
          break;
        }
      }
      if (i != nverts) {
        break;
      }
    }
    polytope->horizon_point = w.point;
    find_horizon(polytope, InOut(*face));
    if (polytope->edge_count < 3) {
      face = nullptr;
      break;
    }
    int nfaces = polytope->face_count;
    int nedges = polytope->edge_count;
    if (nedges > count_free_faces(*polytope)) {
      break;
    }
    int hzn_index = polytope->horizon_indices[0];
    int hzn_edge = polytope->horizon_edges[0];
    Face* hzn_face = &polytope->faces[hzn_index];
    Indices hzn_verts = unpack_vertices(hzn_face->verts);
    int v1 = hzn_verts[hzn_edge];
    int v2 = hzn_verts[(hzn_edge + 1) % 3];
    hzn_face->adj[hzn_edge] = nfaces;
    double dist2 = attach_face(polytope, wi, v2, v1, nfaces + nedges - 1,
                               hzn_index, nfaces + 1);
    if (dist2 == 0) {
      face = nullptr;
      break;
    }
    if (dist2 >= lower2 && dist2 <= upper2) {
      int i = polytope->map_size++;
      polytope->map[i] = &polytope->faces[polytope->face_count - 1];
      polytope->map[i]->index = i;
    }
    for (int i = 1; i < nedges; i++) {
      int cur = nfaces + i;
      int next = nfaces + (i + 1) % nedges;
      hzn_index = polytope->horizon_indices[i];
      hzn_edge = polytope->horizon_edges[i];
      hzn_face = &polytope->faces[hzn_index];
      Indices verts2 = unpack_vertices(hzn_face->verts);
      v1 = verts2[hzn_edge];
      v2 = verts2[(hzn_edge + 1) % 3];
      hzn_face->adj[hzn_edge] = cur;
      dist2 = attach_face(polytope, wi, v2, v1, cur - 1, hzn_index, next);
      if (dist2 == 0) {
        face = nullptr;
        break;
      }
      if (dist2 >= lower2 && dist2 <= upper2) {
        int idx = polytope->map_size++;
        polytope->map[idx] = &polytope->faces[polytope->face_count - 1];
        polytope->map[idx]->index = idx;
      }
    }
    polytope->edge_count = 0;
    if (!polytope->map_size || face == nullptr) {
      break;
    }
  }
  status->epa_iterations = k;
  if (face != nullptr) {
    status->dist[0] = compute_epa_witness(*polytope, *face, Out(status->x1[0]),
                                          Out(status->x2[0]));
    status->witness_count = 1;
  } else {
    status->witness_count = 0;
    status->dist[0] = 0;
  }
  return face;
}

//-- Multiple contacts ---------------------------------------------------------

// The area of the quadrilateral on four of `hull`'s points (area4).
inline auto compute_quad_area(std::span<const Array3> hull, int a, int b, int c,
                              int d) -> double {
  Array3 ca = subtract(hull[a], hull[c]);
  Array3 db = subtract(hull[b], hull[d]);
  return 0.5 * norm(cross(ca, db));
}

// The four points of a convex polygon spanning the largest area (hull4).
inline auto find_largest_quad(std::span<const Array3> hull)
    -> std::array<int, 4> {
  int nhull = static_cast<int>(hull.size());
  int a = 0;
  int b = 1;
  int c = 2;
  int d = 3;
  std::array<int, 4> res{0, 1, 2, 3};
  double m = compute_quad_area(hull, a, b, c, d);
  double m_next = 0.0;
  for (; a < nhull; a++) {
    while (true) {
      int d_next = (d + 1) % nhull;
      m_next = compute_quad_area(hull, a, b, c, d_next);
      if (m_next <= m) {
        break;
      }
      d = d_next, m = m_next;
      res = {a, b, c, d};
      while (true) {
        int c_next = (c + 1) % nhull;
        m_next = compute_quad_area(hull, a, b, c_next, d);
        if (m_next <= m) {
          break;
        }
        c = c_next, m = m_next;
        res = {a, b, c, d};
      }
      while (true) {
        int b_next = (b + 1) % nhull;
        m_next = compute_quad_area(hull, a, b_next, c, d);
        if (m_next <= m) {
          break;
        }
        b = b_next, m = m_next;
        res = {a, b, c, d};
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
  return res;
}

// The plane through the edge from `v1` to `v2` along `n`: its unit normal,
// and its offset (planeNormal).
auto compute_side_plane(Out<Array3> normal, const Array3& v1, const Array3& v2,
                        const Array3& n) -> double {
  Array3 v3 = add(v1, n);
  Array3 diff1 = subtract(v2, v1);
  Array3 diff2 = subtract(v3, v1);
  Array3 res = cross(diff1, diff2);
  normalize3(InOut(res));
  *normal = res;
  return dot(res, v1);
}

// Whether `p` lies inside the plane through `a` with normal `n` (halfspace).
auto lies_inside(const Array3& a, const Array3& n, const Array3& p) -> bool {
  return dot(subtract(p, a), n) > -MINVAL;
}

// The witness points of `v` on a face through `p` with normal `n`, moved
// back along `dir` by its distance, and that distance (witnessOnFace).
inline auto place_witness(Out<Array3> w1, Out<Array3> w2, const Array3& v,
                          const Array3& p, const Array3& n, const Array3& dir)
    -> double {
  double dist = dot(subtract(v, p), n);
  *w1 = add_scaled(v, dir, -std::abs(dist));
  *w2 = v;
  return dist;
}

// The contacts of `face2` clipped to `face1`'s sides, below `face1`, with
// normal `n`, their witnesses moved along `dir` (polygonClip).
auto clip_polygon(InOut<CcdStatus> status, std::span<const Array3> face1,
                  std::span<const Array3> face2, const Array3& n,
                  const Array3& dir) -> void {
  int nface1 = static_cast<int>(face1.size());
  int nface2 = static_cast<int>(face2.size());
  if (nface1 < 3) {
    return;
  }

  // Each side's plane. Here and below, only what is written is read, so the
  // scratch arrays are left uninitialized.
  std::array<Array3, MAX_POLYGON> pn;
  std::array<double, MAX_POLYGON> pd;
  for (int i = 0; i < nface1 - 1; i++) {
    pd[i] = compute_side_plane(Out(pn[i]), face1[i], face1[i + 1], n);
  }
  pd[nface1 - 1] =
      compute_side_plane(Out(pn[nface1 - 1]), face1[nface1 - 1], face1[0], n);

  // Sutherland–Hodgman: each clip adds at most one point.
  std::array<std::array<Array3, 2 * MAX_POLYGON>, 2> polygons;
  int current = 0;
  int npolygon = nface2;
  int nclipped = 0;
  std::ranges::copy(face2, polygons[current].begin());
  for (int e = 0; e < nface1; e++) {
    const std::array<Array3, 2 * MAX_POLYGON>& polygon = polygons[current];
    std::array<Array3, 2 * MAX_POLYGON>& clipped = polygons[1 - current];
    for (int i = 0; i < npolygon; i++) {
      const Array3& p = polygon[i];
      const Array3& q = (i < npolygon - 1) ? polygon[i + 1] : polygon[0];
      Array3 pq = subtract(q, p);
      bool inside1 = lies_inside(face1[e], pn[e], p);
      bool inside2 = lies_inside(face1[e], pn[e], q);
      if (!inside1 && !inside2) {
        continue;
      }
      if (inside1 && inside2) {
        clipped[nclipped++] = q;
        continue;
      }
      double tmp = dot(pn[e], pq);
      if (tmp != 0.0) {
        double t = (pd[e] - dot(pn[e], p)) / tmp;
        if (t >= 0.0 && t <= 1.0) {
          clipped[nclipped++] = add_scaled(p, pq, t);
        }
      }
      if (inside2) {
        clipped[nclipped++] = q;
      }
    }
    current = 1 - current;
    npolygon = nclipped;
    nclipped = 0;
  }

  // The points below `face1`.
  std::array<Array3, 2 * MAX_POLYGON>& polygon = polygons[current];
  int m = npolygon;
  npolygon = 0;
  for (int i = 0; i < m; i++) {
    if (dot(subtract(polygon[i], face1[0]), n) <= 0) {
      if (npolygon != i) {
        polygon[npolygon] = polygon[i];
      }
      npolygon++;
    }
  }
  if (npolygon < 1) {
    return;
  }

  if (status->max_contacts < 5 && npolygon > 4) {
    status->witness_count = 4;
    std::array<int, 4> idx = find_largest_quad(
        std::span<const Array3>{polygon.data(), std::size_t(npolygon)});
    for (int i = 0; i < 4; i++) {
      status->dist[i] = place_witness(Out(status->x1[i]), Out(status->x2[i]),
                                      polygon[idx[i]], face1[0], n, dir);
    }
    return;
  }
  if (nface2 == 2 && npolygon > 2) {
    int best1 = 0;
    int best2 = 1;
    double d = 0;
    for (int i = 0; i < npolygon; i++) {
      for (int j = i + 1; j < npolygon; j++) {
        Array3 diff = subtract(polygon[j], polygon[i]);
        double d2 = dot(diff, diff);
        if (d2 > d) {
          d = d2;
          best1 = i;
          best2 = j;
        }
      }
    }
    status->dist[0] = place_witness(Out(status->x1[0]), Out(status->x2[0]),
                                    polygon[best1], face1[0], n, dir);
    status->dist[1] = place_witness(Out(status->x1[1]), Out(status->x2[1]),
                                    polygon[best2], face1[0], n, dir);
    status->witness_count = 2;
    return;
  }
  npolygon = std::min(npolygon, MAXCONPAIR);
  for (int i = 0; i < npolygon; i++) {
    status->dist[i] = place_witness(Out(status->x1[i]), Out(status->x2[i]),
                                    polygon[i], face1[0], n, dir);
  }
  status->witness_count = npolygon;
}

// A cylinder's cap normal, if the simplex is one vertex on it
// (cylinderNormals).
auto find_cylinder_normals(Out<Triple> normals, Out<Indices> indices, int dim,
                           const Object& object, const Indices& vi) -> int {
  if (dim == 1) {
    (*normals)[0] = multiply(object.mat, Array3{0, 0, vi[0] ? -1.0 : 1.0});
    (*indices)[0] = vi[0];
    return 1;
  }
  return 0;
}

// The box face whose normal lies within FACE_TOL of `n` (boxNormals2).
auto find_box_face_normal(Out<Triple> normals, Out<Indices> indices,
                          const Matrix3& mat, const Array3& n) -> int {
  static constexpr std::array<Array3, 6> FACES{
      {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
  Array3 local_n = multiply_transposed(mat, n);
  local_n = scale(local_n, 1 / std::sqrt(dot(local_n, local_n)));
  for (int i = 0; i < 6; i++) {
    if (dot(local_n, FACES[i]) > FACE_TOL) {
      (*normals)[0] = multiply(mat, FACES[i]);
      (*indices)[0] = i;
      return 1;
    }
  }
  return 0;
}

// The normals of the box faces that hold the simplex's vertices
// (boxNormals).
auto find_box_normals(Out<Triple> normals, Out<Indices> indices, int dim,
                      const Object& object, const Indices& vi,
                      const Array3& dir) -> int {
  int v1 = vi[0];
  int v2 = vi[1];
  int v3 = vi[2];
  const Matrix3& mat = object.mat;
  if (dim == 3) {
    int c = 0;
    int x = ((v1 & 1) && (v2 & 1) && (v3 & 1)) -
            (!(v1 & 1) && !(v2 & 1) && !(v3 & 1));
    int y = ((v1 & 2) && (v2 & 2) && (v3 & 2)) -
            (!(v1 & 2) && !(v2 & 2) && !(v3 & 2));
    int z = ((v1 & 4) && (v2 & 4) && (v3 & 4)) -
            (!(v1 & 4) && !(v2 & 4) && !(v3 & 4));
    (*normals)[0] = multiply(mat, Array3{double(x), double(y), double(z)});
    int sgn = x + y + z;
    if (x) {
      (*indices)[c++] = 0;
    }
    if (y) {
      (*indices)[c++] = 2;
    }
    if (z) {
      (*indices)[c++] = 4;
    }
    if (sgn == -1) {
      (*indices)[0]++;
    }
    return c == 1 ? 1 : find_box_face_normal(normals, indices, mat, dir);
  }
  if (dim == 2) {
    int c = 0;
    int x = ((v1 & 1) && (v2 & 1)) - (!(v1 & 1) && !(v2 & 1));
    int y = ((v1 & 2) && (v2 & 2)) - (!(v1 & 2) && !(v2 & 2));
    int z = ((v1 & 4) && (v2 & 4)) - (!(v1 & 4) && !(v2 & 4));
    if (x) {
      (*normals)[0] = multiply(mat, Array3{double(x), 0, 0});
      (*indices)[c++] = (x > 0) ? 0 : 1;
    }
    if (y) {
      (*normals)[c] = multiply(mat, Array3{0, double(y), 0});
      (*indices)[c++] = (y > 0) ? 2 : 3;
    }
    if (z) {
      // MuJoCo writes the z normal second, wherever it falls.
      (*normals)[1] = multiply(mat, Array3{0, 0, double(z)});
      (*indices)[c++] = (z > 0) ? 4 : 5;
    }
    return c == 2 ? 2 : find_box_face_normal(normals, indices, mat, dir);
  }
  if (dim == 1) {
    double x = (v1 & 1) ? 1 : -1;
    double y = (v1 & 2) ? 1 : -1;
    double z = (v1 & 4) ? 1 : -1;
    (*normals)[0] = multiply(mat, Array3{x, 0, 0});
    (*normals)[1] = multiply(mat, Array3{0, y, 0});
    (*normals)[2] = multiply(mat, Array3{0, 0, z});
    (*indices)[0] = (x > 0) ? 0 : 1;
    (*indices)[1] = (y > 0) ? 2 : 3;
    (*indices)[2] = (z > 0) ? 4 : 5;
    return 3;
  }
  return 0;
}

// The directions of the box edges at the simplex's vertices, and their far
// ends (boxEdgeNormals).
auto find_box_edge_normals(Out<Triple> normals, Out<Triple> ends, int dim,
                           const Object& object, const Triple& v, int v1i)
    -> int {
  const Array3& v1 = v[0];
  const Array3& v2 = v[1];
  const Matrix3& mat = object.mat;
  const Array3& pos = object.pos;
  const Array3& size = object.size;
  if (dim == 2) {
    (*ends)[0] = v2;
    (*normals)[0] = subtract(v2, v1);
    normalize3(InOut((*normals)[0]));
    return 1;
  }
  if (dim == 1) {
    double x = (v1i & 1) ? size[0] : -size[0];
    double y = (v1i & 2) ? size[1] : -size[1];
    double z = (v1i & 4) ? size[2] : -size[2];
    (*ends)[0] = transform(mat, pos, {-x, y, z});
    (*normals)[0] = subtract((*ends)[0], v1);
    normalize3(InOut((*normals)[0]));
    (*ends)[1] = transform(mat, pos, {x, -y, z});
    (*normals)[1] = subtract((*ends)[1], v1);
    normalize3(InOut((*normals)[1]));
    (*ends)[2] = transform(mat, pos, {x, y, -z});
    (*normals)[2] = subtract((*ends)[2], v1);
    normalize3(InOut((*normals)[2]));
    return 3;
  }
  return 0;
}

// A cylinder's side edge at the simplex's vertex, and its far end
// (cylinderEdgeNormals).
auto find_cylinder_edge_normals(Out<Triple> normals, Out<Triple> ends, int dim,
                                const Object& object, const Triple& v, int v1i)
    -> int {
  if (dim == 1 || dim == 2) {
    double sgn = v1i ? 1.0 : -1.0;
    (*normals)[0] = {sgn * object.mat[2], sgn * object.mat[5],
                     sgn * object.mat[8]};
    (*ends)[0] = add_scaled(v[0], (*normals)[0], 2 * object.size[1]);
    return 1;
  }
  return 0;
}

// A cylinder's cap as a polygon of 16 points (cylinderFace).
auto compute_cylinder_face(Out<Polygon> face, const Object& object, int idx)
    -> int {
  static constexpr std::array<double, 16> COS16{
      1.000000000000000,  0.923879532511287,  0.707106781186548,
      0.382683432365090,  0.000000000000000,  -0.382683432365090,
      -0.707106781186547, -0.923879532511287, -1.000000000000000,
      -0.923879532511287, -0.707106781186548, -0.382683432365090,
      0.000000000000000,  0.382683432365090,  0.707106781186547,
      0.923879532511287};
  static constexpr std::array<double, 16> SIN16{
      0.000000000000000,  0.382683432365090,  0.707106781186547,
      0.923879532511287,  1.000000000000000,  0.923879532511287,
      0.707106781186548,  0.382683432365090,  0.000000000000000,
      -0.382683432365090, -0.707106781186547, -0.923879532511287,
      -1.000000000000000, -0.923879532511287, -0.707106781186548,
      -0.382683432365090};
  double sgn = idx ? -1.0 : 1.0;
  for (int i = 0; i < 16; i++) {
    double x = COS16[i] * object.size[0];
    double y = -SIN16[i] * object.size[0] * sgn;
    (*face)[i] =
        transform(object.mat, object.pos, {x, y, sgn * object.size[1]});
  }
  return 16;
}

// A box's face `idx` as its four corners (boxFace).
auto compute_box_face(Out<Polygon> face, const Object& object, int idx) -> int {
  const Matrix3& mat = object.mat;
  const Array3& pos = object.pos;
  const Array3& s = object.size;
  Polygon& f = *face;
  switch (idx) {
    case 0:
      f[0] = transform(mat, pos, {s[0], s[1], s[2]});
      f[1] = transform(mat, pos, {s[0], s[1], -s[2]});
      f[2] = transform(mat, pos, {s[0], -s[1], -s[2]});
      f[3] = transform(mat, pos, {s[0], -s[1], s[2]});
      return 4;
    case 1:
      f[0] = transform(mat, pos, {-s[0], s[1], -s[2]});
      f[1] = transform(mat, pos, {-s[0], s[1], s[2]});
      f[2] = transform(mat, pos, {-s[0], -s[1], s[2]});
      f[3] = transform(mat, pos, {-s[0], -s[1], -s[2]});
      return 4;
    case 2:
      f[0] = transform(mat, pos, {-s[0], s[1], -s[2]});
      f[1] = transform(mat, pos, {s[0], s[1], -s[2]});
      f[2] = transform(mat, pos, {s[0], s[1], s[2]});
      f[3] = transform(mat, pos, {-s[0], s[1], s[2]});
      return 4;
    case 3:
      f[0] = transform(mat, pos, {-s[0], -s[1], s[2]});
      f[1] = transform(mat, pos, {s[0], -s[1], s[2]});
      f[2] = transform(mat, pos, {s[0], -s[1], -s[2]});
      f[3] = transform(mat, pos, {-s[0], -s[1], -s[2]});
      return 4;
    case 4:
      f[0] = transform(mat, pos, {-s[0], s[1], s[2]});
      f[1] = transform(mat, pos, {s[0], s[1], s[2]});
      f[2] = transform(mat, pos, {s[0], -s[1], s[2]});
      f[3] = transform(mat, pos, {-s[0], -s[1], s[2]});
      return 4;
    case 5:
      f[0] = transform(mat, pos, {s[0], s[1], -s[2]});
      f[1] = transform(mat, pos, {-s[0], s[1], -s[2]});
      f[2] = transform(mat, pos, {-s[0], -s[1], -s[2]});
      f[3] = transform(mat, pos, {s[0], -s[1], -s[2]});
      return 4;
    default:
      return 0;
  }
}

// The first face of `v` facing a face of `w` (alignedFaces).
inline auto find_aligned_faces(std::span<const Array3> v,
                               std::span<const Array3> w)
    -> std::optional<std::array<int, 2>> {
  for (int i = 0; i < static_cast<int>(v.size()); i++) {
    for (int j = 0; j < static_cast<int>(w.size()); j++) {
      if (dot(v[i], w[j]) < -FACE_TOL) {
        return std::array<int, 2>{i, j};
      }
    }
  }
  return std::nullopt;
}

// The first edge lying in a face that faces along `dir` (alignedFaceEdge).
inline auto find_aligned_face_edge(std::span<const Array3> edge,
                                   std::span<const Array3> face,
                                   const Array3& dir)
    -> std::optional<std::array<int, 2>> {
  for (int i = 0; i < static_cast<int>(face.size()); i++) {
    if (dot(face[i], dir) <= MINVAL) {
      continue;
    }
    for (int j = 0; j < static_cast<int>(edge.size()); j++) {
      if (std::abs(dot(edge[j], face[i])) < EDGE_TOL) {
        return std::array<int, 2>{j, i};
      }
    }
  }
  return std::nullopt;
}

// How many distinct vertices the simplex has, moved to the front
// (simplexDim).
inline auto reduce_simplex(InOut<Indices> vi, InOut<Triple> v) -> int {
  if ((*vi)[0] == (*vi)[1]) {
    if ((*vi)[0] == (*vi)[2]) {
      return 1;
    }
    (*vi)[1] = (*vi)[2];
    (*v)[1] = (*v)[2];
    return 2;
  }
  return ((*vi)[2] == (*vi)[0] || (*vi)[2] == (*vi)[1]) ? 2 : 3;
}

auto find_normals(const Object& object, Out<Triple> normals,
                  Out<Indices> indices, int dim, const Indices& vi,
                  const Array3& dir) -> int {
  if (object.type == GeomType::BOX) {
    return find_box_normals(normals, indices, dim, object, vi, dir);
  }
  if (object.type == GeomType::CYLINDER) {
    return find_cylinder_normals(normals, indices, dim, object, vi);
  }
  return 0;
}

auto find_edge_normals(const Object& object, Out<Triple> normals,
                       Out<Triple> ends, int dim, const Triple& v, int v1i)
    -> int {
  if (object.type == GeomType::BOX) {
    return find_box_edge_normals(normals, ends, dim, object, v, v1i);
  }
  if (object.type == GeomType::CYLINDER) {
    return find_cylinder_edge_normals(normals, ends, dim, object, v, v1i);
  }
  return 0;
}

auto compute_face(const Object& object, Out<Polygon> face, int index) -> int {
  if (object.type == GeomType::BOX) {
    return compute_box_face(face, object, index);
  }
  if (object.type == GeomType::CYLINDER) {
    return compute_cylinder_face(face, object, index);
  }
  return 0;
}

// Where two faces, or a face and an edge, meet: the one clipped against the
// other (multicontact).
auto find_multicontact(const Polytope& polytope, const Face& face,
                       InOut<CcdStatus> status, const Object& first,
                       const Object& second) -> void {
  Indices verts = unpack_vertices(face.verts);
  Indices v1i{polytope.vertices[verts[0]].first_index,
              polytope.vertices[verts[1]].first_index,
              polytope.vertices[verts[2]].first_index};
  Indices v2i{polytope.vertices[verts[0]].second_index,
              polytope.vertices[verts[1]].second_index,
              polytope.vertices[verts[2]].second_index};
  Triple v1{};
  Triple v2{};
  for (int k = 0; k < 3; ++k) {
    v1[k] = polytope.vertices[verts[k]].first;
    v2[k] = polytope.vertices[verts[k]].second;
  }
  Triple n1{};
  Triple n2{};
  Indices idx1{};
  Indices idx2{};
  Triple ends{};
  Polygon face1{};
  Polygon face2{};
  int nface1 = reduce_simplex(InOut(v1i), InOut(v1));
  int nface2 = reduce_simplex(InOut(v2i), InOut(v2));
  Array3 dir = subtract(status->x2[0], status->x1[0]);
  Array3 dir_neg = subtract(status->x1[0], status->x2[0]);
  int nnorms1 = find_normals(first, Out(n1), Out(idx1), nface1, v1i, dir_neg);
  int nnorms2 = find_normals(second, Out(n2), Out(idx2), nface2, v2i, dir);
  auto first_of = [](const Triple& normals, int count) {
    return std::span<const Array3>{normals.data(), std::size_t(count)};
  };

  std::optional<std::array<int, 2>> res =
      find_aligned_faces(first_of(n1, nnorms1), first_of(n2, nnorms2));
  bool edgecon1 = false;
  bool edgecon2 = false;
  if (!res) {
    if (nface1 < 3 && nface1 <= nface2) {
      nnorms1 =
          find_edge_normals(first, Out(n1), Out(ends), nface1, v1, v1i[0]);
      res = find_aligned_face_edge(first_of(n1, nnorms1), first_of(n2, nnorms2),
                                   dir);
      if (!res) {
        return;
      }
      edgecon1 = true;
    } else if (nface2 < 3) {
      nnorms2 =
          find_edge_normals(second, Out(n2), Out(ends), nface2, v2, v2i[0]);
      res = find_aligned_face_edge(first_of(n2, nnorms2), first_of(n1, nnorms1),
                                   dir_neg);
      if (!res) {
        return;
      }
      edgecon2 = true;
    } else {
      return;
    }
  }
  int i = (*res)[0];
  int j = (*res)[1];
  if (edgecon1) {
    face1[0] = v1[0];
    face1[1] = ends[i];
    nface1 = 2;
  } else {
    nface1 = compute_face(first, Out(face1), edgecon2 ? idx1[j] : idx1[i]);
  }
  if (edgecon2) {
    face2[0] = v2[0];
    face2[1] = ends[i];
    nface2 = 2;
  } else {
    nface2 = compute_face(second, Out(face2), idx2[j]);
  }
  std::span<const Array3> polygon1{face1.data(), std::size_t(nface1)};
  std::span<const Array3> polygon2{face2.data(), std::size_t(nface2)};

  if (edgecon1) {
    clip_polygon(status, polygon2, polygon1, n2[j], scale(n2[j], -1.0));
    for (int k = 0; k < status->witness_count; k++) {
      std::swap(status->x1[k], status->x2[k]);
    }
    return;
  }
  if (edgecon2) {
    clip_polygon(status, polygon1, polygon2, n1[j], scale(n1[j], -1.0));
    return;
  }
  clip_polygon(status, polygon1, polygon2, n1[i], n2[j]);
}

// Moves the witness points out by the margins, and the distance with them
// (inflate).
inline auto inflate(InOut<CcdStatus> status, double margin1, double margin2)
    -> void {
  Array3 n = subtract(status->x2[0], status->x1[0]);
  normalize3(InOut(n));
  if (margin1 != 0) {
    status->x1[0][0] += margin1 * n[0];
    status->x1[0][1] += margin1 * n[1];
    status->x1[0][2] += margin1 * n[2];
  }
  if (margin2 != 0) {
    status->x2[0][0] -= margin2 * n[0];
    status->x2[0][1] -= margin2 * n[1];
    status->x2[0][2] -= margin2 * n[2];
  }
  status->dist[0] -= (margin1 + margin2);
}

// Distance, or penetration where negative, and witness points (mjc_ccd).
auto compute_ccd(int max_contacts, InOut<CcdStatus> status, InOut<Object> first,
                 InOut<Object> second) -> double {
  status->x1[0] = first->pos;
  status->x2[0] = second->pos;
  status->gjk_iterations = 0;
  status->epa_iterations = 0;
  status->epa_status = EPA_NOCONTACT;
  status->tolerance = CCD_TOLERANCE;
  status->max_iterations = CCD_ITERATIONS;
  status->max_contacts = max_contacts;
  status->dist_cutoff = 0;

  // A sphere or capsule shrunk to its point or segment, its radius added
  // back after.
  if (first->type == GeomType::SPHERE || second->type == GeomType::SPHERE ||
      first->type == GeomType::CAPSULE || second->type == GeomType::CAPSULE) {
    Support support1 = first->support;
    Support support2 = second->support;
    double full_margin1 = 0;
    double full_margin2 = 0;
    double margin1 = first->margin;
    double margin2 = second->margin;
    if (first->type == GeomType::SPHERE) {
      full_margin1 = first->size[0] + 0.5 * margin1;
      first->support = support_point;
      first->margin = 0;
    } else if (first->type == GeomType::CAPSULE) {
      full_margin1 = first->size[0] + 0.5 * margin1;
      first->support = support_line;
      first->margin = 0;
    }
    if (second->type == GeomType::SPHERE) {
      full_margin2 = second->size[0] + 0.5 * margin2;
      second->support = support_point;
      second->margin = 0;
    } else if (second->type == GeomType::CAPSULE) {
      full_margin2 = second->size[0] + 0.5 * margin2;
      second->support = support_line;
      second->margin = 0;
    }
    status->dist_cutoff += full_margin1 + full_margin2;
    run_gjk(status, first, second);
    status->dist_cutoff = 0;
    first->margin = margin1;
    second->margin = margin2;
    first->support = support1;
    second->support = support2;
    if (status->dist[0] > status->tolerance) {
      inflate(status, full_margin1, full_margin2);
      if (status->dist[0] > status->dist_cutoff) {
        status->dist[0] = MAX_LIMIT;
      }
      return status->dist[0];
    }
    status->gjk_iterations = 0;
    status->x1[0] = first->pos;
    status->x2[0] = second->pos;
  }

  run_gjk(status, first, second);
  if (status->dist[0] <= CCD_TOLERANCE && status->simplex_size > 1 &&
      !status->separated) {
    status->dist[0] = 0;
    Polytope polytope;
    int n = CCD_ITERATIONS;
    polytope.max_faces = 6 * n;
    polytope.vertices.resize(5 + std::size_t(n));
    polytope.faces.resize(std::size_t(6 * n));
    polytope.map.resize(std::size_t(6 * n));
    int ret = 0;
    if (status->simplex_size == 2) {
      ret = build_segment_polytope(InOut(polytope), status, first, second);
    } else if (status->simplex_size == 3) {
      ret = build_triangle_polytope(InOut(polytope), status, first, second);
    } else {
      ret = build_tetrahedron_polytope(InOut(polytope), status, first, second);
    }
    status->epa_status = ret;
    if (ret == 0) {
      Face* face = run_epa(status, InOut(polytope), first, second);
      if (max_contacts > 1 && face != nullptr) {
        find_multicontact(polytope, *face, status, *first, *second);
      }
    }
  }
  double min_dist = status->dist[0];
  for (int i = 1; i < status->witness_count; i++) {
    min_dist = std::min(min_dist, status->dist[i]);
  }
  return min_dist;
}

// Writes contacts from the witness points of a penetration and returns how
// many (mjc_penetration).
auto find_penetration(InOut<Object> first, InOut<Object> second,
                      std::span<PreContact> out, int max_contacts,
                      double margin) -> int {
  CcdStatus status;
  if (compute_ccd(max_contacts, InOut(status), first, second) >= 0) {
    return 0;
  }
  for (int i = 0; i < status.witness_count; i++) {
    PreContact& contact = out[i];
    contact.dist = margin + status.dist[i];
    contact.pos = scale(add(status.x1[i], status.x2[i]), 0.5);
    contact.normal = subtract(status.x1[i], status.x2[i]);
    normalize3(InOut(contact.normal));
    contact.tangent = {0, 0, 0};
  }
  return status.witness_count;
}

// How many contacts one pass may find (maxContacts).
auto count_max_contacts(const Object& first, const Object& second) -> int {
  if (first.margin > 0 || second.margin > 0) {
    return 1;
  }
  bool polygonal1 =
      first.type == GeomType::BOX || first.type == GeomType::CYLINDER;
  bool polygonal2 =
      second.type == GeomType::BOX || second.type == GeomType::CYLINDER;
  if (first.type == GeomType::BOX && second.type == GeomType::BOX) {
    return 8;
  }
  return polygonal1 && polygonal2 ? 4 : 1;
}

// Turns a frame by `rot` about `origin` (mju_rotateFrame).
auto rotate_frame(const Array3& origin, const Matrix3& rot, InOut<Matrix3> mat,
                  InOut<Array3> pos) -> void {
  Matrix3 turned{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      turned[3 * i + j] = rot[3 * i] * (*mat)[j] +
                          rot[3 * i + 1] * (*mat)[3 + j] +
                          rot[3 * i + 2] * (*mat)[6 + j];
    }
  }
  *mat = turned;
  Array3 rel = subtract(origin, *pos);
  Array3 vec = subtract(multiply(rot, rel), rel);
  *pos = subtract(*pos, vec);
}

}  // namespace

auto collide_convex(const Geom& first, const GeomFrame& first_frame,
                    double first_radius, const Geom& second,
                    const GeomFrame& second_frame, double second_radius,
                    double margin, std::span<PreContact, MAX_PAIR_CONTACTS> out)
    -> std::uint32_t {
  // A plane against a convex geom: its support point opposite the plane's
  // normal (mjc_PlaneConvex).
  if (first.type == GeomType::PLANE) {
    Object object = make_object(second, second_frame, 0);
    const Matrix3& mat = first_frame.mat;
    Array3 normal{mat[2], mat[5], mat[8]};
    Array3 v = object.support(InOut(object), {-mat[2], -mat[5], -mat[8]});
    double dist = dot(normal, subtract(v, first_frame.pos));
    if (dist > margin) {
      return 0;
    }
    out[0].dist = dist;
    for (int k = 0; k < 3; ++k) {
      out[0].pos[k] = v[k] + normal[k] * (-0.5 * dist);
    }
    out[0].normal = normal;
    out[0].tangent = {0, 0, 0};
    return 1;
  }

  Object object1 = make_object(first, first_frame, margin);
  Object object2 = make_object(second, second_frame, margin);
  int most = count_max_contacts(object1, object2);
  int ncon =
      find_penetration(InOut(object1), InOut(object2), out, most, margin);
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

    // mju_makeFrame of the normal alone: its two tangents are the axes.
    Array3 x = out[0].normal;
    normalize3(InOut(x));
    Array3 y{0, 0, 0};
    if (x[1] < 0.5 && x[1] > -0.5) {
      y[1] = 1;
    } else {
      y[2] = 1;
    }
    y = subtract(y, scale(x, dot(x, y)));
    normalize3(InOut(y));
    Array3 z = cross(x, y);

    double tolerance =
        RELATIVE_TOLERANCE * std::min(first_radius, second_radius);
    for (const Array3& axis : std::array<Array3, 2>{y, z}) {
      for (double angle : {-PERTURBATION, PERTURBATION}) {
        // mji_axisAngle2Quat, mju_quat2Mat.
        double s = std::sin(angle * 0.5);
        Quaternion4 quat{std::cos(angle * 0.5), axis[0] * s, axis[1] * s,
                         axis[2] * s};
        Matrix3 rot = articulated::convert_to_matrix(quat);
        Array3 origin = out[0].pos;
        rotate_frame(origin, rot, InOut(object1.mat), InOut(object1.pos));
        rotate_frame(origin, transpose(rot), InOut(object2.mat),
                     InOut(object2.pos));
        int n = find_penetration(InOut(object1), InOut(object2),
                                 out.subspan(ncon), 1, margin);
        if (n != 0 && ncon + 1 <= static_cast<int>(MAX_PAIR_CONTACTS)) {
          bool distinct = true;
          for (int i = 0; i < ncon; i++) {
            if (norm(subtract(out[i].pos, out[ncon].pos)) <= tolerance) {
              distinct = false;
              break;
            }
          }
          if (distinct) {
            out[ncon].dist = out[0].dist;
            ncon += 1;
          }
        }
        object1.pos = first_frame.pos;
        object2.pos = second_frame.pos;
        object1.mat = first_frame.mat;
        object2.mat = second_frame.mat;
      }
    }
  }
  return static_cast<std::uint32_t>(ncon);
}

}  // namespace simon::model
