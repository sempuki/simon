// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/articulated_collision.hpp"

#include <algorithm>
#include <cmath>

namespace simon::model {

namespace {

using articulated::convert_to_matrix;
using articulated::cross;
using articulated::MINVAL;
using articulated::multiply;
using articulated::normalize3;

constexpr double MAXVAL = 1e10;  // mjMAXVAL.

// MuJoCo's three-vector arithmetic (engine_inline.h, engine_util_blas.c), in
// its order of operations.

auto dot(const Array3& a, const Array3& b) -> double {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

auto add(const Array3& a, const Array3& b) -> Array3 {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

auto subtract(const Array3& a, const Array3& b) -> Array3 {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

auto scale(const Array3& v, double s) -> Array3 {
  return {v[0] * s, v[1] * s, v[2] * s};
}

// a + s b (mji_addScl3).
auto add_scaled(const Array3& a, const Array3& b, double s) -> Array3 {
  return {a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]};
}

// v += w s (mji_addToScl3).
auto add_to_scaled(InOut<Array3> v, const Array3& w, double s) -> void {
  (*v)[0] += w[0] * s;
  (*v)[1] += w[1] * s;
  (*v)[2] += w[2] * s;
}

auto multiply_transposed(const Matrix3& m, const Array3& v) -> Array3 {
  return {m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
          m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
          m[2] * v[0] + m[5] * v[1] + m[8] * v[2]};
}

auto column(const Matrix3& m, int c) -> Array3 {
  return {m[c], m[c + 3], m[c + 6]};
}

auto clip(double x, double low, double high) -> double {
  return x < low ? low : (x > high ? high : x);
}

using Out1 = std::span<PreContact>;

//-- Planes, spheres and capsules (engine_collision_primitive.c) --------------

// A sphere of radius `radius` at `pos2` on the plane at `pos1`, its normal
// the third column of `mat1` (mjraw_PlaneSphere).
auto collide_plane_sphere(Out1 con, double margin, const Array3& pos1,
                          const Matrix3& mat1, const Array3& pos2,
                          double radius) -> std::uint32_t {
  con[0].normal = column(mat1, 2);
  Array3 tmp = subtract(pos2, pos1);
  double cdist = dot(tmp, con[0].normal);
  if (cdist > margin + radius) {
    return 0;
  }
  con[0].dist = cdist - radius;
  tmp = scale(con[0].normal, -con[0].dist / 2 - radius);
  con[0].pos = add(pos2, tmp);
  con[0].tangent = {};
  return 1;
}

// mjraw_SphereSphere.
auto collide_spheres(Out1 con, double margin, const Array3& pos1,
                     const Matrix3& mat1, double radius1, const Array3& pos2,
                     const Matrix3& mat2, double radius2) -> std::uint32_t {
  Array3 dif = subtract(pos1, pos2);
  double cdist_sqr = dot(dif, dif);
  double min_dist = margin + radius1 + radius2;
  if (cdist_sqr > min_dist * min_dist) {
    return 0;
  }
  con[0].dist = std::sqrt(cdist_sqr) - radius1 - radius2;
  con[0].normal = subtract(pos2, pos1);
  double len = normalize3(con[0].normal);
  if (len < MINVAL) {
    con[0].normal = cross(column(mat1, 2), column(mat2, 2));
    normalize3(con[0].normal);
  }
  con[0].pos = scale(con[0].normal, radius1 + con[0].dist / 2);
  con[0].pos = add(con[0].pos, pos1);
  con[0].tangent = {};
  return 1;
}

auto collide_plane_capsule(Out1 con, double margin, const GeomFrame& f1,
                           const GeomFrame& f2, const Array3& size2)
    -> std::uint32_t {
  Array3 axis = column(f2.mat, 2);
  Array3 segment{size2[1] * axis[0], size2[1] * axis[1], size2[1] * axis[2]};
  Array3 endpoint = add(f2.pos, segment);
  std::uint32_t n1 =
      collide_plane_sphere(con, margin, f1.pos, f1.mat, endpoint, size2[0]);
  endpoint = subtract(f2.pos, segment);
  std::uint32_t n2 = collide_plane_sphere(con.subspan(n1), margin, f1.pos,
                                          f1.mat, endpoint, size2[0]);
  if (n1 != 0) {
    con[0].tangent = axis;
  }
  if (n2 != 0) {
    con[n1].tangent = axis;
  }
  return n1 + n2;
}

auto collide_plane_cylinder(Out1 con, double margin, const GeomFrame& f1,
                            const GeomFrame& f2, const Array3& size2)
    -> std::uint32_t {
  Array3 normal = column(f1.mat, 2);
  Array3 axis = column(f2.mat, 2);
  double prjaxis = dot(normal, axis);
  if (prjaxis > 0) {
    axis = scale(axis, -1);
    prjaxis = -prjaxis;
  }
  Array3 vec = subtract(f2.pos, f1.pos);
  double dist0 = dot(vec, normal);
  vec = scale(axis, prjaxis);
  vec = subtract(vec, normal);
  double len_sqr = dot(vec, vec);
  if (len_sqr >= MINVAL * MINVAL) {
    double scl = size2[0] / std::sqrt(len_sqr);
    vec = {vec[0] * scl, vec[1] * scl, vec[2] * scl};
  } else {
    vec = {f2.mat[0] * size2[0], f2.mat[3] * size2[0], f2.mat[6] * size2[0]};
  }
  double prjvec = dot(vec, normal);
  axis = scale(axis, size2[1]);
  prjaxis *= size2[1];

  std::uint32_t cnt = 0;
  auto emit = [&](double dist, const Array3& pos) {
    con[cnt].dist = dist;
    con[cnt].pos = pos;
    add_to_scaled(InOut(con[cnt].pos), normal, -dist * 0.5);
    con[cnt].normal = normal;
    con[cnt].tangent = {};
    ++cnt;
  };
  if (dist0 + prjaxis + prjvec <= margin) {
    emit(dist0 + prjaxis + prjvec, add(add(f2.pos, vec), axis));
  } else {
    return 0;
  }
  if (dist0 - prjaxis + prjvec <= margin) {
    emit(dist0 - prjaxis + prjvec, subtract(add(f2.pos, vec), axis));
  }
  double prjvec1 = -prjvec * 0.5;
  if (dist0 + prjaxis + prjvec1 <= margin) {
    Array3 vec1 = cross(vec, axis);
    normalize3(vec1);
    vec1 = scale(vec1, size2[0] * std::sqrt(3.0) / 2);
    Array3 a = add(add(f2.pos, vec1), axis);
    add_to_scaled(InOut(a), vec, -0.5);
    emit(dist0 + prjaxis + prjvec1, a);
    Array3 b = add(subtract(f2.pos, vec1), axis);
    add_to_scaled(InOut(b), vec, -0.5);
    emit(dist0 + prjaxis + prjvec1, b);
  }
  return cnt;
}

auto collide_plane_box(Out1 con, double margin, const GeomFrame& f1,
                       const GeomFrame& f2, const Array3& size2)
    -> std::uint32_t {
  Array3 norm = column(f1.mat, 2);
  Array3 dif = subtract(f2.pos, f1.pos);
  double dist = dot(dif, norm);
  std::uint32_t cnt = 0;
  for (int i = 0; i < 8; ++i) {
    Array3 vec{(i & 1) != 0 ? size2[0] : -size2[0],
               (i & 2) != 0 ? size2[1] : -size2[1],
               (i & 4) != 0 ? size2[2] : -size2[2]};
    Array3 corner = multiply(f2.mat, vec);
    double ldist = dot(norm, corner);
    if (dist + ldist > margin || ldist > 0) {
      continue;
    }
    con[cnt].dist = dist + ldist;
    con[cnt].normal = norm;
    corner = add(corner, f2.pos);
    vec = scale(norm, -con[cnt].dist / 2);
    con[cnt].pos = add(corner, vec);
    con[cnt].tangent = {};
    if (++cnt >= 4) {
      return 4;
    }
  }
  return cnt;
}

// mjraw_SphereCapsule.
auto collide_sphere_capsule(Out1 con, double margin, const Array3& pos1,
                            const Matrix3& mat1, double radius1,
                            const Array3& pos2, const Matrix3& mat2,
                            const Array3& size2) -> std::uint32_t {
  double len = size2[1];
  Array3 axis = column(mat2, 2);
  Array3 vec = subtract(pos1, pos2);
  double x = clip(dot(axis, vec), -len, len);
  vec = add(scale(axis, x), pos2);
  return collide_spheres(con, margin, pos1, mat1, radius1, vec, mat2, size2[0]);
}

auto collide_sphere_cylinder(Out1 con, double margin, const GeomFrame& f1,
                             const Array3& size1, const GeomFrame& f2,
                             const Array3& size2) -> std::uint32_t {
  double radius = size2[0];
  double height = size2[1];
  Array3 axis = column(f2.mat, 2);
  Array3 vec = subtract(f1.pos, f2.pos);
  double x = dot(axis, vec);
  Array3 a_proj = scale(axis, x);
  Array3 p_proj = subtract(vec, a_proj);
  double p_proj_sqr = dot(p_proj, p_proj);

  bool collide_side = std::abs(x) < height;
  bool collide_cap = p_proj_sqr < radius * radius;
  if (collide_side && collide_cap) {
    double dist_cap = height - std::abs(x);
    double dist_radius = radius - std::sqrt(p_proj_sqr);
    if (dist_cap < dist_radius) {
      collide_side = false;
    } else {
      collide_cap = false;
    }
  }
  if (collide_side) {
    a_proj = add(a_proj, f2.pos);
    return collide_spheres(con, margin, f1.pos, f1.mat, size1[0], a_proj,
                           f2.mat, size2[0]);
  }
  if (collide_cap) {
    const Matrix3& m = f2.mat;
    Matrix3 flip{-m[0], m[1], -m[2], -m[3], m[4], -m[5], -m[6], m[7], -m[8]};
    Array3 pos_cap = add_scaled(f2.pos, axis, x > 0 ? height : -height);
    std::uint32_t n = collide_plane_sphere(con, margin, pos_cap,
                                           x > 0 ? m : flip, f1.pos, size1[0]);
    if (n != 0) {
      con[0].normal = scale(con[0].normal, -1);
    }
    return n;
  }
  p_proj = scale(p_proj, size2[0] / std::sqrt(p_proj_sqr));
  vec = scale(axis, x > 0 ? height : -height);
  vec = add(vec, p_proj);
  vec = add(vec, f2.pos);
  return collide_spheres(con, margin, f1.pos, f1.mat, size1[0], vec, f2.mat,
                         0.0);
}

// mjraw_CapsuleCapsule.
auto collide_capsules(Out1 con, double margin, const GeomFrame& f1,
                      const Array3& size1, const GeomFrame& f2,
                      const Array3& size2) -> std::uint32_t {
  const Matrix3& mat1 = f1.mat;
  const Matrix3& mat2 = f2.mat;
  Array3 axis1{mat1[2] * size1[1], mat1[5] * size1[1], mat1[8] * size1[1]};
  Array3 axis2{mat2[2] * size2[1], mat2[5] * size2[1], mat2[8] * size2[1]};
  Array3 dif = subtract(f1.pos, f2.pos);
  double ma = dot(axis1, axis1);
  double mb = -dot(axis1, axis2);
  double mc = dot(axis2, axis2);
  double u = -dot(axis1, dif);
  double v = dot(axis2, dif);
  double det = ma * mc - mb * mb;
  auto spheres = [&](Out1 out, const Array3& vec1, const Array3& vec2) {
    return collide_spheres(out, margin, vec1, mat1, size1[0], vec2, mat2,
                           size2[0]);
  };
  if (std::abs(det) >= MINVAL) {
    double x1 = (mc * u - mb * v) / det;
    double x2 = (ma * v - mb * u) / det;
    if (x1 > 1) {
      x1 = 1;
      x2 = (v - mb) / mc;
    } else if (x1 < -1) {
      x1 = -1;
      x2 = (v + mb) / mc;
    }
    if (x2 > 1) {
      x2 = 1;
      x1 = clip((u - mb) / ma, -1, 1);
    } else if (x2 < -1) {
      x2 = -1;
      x1 = clip((u + mb) / ma, -1, 1);
    }
    return spheres(con, add(scale(axis1, x1), f1.pos),
                   add(scale(axis2, x2), f2.pos));
  }
  Array3 vec1 = add(f1.pos, axis1);
  double x2 = clip((v - mb) / mc, -1, 1);
  std::uint32_t n1 = spheres(con, vec1, add(scale(axis2, x2), f2.pos));
  vec1 = subtract(f1.pos, axis1);
  x2 = clip((v + mb) / mc, -1, 1);
  std::uint32_t n2 =
      spheres(con.subspan(n1), vec1, add(scale(axis2, x2), f2.pos));
  if (n1 + n2 >= 2) {
    return n1 + n2;
  }
  Array3 vec2 = add(f2.pos, axis2);
  double x1 = clip((u - mb) / ma, -1, 1);
  std::uint32_t n3 =
      spheres(con.subspan(n1 + n2), add(scale(axis1, x1), f1.pos), vec2);
  if (n1 + n2 + n3 >= 2) {
    return n1 + n2 + n3;
  }
  vec2 = subtract(f2.pos, axis2);
  x1 = clip((u + mb) / ma, -1, 1);
  std::uint32_t n4 =
      spheres(con.subspan(n1 + n2 + n3), add(scale(axis1, x1), f1.pos), vec2);
  return n1 + n2 + n3 + n4;
}

//-- Boxes (engine_collision_box.c) -------------------------------------------

// mjraw_SphereBox.
auto collide_sphere_box(Out1 con, double margin, const Array3& pos1,
                        double radius1, const Array3& pos2, const Matrix3& mat2,
                        const Array3& size2) -> std::uint32_t {
  Array3 tmp = subtract(pos1, pos2);
  Array3 center = multiply_transposed(mat2, tmp);
  Array3 clamped = center;
  for (int i = 0; i < 3; ++i) {
    if (size2[i] > 0) {
      clamped[i] = clip(clamped[i], -size2[i], size2[i]);
    }
  }
  Array3 deepest = center;
  tmp = subtract(clamped, center);
  double dist = normalize3(tmp);
  if (dist - radius1 > margin) {
    return 0;
  }
  Array3 pos{};
  if (dist <= MINVAL) {
    double closest = (size2[0] + size2[1] + size2[2]) * 2;
    int k = 0;
    for (int i = 0; i < 6; ++i) {
      double face =
          std::abs((i % 2 != 0 ? 1 : -1) * size2[i / 2] - center[i / 2]);
      if (closest > face) {
        closest = face;
        k = i;
      }
    }
    Array3 nearest{};
    nearest[k / 2] = k % 2 != 0 ? -1 : 1;
    pos = center;
    add_to_scaled(InOut(pos), nearest, (radius1 - closest) / 2);
    con[0].normal = multiply(mat2, nearest);
    dist = -closest;
  } else {
    add_to_scaled(InOut(deepest), tmp, radius1);
    add_to_scaled(InOut(pos), clamped, 0.5);
    add_to_scaled(InOut(pos), deepest, 0.5);
    con[0].normal = multiply(mat2, tmp);
  }
  tmp = multiply(mat2, pos);
  con[0].pos = add(tmp, pos2);
  con[0].dist = dist - radius1;
  con[0].tangent = {};
  return 1;
}

// mjraw_CapsuleBox: the point of the capsule's segment nearest the box, by
// its ends against the faces and the segment against the edges, then a
// second point along the segment when it lies within 45 degrees of a face
// or an edge, each a sphere against the box.
auto collide_capsule_box(Out1 con, double margin, const GeomFrame& f1,
                         const Array3& size1, const GeomFrame& f2,
                         const Array3& size2) -> std::uint32_t {
  double halflength = size1[1];
  double secondpos = -4;
  Array3 pos = multiply_transposed(f2.mat, subtract(f1.pos, f2.pos));
  Array3 axis = multiply_transposed(f2.mat, column(f1.mat, 2));
  Array3 halfaxis = scale(axis, halflength);

  int axisdir = 0;
  if (halfaxis[0] > 0) {
    axisdir += 1;
  }
  if (halfaxis[1] > 0) {
    axisdir += 2;
  }
  if (halfaxis[2] > 0) {
    axisdir += 4;
  }

  double bestdistmax =
      margin + 2 * (size1[0] + halflength + size2[0] + size2[1] + size2[2]);
  double bestdist = bestdistmax;
  double bestsegmentpos = 0;
  double bestboxpos = 0;
  int cltype = -4;
  int clface = 0;
  int clcorner = 0;
  int cledge = 0;

  // An end of the capsule nearest a face.
  for (int i = -1; i <= 1; i += 2) {
    Array3 tmp1 = pos;
    add_to_scaled(InOut(tmp1), halfaxis, i);
    Array3 tmp2 = tmp1;
    int c1 = 0;
    int c2 = -1;
    for (int j = 0; j < 3; ++j) {
      if (tmp1[j] < -size2[j]) {
        ++c1;
        c2 = j;
        tmp1[j] = -size2[j];
      } else if (tmp1[j] > size2[j]) {
        ++c1;
        c2 = j;
        tmp1[j] = size2[j];
      }
    }
    if (c1 > 1) {
      continue;
    }
    tmp1 = subtract(tmp1, tmp2);
    double dist = dot(tmp1, tmp1);
    if (dist < bestdist) {
      bestdist = dist;
      bestsegmentpos = i;
      cltype = -2 + i;
      clface = c2;
    }
  }

  // The segment nearest an edge.
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 8; ++i) {
      if ((i & (1 << j)) != 0) {
        continue;
      }
      Array3 tmp3{((i & 1) != 0 ? 1 : -1) * size2[0],
                  ((i & 2) != 0 ? 1 : -1) * size2[1],
                  ((i & 4) != 0 ? 1 : -1) * size2[2]};
      tmp3[j] = 0;
      Array3 dif = subtract(tmp3, pos);
      double ma = size2[j] * size2[j];
      double mb = -size2[j] * halfaxis[j];
      double mc = size1[1] * size1[1];
      double u = -size2[j] * dif[j];
      double v = dot(halfaxis, dif);
      double det = ma * mc - mb * mb;
      if (std::abs(det) < MINVAL) {
        continue;
      }
      double idet = 1 / det;
      double x1 = (mc * u - mb * v) * idet;
      double x2 = (ma * v - mb * u) * idet;
      int s1 = 1;
      int s2 = 1;
      if (x1 > 1) {
        x1 = 1;
        s1 = 2;
        x2 = (v - mb) * (1 / mc);
      } else if (x1 < -1) {
        x1 = -1;
        s1 = 0;
        x2 = (v + mb) * (1 / mc);
      }
      if (x2 > 1) {
        x2 = 1;
        s2 = 2;
        x1 = (u - mb) * (1 / ma);
        if (x1 > 1) {
          x1 = 1;
          s1 = 2;
        } else if (x1 < -1) {
          x1 = -1;
          s1 = 0;
        }
      } else if (x2 < -1) {
        x2 = -1;
        s2 = 0;
        x1 = (u + mb) * (1 / ma);
        if (x1 > 1) {
          x1 = 1;
          s1 = 2;
        } else if (x1 < -1) {
          x1 = -1;
          s1 = 0;
        }
      }
      dif = subtract(tmp3, pos);
      add_to_scaled(InOut(dif), halfaxis, -x2);
      dif[j] += size2[j] * x1;
      double d2 = dot(dif, dif);
      int c1 = s1 * 3 + s2;
      if (d2 < bestdist - MINVAL) {
        bestdist = d2;
        bestsegmentpos = x2;
        bestboxpos = x1;
        clcorner = i + (1 << j) * (c1 / 6);
        cledge = j;
        cltype = c1;
      }
    }
  }

  if (cltype == -4) {
    return 0;
  }

  double mul = 0;
  double e1 = 0;
  int ax = 0;
  int ax1 = 0;
  int ax2 = 0;
  if (cltype >= 0 && cltype / 3 != 1) {
    // Nearest a corner of the box.
    int c1 = axisdir ^ clcorner;
    double de = 0;
    double dp = 0;
    // Pointing at or away from the corner leaves one contact.
    if (c1 != 0 && c1 != 7) {
      if (c1 == 1 || c1 == 2 || c1 == 4) {
        mul = 1;
        de = 1 - bestsegmentpos;
        dp = 1 + bestsegmentpos;
      }
      if (c1 == 3 || c1 == 5 || c1 == 6) {
        mul = -1;
        c1 = 7 - c1;
        dp = 1 - bestsegmentpos;
        de = 1 + bestsegmentpos;
      }
      if (c1 == 1) {
        ax = 0, ax1 = 1, ax2 = 2;
      }
      if (c1 == 2) {
        ax = 1, ax1 = 2, ax2 = 0;
      }
      if (c1 == 4) {
        ax = 2, ax1 = 0, ax2 = 1;
      }
      if (axis[ax] * axis[ax] > 0.5) {
        secondpos = de;
        e1 = 2 * size2[ax] / std::abs(halfaxis[ax]);
        if (e1 < secondpos) {
          secondpos = e1;
        }
        secondpos *= mul;
      } else {
        secondpos = dp;
        e1 = 2 * size2[ax1] / std::abs(halfaxis[ax1]);
        if (e1 < secondpos) {
          secondpos = e1;
        }
        e1 = 2 * size2[ax2] / std::abs(halfaxis[ax2]);
        if (e1 < secondpos) {
          secondpos = e1;
        }
        secondpos *= -mul;
      }
    }
  } else if (cltype >= 0 && cltype / 3 == 1) {
    // On an edge of the box: a T leaves one contact, an X two.
    int c1 = axisdir ^ clcorner;
    c1 &= 7 - (1 << cledge);
    if (c1 == 1 || c1 == 2 || c1 == 4) {
      if (cledge == 0) {
        ax1 = 1, ax2 = 2;
      }
      if (cledge == 1) {
        ax1 = 2, ax2 = 0;
      }
      if (cledge == 2) {
        ax1 = 0, ax2 = 1;
      }
      ax = cledge;
      if (std::abs(axis[ax1]) > std::abs(axis[ax2])) {
        ax1 = ax2;
      }
      ax2 = 3 - ax - ax1;
      if ((c1 & (1 << ax2)) != 0) {
        mul = 1;
        secondpos = 1 - bestsegmentpos;
      } else {
        mul = -1;
        secondpos = 1 + bestsegmentpos;
      }
      e1 = 2 * size2[ax2] / std::abs(halfaxis[ax2]);
      if (e1 < secondpos) {
        secondpos = e1;
      }
      double e2 = ((axisdir & (1 << ax)) != 0) == ((c1 & (1 << ax2)) != 0)
                      ? 1 - bestboxpos
                      : 1 + bestboxpos;
      e1 = size2[ax] * e2 / std::abs(halfaxis[ax]);
      if (e1 < secondpos) {
        secondpos = e1;
      }
      secondpos *= mul;
    }
  } else if (cltype < 0) {
    // An end nearest a face, outside the box: the farthest point of the
    // segment over that face.
    if (clface != -1) {
      mul = cltype == -3 ? 1 : -1;
      secondpos = 2;
      Array3 tmp1 = pos;
      add_to_scaled(InOut(tmp1), halfaxis, -mul);
      for (int i = 0; i < 3; ++i) {
        if (i != clface) {
          e1 = (size2[i] - tmp1[i]) / halfaxis[i] * mul;
          if (e1 > 0 && e1 < secondpos) {
            secondpos = e1;
          }
          e1 = (-size2[i] - tmp1[i]) / halfaxis[i] * mul;
          if (e1 > 0 && e1 < secondpos) {
            secondpos = e1;
          }
        }
      }
      secondpos *= mul;
    }
  }

  Array3 tmp1 = pos;
  add_to_scaled(InOut(tmp1), halfaxis, bestsegmentpos);
  Array3 tmp2 = add(multiply(f2.mat, tmp1), f2.pos);
  std::uint32_t n =
      collide_sphere_box(con, margin, tmp2, size1[0], f2.pos, f2.mat, size2);
  if (secondpos > -3) {
    tmp1 = pos;
    add_to_scaled(InOut(tmp1), halfaxis, secondpos + bestsegmentpos);
    tmp2 = add(multiply(f2.mat, tmp1), f2.pos);
    n += collide_sphere_box(con.subspan(n), margin, tmp2, size1[0], f2.pos,
                            f2.mat, size2);
  }
  return n;
}

// Box-box rounding scales and limits (engine_collision_box.c).
constexpr double SEPEPS = 1e-13;
constexpr double PAREPS = 1e-16;
constexpr double SGNEPS = 1e-9;
constexpr double DUPEPS = 1e-14;
constexpr double EDGEBIAS = 1e-6;
constexpr int MAXVERT = 12;

using Polygon = std::array<Array3, MAXVERT>;

// Clips polygon `*cur` of `nin` vertices to sign v[coord] <= limit,
// swapping the buffers only when it clips (clipHalfPlane).
auto clip_half_plane(int nin, InOut<Polygon*> cur, InOut<Polygon*> spare,
                     int coord, double sign, double limit) -> int {
  Polygon& in = **cur;
  std::array<double, MAXVERT> d{};
  bool all_inside = true;
  for (int k = 0; k < nin; ++k) {
    d[k] = sign * in[k][coord] - limit;
    all_inside = all_inside && d[k] <= 0;
  }
  if (all_inside) {
    return nin;
  }
  Polygon& out = **spare;
  int nout = 0;
  for (int k = 0; k < nin; ++k) {
    const Array3& p = in[k];
    int k1 = k + 1 == nin ? 0 : k + 1;
    double dp = d[k];
    double dq = d[k1];
    if (dp <= 0 && nout < MAXVERT) {
      out[nout++] = p;
    }
    if (((dp < 0 && dq > 0) || (dp > 0 && dq < 0)) && nout < MAXVERT) {
      const Array3& q = in[k1];
      double t = dp / (dp - dq);
      out[nout] = {p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]),
                   p[2] + t * (q[2] - p[2])};
      ++nout;
    }
  }
  std::swap(*cur, *spare);
  return nout;
}

// mjc_BoxBox: the axis of greatest separation among the 15 a separating
// axis test tries, faces preferred on near-ties; then for an edge axis one
// contact between the nearest points of the two edges, and for a face axis
// the other box's face clipped to it (Sutherland–Hodgman), a contact at
// each vertex within the margin.
auto collide_boxes(Out1 con, double margin, const GeomFrame& f1,
                   const Array3& size1, const GeomFrame& f2,
                   const Array3& size2) -> std::uint32_t {
  const Array3& pos1 = f1.pos;
  const Array3& pos2 = f2.pos;
  const Matrix3& mat1 = f1.mat;
  const Matrix3& mat2 = f2.mat;
  Array3 pos21 = multiply_transposed(mat1, subtract(pos2, pos1));
  Array3 pos12 = multiply_transposed(mat2, subtract(pos1, pos2));
  Matrix3 rot{mat1[0] * mat2[0] + mat1[3] * mat2[3] + mat1[6] * mat2[6],
              mat1[0] * mat2[1] + mat1[3] * mat2[4] + mat1[6] * mat2[7],
              mat1[0] * mat2[2] + mat1[3] * mat2[5] + mat1[6] * mat2[8],
              mat1[1] * mat2[0] + mat1[4] * mat2[3] + mat1[7] * mat2[6],
              mat1[1] * mat2[1] + mat1[4] * mat2[4] + mat1[7] * mat2[7],
              mat1[1] * mat2[2] + mat1[4] * mat2[5] + mat1[7] * mat2[8],
              mat1[2] * mat2[0] + mat1[5] * mat2[3] + mat1[8] * mat2[6],
              mat1[2] * mat2[1] + mat1[5] * mat2[4] + mat1[8] * mat2[7],
              mat1[2] * mat2[2] + mat1[5] * mat2[5] + mat1[8] * mat2[8]};
  Matrix3 rotabs{};
  for (int i = 0; i < 9; ++i) {
    rotabs[i] = std::abs(rot[i]);
  }

  double septol = margin + SEPEPS * (size1[0] + size1[1] + size1[2] + size2[0] +
                                     size2[1] + size2[2]);
  double sep_best = -MAXVAL;
  int code = -1;
  for (int i = 0; i < 3; ++i) {
    double radius2 = rotabs[3 * i + 0] * size2[0] +
                     rotabs[3 * i + 1] * size2[1] +
                     rotabs[3 * i + 2] * size2[2];
    double sep = std::abs(pos21[i]) - size1[i] - radius2;
    if (sep > septol) {
      return 0;
    }
    if (sep > sep_best) {
      sep_best = sep;
      code = i;
    }
  }
  for (int j = 0; j < 3; ++j) {
    double radius1 = rotabs[0 + j] * size1[0] + rotabs[3 + j] * size1[1] +
                     rotabs[6 + j] * size1[2];
    double sep = std::abs(pos12[j]) - size2[j] - radius1;
    if (sep > septol) {
      return 0;
    }
    if (sep > sep_best) {
      sep_best = sep;
      code = 3 + j;
    }
  }
  double sep_face = sep_best;
  int code_face = code;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      int i1 = (i + 1) % 3;
      int i2 = (i + 2) % 3;
      double ax1 = -rot[3 * i2 + j];
      double ax2 = rot[3 * i1 + j];
      double norm2 = ax1 * ax1 + ax2 * ax2;
      if (norm2 < PAREPS) {
        continue;
      }
      double inv = 1 / std::sqrt(norm2);
      ax1 *= inv;
      ax2 *= inv;
      double radius1 = size1[i1] * std::abs(ax1) + size1[i2] * std::abs(ax2);
      int j1 = (j + 1) % 3;
      int j2 = (j + 2) % 3;
      double a2_1 = ax1 * rot[3 * i1 + j1] + ax2 * rot[3 * i2 + j1];
      double a2_2 = ax1 * rot[3 * i1 + j2] + ax2 * rot[3 * i2 + j2];
      double radius2 = size2[j1] * std::abs(a2_1) + size2[j2] * std::abs(a2_2);
      double sep =
          std::abs(ax1 * pos21[i1] + ax2 * pos21[i2]) - radius1 - radius2;
      if (sep > septol) {
        return 0;
      }
      if (sep - EDGEBIAS * std::abs(sep) > sep_best && sep > sep_face) {
        sep_best = sep;
        code = 6 + 3 * i + j;
      }
    }
  }
  if (code < 0) {
    return 0;
  }

  auto edge_axis = [&](int i, int j) {
    int i1 = (i + 1) % 3;
    int i2 = (i + 2) % 3;
    Array3 axis{};
    axis[i] = 0;
    axis[i1] = -rot[3 * i2 + j];
    axis[i2] = rot[3 * i1 + j];
    normalize3(axis);
    return axis;
  };

  // An edge axis nearly along the best face's yields to the face.
  if (code >= 6) {
    Array3 axis = edge_axis((code - 6) / 3, (code - 6) % 3);
    double face_dot = 0;
    if (code_face < 3) {
      face_dot = std::abs(axis[code_face]);
    } else {
      int f = code_face - 3;
      face_dot = std::abs(axis[0] * rot[0 + f] + axis[1] * rot[3 + f] +
                          axis[2] * rot[6 + f]);
    }
    if (face_dot > 0.99 &&
        sep_best < sep_face + 0.05 * std::abs(sep_face) + MINVAL) {
      code = code_face;
      sep_best = sep_face;
    }
  }

  if (code >= 6) {
    int i = (code - 6) / 3;
    int j = (code - 6) % 3;
    int i1 = (i + 1) % 3;
    int i2 = (i + 2) % 3;
    int j1 = (j + 1) % 3;
    int j2 = (j + 2) % 3;
    Array3 axis = edge_axis(i, j);
    if (dot(axis, pos21) < 0) {
      axis = {-axis[0], -axis[1], -axis[2]};
    }
    Array3 a2{
        axis[0] * rot[0 + 0] + axis[1] * rot[3 + 0] + axis[2] * rot[6 + 0],
        axis[0] * rot[0 + 1] + axis[1] * rot[3 + 1] + axis[2] * rot[6 + 1],
        axis[0] * rot[0 + 2] + axis[1] * rot[3 + 2] + axis[2] * rot[6 + 2]};
    int amb1 = -1;
    int amb2 = -1;
    if (std::abs(axis[i1]) < SGNEPS) {
      amb1 = i1;
    } else if (std::abs(axis[i2]) < SGNEPS) {
      amb1 = i2;
    }
    if (std::abs(a2[j1]) < SGNEPS) {
      amb2 = j1;
    } else if (std::abs(a2[j2]) < SGNEPS) {
      amb2 = j2;
    }
    Array3 d2{rot[0 + j], rot[3 + j], rot[6 + j]};
    double b = d2[i];
    double denom = 1 - b * b;
    Array3 w1{};
    Array3 w2{};
    double best_d2 = MAXVAL;
    for (int v1 = 0; v1 < (amb1 >= 0 ? 2 : 1); ++v1) {
      for (int v2 = 0; v2 < (amb2 >= 0 ? 2 : 1); ++v2) {
        Array3 c1{};
        c1[i] = 0;
        c1[i1] = axis[i1] >= 0 ? size1[i1] : -size1[i1];
        c1[i2] = axis[i2] >= 0 ? size1[i2] : -size1[i2];
        if (amb1 >= 0 && v1 != 0) {
          c1[amb1] = -c1[amb1];
        }
        Array3 cc{};
        cc[j] = 0;
        cc[j1] = a2[j1] >= 0 ? -size2[j1] : size2[j1];
        cc[j2] = a2[j2] >= 0 ? -size2[j2] : size2[j2];
        if (amb2 >= 0 && v2 != 0) {
          cc[amb2] = -cc[amb2];
        }
        Array3 c2 = add(multiply(rot, cc), pos21);
        Array3 e = subtract(c2, c1);
        double d1e = e[i];
        double d2e = dot(d2, e);
        double s = denom < MINVAL ? 0 : (d1e - b * d2e) / denom;
        s = clip(s, -size1[i], size1[i]);
        double t = clip(b * s - d2e, -size2[j], size2[j]);
        s = clip(d1e + b * t, -size1[i], size1[i]);
        Array3 p1 = c1;
        p1[i] += s;
        Array3 p2 = c2;
        add_to_scaled(InOut(p2), d2, t);
        Array3 gap = subtract(p2, p1);
        double gap2 = dot(gap, gap);
        if (gap2 < best_d2) {
          best_d2 = gap2;
          w1 = p1;
          w2 = p2;
        }
      }
    }
    double dist = dot(subtract(w2, w1), axis);
    if (dist > septol) {
      return 0;
    }
    Array3 mid{0.5 * (w1[0] + w2[0]), 0.5 * (w1[1] + w2[1]),
               0.5 * (w1[2] + w2[2])};
    con[0].dist = dist;
    con[0].pos = add(multiply(mat1, mid), pos1);
    con[0].normal = multiply(mat1, axis);
    con[0].tangent = {};
    return 1;
  }

  // A face: the incident face clipped to the reference face.
  bool ref1 = code < 3;
  int a = ref1 ? code : code - 3;
  const Array3& sizeref = ref1 ? size1 : size2;
  const Array3& sizeinc = ref1 ? size2 : size1;
  const Array3& posref = ref1 ? pos1 : pos2;
  const Matrix3& matref = ref1 ? mat1 : mat2;
  const Array3& posoi = ref1 ? pos21 : pos12;
  Matrix3 rinc = rot;
  if (!ref1) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        rinc[3 * r + c] = rot[3 * c + r];
      }
    }
  }
  double sgn = posoi[a] >= 0 ? 1 : -1;
  int binc = 0;
  for (int k = 1; k < 3; ++k) {
    if (std::abs(rinc[3 * a + k]) > std::abs(rinc[3 * a + binc])) {
      binc = k;
    }
  }
  double tinc = sgn * rinc[3 * a + binc] > 0 ? -1 : 1;
  int ax = (a + 1) % 3;
  int ay = (a + 2) % 3;
  int bu = (binc + 1) % 3;
  int bv = (binc + 2) % 3;
  std::array<Polygon, 2> poly{};
  Array3 cx{};
  Array3 du{};
  Array3 dv{};
  for (int r = 0; r < 3; ++r) {
    int c = r == 0 ? ax : (r == 1 ? ay : a);
    cx[r] = posoi[c] + tinc * sizeinc[binc] * rinc[3 * c + binc];
    du[r] = sizeinc[bu] * rinc[3 * c + bu];
    dv[r] = sizeinc[bv] * rinc[3 * c + bv];
  }
  cx[2] = sgn * cx[2] - sizeref[a];
  du[2] *= sgn;
  dv[2] *= sgn;
  constexpr std::array<std::array<double, 2>, 4> CORNER_SIGN{
      {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}}};
  for (int k = 0; k < 4; ++k) {
    double su = CORNER_SIGN[k][0];
    double sv = CORNER_SIGN[k][1];
    for (int r = 0; r < 3; ++r) {
      poly[0][k][r] = cx[r] + su * du[r] + sv * dv[r];
    }
  }
  int nvert = 4;
  Polygon* cur = poly.data();
  Polygon* spare = poly.data() + 1;
  nvert = clip_half_plane(nvert, InOut(cur), InOut(spare), 0, 1, sizeref[ax]);
  nvert = clip_half_plane(nvert, InOut(cur), InOut(spare), 0, -1, sizeref[ax]);
  nvert = clip_half_plane(nvert, InOut(cur), InOut(spare), 1, 1, sizeref[ay]);
  nvert = clip_half_plane(nvert, InOut(cur), InOut(spare), 1, -1, sizeref[ay]);

  Polygon accepted{};
  int naccept = 0;
  double dupe2 =
      DUPEPS * (sizeref[ax] * sizeref[ax] + sizeref[ay] * sizeref[ay]);
  for (int k = 0; k < nvert; ++k) {
    const Array3& vertex = (*cur)[k];
    if (vertex[2] > margin) {
      continue;
    }
    bool dupe = false;
    for (int q = 0; q < naccept; ++q) {
      double dx = accepted[q][0] - vertex[0];
      double dy = accepted[q][1] - vertex[1];
      if (dx * dx + dy * dy < dupe2) {
        dupe = true;
        break;
      }
    }
    if (!dupe) {
      accepted[naccept++] = vertex;
    }
  }
  if (naccept == 0) {
    return 0;
  }
  double nsign = ref1 ? sgn : -sgn;
  Array3 normal{nsign * matref[3 * 0 + a], nsign * matref[3 * 1 + a],
                nsign * matref[3 * 2 + a]};
  for (int k = 0; k < naccept; ++k) {
    const Array3& v = accepted[k];
    Array3 posc{};
    posc[ax] = v[0];
    posc[ay] = v[1];
    posc[a] = sgn * (sizeref[a] + 0.5 * v[2]);
    con[k].dist = v[2];
    con[k].pos = add(multiply(matref, posc), posref);
    con[k].normal = normal;
    con[k].tangent = {};
  }
  return static_cast<std::uint32_t>(naccept);
}

// A contact's frame from its normal and tangent (mju_makeFrame).
auto make_frame(const Array3& normal, const Array3& tangent) -> Matrix3 {
  Array3 x = normal;
  normalize3(x);
  Array3 y = tangent;
  if (dot(y, y) < 0.25) {
    y = {};
    if (x[1] < 0.5 && x[1] > -0.5) {
      y[1] = 1;
    } else {
      y[2] = 1;
    }
  }
  Array3 tmp = scale(x, dot(x, y));
  y = subtract(y, tmp);
  normalize3(y);
  Array3 z = cross(x, y);
  return {x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]};
}

}  // namespace

auto has_collider(GeomType first, GeomType second) -> bool {
  using enum GeomType;
  switch (first) {
    case PLANE:
      return second == SPHERE || second == CAPSULE || second == CYLINDER ||
             second == BOX;
    case SPHERE:
      return second == SPHERE || second == CAPSULE || second == CYLINDER ||
             second == BOX;
    case CAPSULE:
      return second == CAPSULE || second == BOX;
    case BOX:
      return second == BOX;
    default:
      return false;
  }
}

auto compute_bounding_radius(const Geom& geom) -> double {
  const Array3& s = geom.size;
  switch (geom.type) {
    case GeomType::SPHERE:
      return s[0];
    case GeomType::CAPSULE:
      return s[0] + s[1];
    case GeomType::CYLINDER:
      return std::sqrt(s[0] * s[0] + s[1] * s[1]);
    case GeomType::ELLIPSOID:
      return std::max(std::max(s[0], s[1]), s[2]);
    case GeomType::BOX:
      return std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    default:
      return 0.0;
  }
}

auto compute_geom_frame(const Geom& geom, const BodyFrame& body) -> GeomFrame {
  GeomFrame frame;
  switch (geom.frame) {
    case SameFrame::BODY:
      frame.pos = body.xpos;
      break;
    case SameFrame::INERTIA:
      frame.pos = body.xipos;
      break;
    default:
      frame.pos = add(multiply(body.xmat, geom.pos), body.xpos);
      break;
  }
  switch (geom.frame) {
    case SameFrame::BODY:
    case SameFrame::BODY_ROTATION:
      frame.mat = body.xmat;
      break;
    case SameFrame::INERTIA:
    case SameFrame::INERTIA_ROTATION:
      frame.mat = body.ximat;
      break;
    default:
      frame.mat = convert_to_matrix(multiply(body.xquat, geom.quat));
      break;
  }
  return frame;
}

auto collide(const Geom& first, const GeomFrame& f1, const Geom& second,
             const GeomFrame& f2, double margin,
             std::span<PreContact, MAX_PAIR_CONTACTS> out) -> std::uint32_t {
  using enum GeomType;
  Out1 con{out};
  const Array3& size1 = first.size;
  const Array3& size2 = second.size;
  switch (first.type) {
    case PLANE:
      switch (second.type) {
        case SPHERE:
          return collide_plane_sphere(con, margin, f1.pos, f1.mat, f2.pos,
                                      size2[0]);
        case CAPSULE:
          return collide_plane_capsule(con, margin, f1, f2, size2);
        case CYLINDER:
          return collide_plane_cylinder(con, margin, f1, f2, size2);
        case BOX:
          return collide_plane_box(con, margin, f1, f2, size2);
        default:
          return 0;
      }
    case SPHERE:
      switch (second.type) {
        case SPHERE:
          return collide_spheres(con, margin, f1.pos, f1.mat, size1[0], f2.pos,
                                 f2.mat, size2[0]);
        case CAPSULE:
          return collide_sphere_capsule(con, margin, f1.pos, f1.mat, size1[0],
                                        f2.pos, f2.mat, size2);
        case CYLINDER:
          return collide_sphere_cylinder(con, margin, f1, size1, f2, size2);
        case BOX:
          return collide_sphere_box(con, margin, f1.pos, size1[0], f2.pos,
                                    f2.mat, size2);
        default:
          return 0;
      }
    case CAPSULE:
      switch (second.type) {
        case CAPSULE:
          return collide_capsules(con, margin, f1, size1, f2, size2);
        case BOX:
          return collide_capsule_box(con, margin, f1, size1, f2, size2);
        default:
          return 0;
      }
    case BOX:
      return second.type == BOX
                 ? collide_boxes(con, margin, f1, size1, f2, size2)
                 : 0;
    default:
      return 0;
  }
}

BodyFilter::BodyFilter(const ArticulatedModel& model) {
  std::size_t n = model.bodies.size();
  weld_.resize(n);
  weld_parent_.resize(n);
  weld_dofs_.resize(n);
  for (std::uint32_t b = 0; b < n; ++b) {
    const ArticulatedBody& body = model.bodies[b];
    weld_[b] = b == 0 || body.joints > 0 ? b : weld_[body.parent];
  }
  for (std::uint32_t b = 0; b < n; ++b) {
    weld_parent_[b] = weld_[model.bodies[weld_[b]].parent];
    weld_dofs_[b] = model.bodies[weld_[b]].dofs;
  }
}

auto BodyFilter::discards(std::uint32_t first, std::uint32_t second) const
    -> bool {
  std::uint32_t weld1 = weld_[first];
  std::uint32_t weld2 = weld_[second];
  if (weld1 == weld2) {
    return true;
  }
  if (weld_dofs_[first] == 0 && weld_dofs_[second] == 0) {
    return true;
  }
  return weld1 != 0 && weld2 != 0 &&
         (weld1 == weld_parent_[second] || weld2 == weld_parent_[first]);
}

auto append_contacts(const ArticulatedModel& model, std::uint32_t first,
                     std::uint32_t second, std::span<const GeomFrame> frames,
                     std::span<const double> radii,
                     InOut<std::vector<Contact>> out) -> void {
  const Geom* g1 = &model.geoms[first];
  const Geom* g2 = &model.geoms[second];
  if ((g1->contype & g2->conaffinity) == 0 &&
      (g2->contype & g1->conaffinity) == 0) {
    return;
  }
  double margin = g1->margin + g2->margin;
  double gap = g1->gap + g2->gap;
  double bound = margin + gap;
  // Bounding spheres, or a sphere against a plane (mj_filterSphere).
  const GeomFrame& p1 = frames[first];
  const GeomFrame& p2 = frames[second];
  double r1 = radii[first];
  double r2 = radii[second];
  if (r1 > 0 && r2 > 0) {
    Array3 dif = subtract(p1.pos, p2.pos);
    double reach = r1 + r2 + bound;
    if (dot(dif, dif) > reach * reach) {
      return;
    }
  } else {
    auto above = [&](const GeomFrame& plane, const GeomFrame& other) {
      return dot(subtract(other.pos, plane.pos), column(plane.mat, 2));
    };
    if (g1->type == GeomType::PLANE && r2 > 0 && above(p1, p2) > bound + r2) {
      return;
    }
    if (g2->type == GeomType::PLANE && r1 > 0 && above(p2, p1) > bound + r1) {
      return;
    }
  }
  if (g1->type > g2->type) {
    std::swap(first, second);
    std::swap(g1, g2);
  }
  std::array<PreContact, MAX_PAIR_CONTACTS> found{};
  std::uint32_t n =
      collide(*g1, frames[first], *g2, frames[second], bound, found);
  if (n == 0) {
    return;
  }

  // Parameters: the geom of higher priority's, else mixed (mj_contactParam;
  // solmix is 1 for both, so the mix is even).
  Contact contact;
  contact.geom = {first, second};
  contact.include_margin = margin;
  Array3 friction{};
  if (g1->priority != g2->priority) {
    const Geom& high = g1->priority > g2->priority ? *g1 : *g2;
    contact.dim = high.condim;
    contact.soft = high.contact;
    friction = high.friction;
  } else {
    contact.dim = std::max(g1->condim, g2->condim);
    double mix = 1.0 / (1.0 + 1.0);
    const SoftConstraint& s1 = g1->contact;
    const SoftConstraint& s2 = g2->contact;
    for (int i = 0; i < 2; ++i) {
      contact.soft.reference[i] =
          s1.reference[0] > 0 && s2.reference[0] > 0
              ? mix * s1.reference[i] + (1 - mix) * s2.reference[i]
              : std::min(s1.reference[i], s2.reference[i]);
    }
    for (int i = 0; i < 5; ++i) {
      contact.soft.impedance[i] =
          mix * s1.impedance[i] + (1 - mix) * s2.impedance[i];
    }
    for (int i = 0; i < 3; ++i) {
      friction[i] = std::max(g1->friction[i], g2->friction[i]);
    }
  }
  contact.friction = {friction[0], friction[0], friction[1], friction[2],
                      friction[2]};
  for (std::uint32_t k = 0; k < n; ++k) {
    contact.dist = found[k].dist;
    contact.pos = found[k].pos;
    contact.frame = make_frame(found[k].normal, found[k].tangent);
    contact.exclude = contact.dist >= contact.include_margin;
    out->push_back(contact);
  }
}

}  // namespace simon::model
