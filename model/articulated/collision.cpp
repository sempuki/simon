// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#include "model/articulated/collision.hpp"

#include <algorithm>
#include <cmath>

#include "model/articulated/convex.hpp"

namespace simon::articulated {

namespace {

constexpr double MAXVAL = 1e10;  // mjMAXVAL.

auto clip(double x, double low, double high) -> double {
  return x < low ? low : (x > high ? high : x);
}

using Out1 = std::span<PreContact>;

//-- Planes, spheres and capsules (engine_collision_primitive.c) --------------

// A sphere of radius `radius` at `pos2` on the plane at `pos1`, its normal
// the third column of `mat1` (mjraw_PlaneSphere).
auto collide_plane_sphere(Out1 con, double margin, const Vector3& pos1,
                          const Matrix3& mat1, const Vector3& pos2,
                          double radius) -> std::uint32_t {
  con[0].normal = mat1.col(2);
  Vector3 tmp = pos2 - pos1;
  double cdist = tmp.dot(con[0].normal);
  if (cdist > margin + radius) {
    return 0;
  }
  con[0].dist = cdist - radius;
  tmp = con[0].normal * (-con[0].dist / 2 - radius);
  con[0].pos = pos2 + tmp;
  con[0].tangent = Vector3::Zero();
  return 1;
}

// mjraw_SphereSphere.
auto collide_spheres(Out1 con, double margin, const Vector3& pos1,
                     const Matrix3& mat1, double radius1, const Vector3& pos2,
                     const Matrix3& mat2, double radius2) -> std::uint32_t {
  Vector3 dif = pos1 - pos2;
  double cdist_sqr = dif.dot(dif);
  double min_dist = margin + radius1 + radius2;
  if (cdist_sqr > min_dist * min_dist) {
    return 0;
  }
  con[0].dist = std::sqrt(cdist_sqr) - radius1 - radius2;
  con[0].normal = pos2 - pos1;
  double len = normalize(InOut(con[0].normal));
  if (len < MINVAL) {
    con[0].normal = mat1.col(2).cross(mat2.col(2));
    normalize(InOut(con[0].normal));
  }
  con[0].pos = con[0].normal * (radius1 + con[0].dist / 2);
  con[0].pos += pos1;
  con[0].tangent = Vector3::Zero();
  return 1;
}

auto collide_plane_capsule(Out1 con, double margin, const GeomFrame& f1,
                           const GeomFrame& f2, const Vector3& size2)
    -> std::uint32_t {
  Vector3 axis = f2.mat.col(2);
  Vector3 segment = size2[1] * axis;
  Vector3 endpoint = f2.pos + segment;
  std::uint32_t n1 =
      collide_plane_sphere(con, margin, f1.pos, f1.mat, endpoint, size2[0]);
  endpoint = f2.pos - segment;
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
                            const GeomFrame& f2, const Vector3& size2)
    -> std::uint32_t {
  Vector3 normal = f1.mat.col(2);
  Vector3 axis = f2.mat.col(2);
  double prjaxis = normal.dot(axis);
  if (prjaxis > 0) {
    axis = -axis;
    prjaxis = -prjaxis;
  }
  Vector3 vec = f2.pos - f1.pos;
  double dist0 = vec.dot(normal);
  vec = axis * prjaxis;
  vec -= normal;
  double len_sqr = vec.dot(vec);
  if (len_sqr >= MINVAL * MINVAL) {
    vec *= size2[0] / std::sqrt(len_sqr);
  } else {
    vec = f2.mat.col(0) * size2[0];
  }
  double prjvec = vec.dot(normal);
  axis = axis * size2[1];
  prjaxis *= size2[1];

  std::uint32_t cnt = 0;
  auto emit = [&](double dist, const Vector3& pos) {
    con[cnt].dist = dist;
    con[cnt].pos = pos;
    con[cnt].pos -= 0.5 * dist * normal;
    con[cnt].normal = normal;
    con[cnt].tangent = Vector3::Zero();
    ++cnt;
  };
  if (dist0 + prjaxis + prjvec <= margin) {
    emit(dist0 + prjaxis + prjvec, f2.pos + vec + axis);
  } else {
    return 0;
  }
  if (dist0 - prjaxis + prjvec <= margin) {
    emit(dist0 - prjaxis + prjvec, f2.pos + vec - axis);
  }
  double prjvec1 = -prjvec * 0.5;
  if (dist0 + prjaxis + prjvec1 <= margin) {
    Vector3 vec1 = vec.cross(axis);
    normalize(InOut(vec1));
    vec1 = vec1 * (size2[0] * std::sqrt(3.0) / 2);
    Vector3 a = f2.pos + vec1 + axis;
    a -= 0.5 * vec;
    emit(dist0 + prjaxis + prjvec1, a);
    Vector3 b = f2.pos - vec1 + axis;
    b -= 0.5 * vec;
    emit(dist0 + prjaxis + prjvec1, b);
  }
  return cnt;
}

auto collide_plane_box(Out1 con, double margin, const GeomFrame& f1,
                       const GeomFrame& f2, const Vector3& size2)
    -> std::uint32_t {
  Vector3 norm = f1.mat.col(2);
  Vector3 dif = f2.pos - f1.pos;
  double dist = dif.dot(norm);
  std::uint32_t cnt = 0;
  for (int i = 0; i < 8; ++i) {
    Vector3 vec{(i & 1) != 0 ? size2[0] : -size2[0],
                (i & 2) != 0 ? size2[1] : -size2[1],
                (i & 4) != 0 ? size2[2] : -size2[2]};
    Vector3 corner = f2.mat * vec;
    double ldist = norm.dot(corner);
    if (dist + ldist > margin || ldist > 0) {
      continue;
    }
    con[cnt].dist = dist + ldist;
    con[cnt].normal = norm;
    corner += f2.pos;
    vec = norm * (-con[cnt].dist / 2);
    con[cnt].pos = corner + vec;
    con[cnt].tangent = Vector3::Zero();
    if (++cnt >= 4) {
      return 4;
    }
  }
  return cnt;
}

// mjraw_SphereCapsule.
auto collide_sphere_capsule(Out1 con, double margin, const Vector3& pos1,
                            const Matrix3& mat1, double radius1,
                            const Vector3& pos2, const Matrix3& mat2,
                            const Vector3& size2) -> std::uint32_t {
  double len = size2[1];
  Vector3 axis = mat2.col(2);
  Vector3 vec = pos1 - pos2;
  double x = clip(axis.dot(vec), -len, len);
  vec = axis * x + pos2;
  return collide_spheres(con, margin, pos1, mat1, radius1, vec, mat2, size2[0]);
}

auto collide_sphere_cylinder(Out1 con, double margin, const GeomFrame& f1,
                             const Vector3& size1, const GeomFrame& f2,
                             const Vector3& size2) -> std::uint32_t {
  double radius = size2[0];
  double height = size2[1];
  Vector3 axis = f2.mat.col(2);
  Vector3 vec = f1.pos - f2.pos;
  double x = axis.dot(vec);
  Vector3 a_proj = axis * x;
  Vector3 p_proj = vec - a_proj;
  double p_proj_sqr = p_proj.dot(p_proj);

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
    a_proj += f2.pos;
    return collide_spheres(con, margin, f1.pos, f1.mat, size1[0], a_proj,
                           f2.mat, size2[0]);
  }
  if (collide_cap) {
    const Matrix3& m = f2.mat;
    // The cap at the axis's far end faces the other way: x and z turned.
    Matrix3 flip = m;
    flip.col(0) = -m.col(0);
    flip.col(2) = -m.col(2);
    Vector3 pos_cap = f2.pos + (x > 0 ? height : -height) * axis;
    std::uint32_t n = collide_plane_sphere(con, margin, pos_cap,
                                           x > 0 ? m : flip, f1.pos, size1[0]);
    if (n != 0) {
      con[0].normal = -con[0].normal;
    }
    return n;
  }
  p_proj = p_proj * (size2[0] / std::sqrt(p_proj_sqr));
  vec = axis * (x > 0 ? height : -height);
  vec += p_proj;
  vec += f2.pos;
  return collide_spheres(con, margin, f1.pos, f1.mat, size1[0], vec, f2.mat,
                         0.0);
}

// mjraw_CapsuleCapsule.
auto collide_capsules(Out1 con, double margin, const GeomFrame& f1,
                      const Vector3& size1, const GeomFrame& f2,
                      const Vector3& size2) -> std::uint32_t {
  const Matrix3& mat1 = f1.mat;
  const Matrix3& mat2 = f2.mat;
  Vector3 axis1 = mat1.col(2) * size1[1];
  Vector3 axis2 = mat2.col(2) * size2[1];
  Vector3 dif = f1.pos - f2.pos;
  double ma = axis1.dot(axis1);
  double mb = -axis1.dot(axis2);
  double mc = axis2.dot(axis2);
  double u = -axis1.dot(dif);
  double v = axis2.dot(dif);
  double det = ma * mc - mb * mb;
  auto spheres = [&](Out1 out, const Vector3& vec1, const Vector3& vec2) {
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
    return spheres(con, axis1 * x1 + f1.pos, axis2 * x2 + f2.pos);
  }
  Vector3 vec1 = f1.pos + axis1;
  double x2 = clip((v - mb) / mc, -1, 1);
  std::uint32_t n1 = spheres(con, vec1, axis2 * x2 + f2.pos);
  vec1 = f1.pos - axis1;
  x2 = clip((v + mb) / mc, -1, 1);
  std::uint32_t n2 = spheres(con.subspan(n1), vec1, axis2 * x2 + f2.pos);
  if (n1 + n2 >= 2) {
    return n1 + n2;
  }
  Vector3 vec2 = f2.pos + axis2;
  double x1 = clip((u - mb) / ma, -1, 1);
  std::uint32_t n3 = spheres(con.subspan(n1 + n2), axis1 * x1 + f1.pos, vec2);
  if (n1 + n2 + n3 >= 2) {
    return n1 + n2 + n3;
  }
  vec2 = f2.pos - axis2;
  x1 = clip((u + mb) / ma, -1, 1);
  std::uint32_t n4 =
      spheres(con.subspan(n1 + n2 + n3), axis1 * x1 + f1.pos, vec2);
  return n1 + n2 + n3 + n4;
}

//-- Boxes (engine_collision_box.c) -------------------------------------------

// mjraw_SphereBox.
auto collide_sphere_box(Out1 con, double margin, const Vector3& pos1,
                        double radius1, const Vector3& pos2,
                        const Matrix3& mat2, const Vector3& size2)
    -> std::uint32_t {
  Vector3 tmp = pos1 - pos2;
  Vector3 center = mat2.transpose() * tmp;
  Vector3 clamped = center;
  for (int i = 0; i < 3; ++i) {
    if (size2[i] > 0) {
      clamped[i] = clip(clamped[i], -size2[i], size2[i]);
    }
  }
  Vector3 deepest = center;
  tmp = clamped - center;
  double dist = normalize(InOut(tmp));
  if (dist - radius1 > margin) {
    return 0;
  }
  Vector3 pos = Vector3::Zero();
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
    Vector3 nearest = Vector3::Zero();
    nearest[k / 2] = k % 2 != 0 ? -1 : 1;
    pos = center;
    pos += ((radius1 - closest) / 2) * nearest;
    con[0].normal = mat2 * nearest;
    dist = -closest;
  } else {
    deepest += radius1 * tmp;
    pos += 0.5 * clamped;
    pos += 0.5 * deepest;
    con[0].normal = mat2 * tmp;
  }
  tmp = mat2 * pos;
  con[0].pos = tmp + pos2;
  con[0].dist = dist - radius1;
  con[0].tangent = Vector3::Zero();
  return 1;
}

// mjraw_CapsuleBox: the point of the capsule's segment nearest the box, by
// its ends against the faces and the segment against the edges, then a
// second point along the segment when it lies within 45 degrees of a face
// or an edge, each a sphere against the box.
auto collide_capsule_box(Out1 con, double margin, const GeomFrame& f1,
                         const Vector3& size1, const GeomFrame& f2,
                         const Vector3& size2) -> std::uint32_t {
  double halflength = size1[1];
  double secondpos = -4;
  Vector3 pos = f2.mat.transpose() * (f1.pos - f2.pos);
  Vector3 axis = f2.mat.transpose() * f1.mat.col(2);
  Vector3 halfaxis = axis * halflength;

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
    Vector3 tmp1 = pos;
    tmp1 += i * halfaxis;
    Vector3 tmp2 = tmp1;
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
    tmp1 -= tmp2;
    double dist = tmp1.dot(tmp1);
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
      Vector3 tmp3{((i & 1) != 0 ? 1 : -1) * size2[0],
                   ((i & 2) != 0 ? 1 : -1) * size2[1],
                   ((i & 4) != 0 ? 1 : -1) * size2[2]};
      tmp3[j] = 0;
      Vector3 dif = tmp3 - pos;
      double ma = size2[j] * size2[j];
      double mb = -size2[j] * halfaxis[j];
      double mc = size1[1] * size1[1];
      double u = -size2[j] * dif[j];
      double v = halfaxis.dot(dif);
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
      dif = tmp3 - pos;
      dif -= x2 * halfaxis;
      dif[j] += size2[j] * x1;
      double d2 = dif.dot(dif);
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
      Vector3 tmp1 = pos;
      tmp1 -= mul * halfaxis;
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

  Vector3 tmp1 = pos;
  tmp1 += bestsegmentpos * halfaxis;
  Vector3 tmp2 = f2.mat * tmp1 + f2.pos;
  std::uint32_t n =
      collide_sphere_box(con, margin, tmp2, size1[0], f2.pos, f2.mat, size2);
  if (secondpos > -3) {
    tmp1 = pos;
    tmp1 += (secondpos + bestsegmentpos) * halfaxis;
    tmp2 = f2.mat * tmp1 + f2.pos;
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

using Polygon = std::array<Vector3, MAXVERT>;

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
    const Vector3& p = in[k];
    int k1 = k + 1 == nin ? 0 : k + 1;
    double dp = d[k];
    double dq = d[k1];
    if (dp <= 0 && nout < MAXVERT) {
      out[nout++] = p;
    }
    if (((dp < 0 && dq > 0) || (dp > 0 && dq < 0)) && nout < MAXVERT) {
      const Vector3& q = in[k1];
      double t = dp / (dp - dq);
      out[nout] = p + t * (q - p);
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
                   const Vector3& size1, const GeomFrame& f2,
                   const Vector3& size2) -> std::uint32_t {
  const Vector3& pos1 = f1.pos;
  const Vector3& pos2 = f2.pos;
  const Matrix3& mat1 = f1.mat;
  const Matrix3& mat2 = f2.mat;
  Vector3 pos21 = mat1.transpose() * (pos2 - pos1);
  Vector3 pos12 = mat2.transpose() * (pos1 - pos2);
  // The second box's axes in the first's frame.
  Matrix3 rot = mat1.transpose() * mat2;
  Matrix3 rotabs = rot.cwiseAbs();

  double septol = margin + SEPEPS * (size1[0] + size1[1] + size1[2] + size2[0] +
                                     size2[1] + size2[2]);
  double sep_best = -MAXVAL;
  int code = -1;
  for (int i = 0; i < 3; ++i) {
    double radius2 = rotabs.row(i).dot(size2);
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
    double radius1 = rotabs.col(j).dot(size1);
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
      double ax1 = -rot(i2, j);
      double ax2 = rot(i1, j);
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
      double a2_1 = ax1 * rot(i1, j1) + ax2 * rot(i2, j1);
      double a2_2 = ax1 * rot(i1, j2) + ax2 * rot(i2, j2);
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
    Vector3 axis = Vector3::Zero();
    axis[i] = 0;
    axis[i1] = -rot(i2, j);
    axis[i2] = rot(i1, j);
    normalize(InOut(axis));
    return axis;
  };

  // An edge axis nearly along the best face's yields to the face.
  if (code >= 6) {
    Vector3 axis = edge_axis((code - 6) / 3, (code - 6) % 3);
    double face_dot = 0;
    if (code_face < 3) {
      face_dot = std::abs(axis[code_face]);
    } else {
      int f = code_face - 3;
      face_dot = std::abs(axis.dot(rot.col(f)));
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
    Vector3 axis = edge_axis(i, j);
    if (axis.dot(pos21) < 0) {
      axis = -axis;
    }
    Vector3 a2 = rot.transpose() * axis;
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
    Vector3 d2 = rot.col(j);
    double b = d2[i];
    double denom = 1 - b * b;
    Vector3 w1 = Vector3::Zero();
    Vector3 w2 = Vector3::Zero();
    double best_d2 = MAXVAL;
    for (int v1 = 0; v1 < (amb1 >= 0 ? 2 : 1); ++v1) {
      for (int v2 = 0; v2 < (amb2 >= 0 ? 2 : 1); ++v2) {
        Vector3 c1 = Vector3::Zero();
        c1[i] = 0;
        c1[i1] = axis[i1] >= 0 ? size1[i1] : -size1[i1];
        c1[i2] = axis[i2] >= 0 ? size1[i2] : -size1[i2];
        if (amb1 >= 0 && v1 != 0) {
          c1[amb1] = -c1[amb1];
        }
        Vector3 cc = Vector3::Zero();
        cc[j] = 0;
        cc[j1] = a2[j1] >= 0 ? -size2[j1] : size2[j1];
        cc[j2] = a2[j2] >= 0 ? -size2[j2] : size2[j2];
        if (amb2 >= 0 && v2 != 0) {
          cc[amb2] = -cc[amb2];
        }
        Vector3 c2 = rot * cc + pos21;
        Vector3 e = c2 - c1;
        double d1e = e[i];
        double d2e = d2.dot(e);
        double s = denom < MINVAL ? 0 : (d1e - b * d2e) / denom;
        s = clip(s, -size1[i], size1[i]);
        double t = clip(b * s - d2e, -size2[j], size2[j]);
        s = clip(d1e + b * t, -size1[i], size1[i]);
        Vector3 p1 = c1;
        p1[i] += s;
        Vector3 p2 = c2;
        p2 += t * d2;
        Vector3 gap = p2 - p1;
        double gap2 = gap.dot(gap);
        if (gap2 < best_d2) {
          best_d2 = gap2;
          w1 = p1;
          w2 = p2;
        }
      }
    }
    double dist = (w2 - w1).dot(axis);
    if (dist > septol) {
      return 0;
    }
    Vector3 mid = 0.5 * (w1 + w2);
    con[0].dist = dist;
    con[0].pos = mat1 * mid + pos1;
    con[0].normal = mat1 * axis;
    con[0].tangent = Vector3::Zero();
    return 1;
  }

  // A face: the incident face clipped to the reference face.
  bool ref1 = code < 3;
  int a = ref1 ? code : code - 3;
  const Vector3& sizeref = ref1 ? size1 : size2;
  const Vector3& sizeinc = ref1 ? size2 : size1;
  const Vector3& posref = ref1 ? pos1 : pos2;
  const Matrix3& matref = ref1 ? mat1 : mat2;
  const Vector3& posoi = ref1 ? pos21 : pos12;
  Matrix3 rinc = ref1 ? rot : Matrix3{rot.transpose()};
  double sgn = posoi[a] >= 0 ? 1 : -1;
  int binc = 0;
  for (int k = 1; k < 3; ++k) {
    if (std::abs(rinc(a, k)) > std::abs(rinc(a, binc))) {
      binc = k;
    }
  }
  double tinc = sgn * rinc(a, binc) > 0 ? -1 : 1;
  int ax = (a + 1) % 3;
  int ay = (a + 2) % 3;
  int bu = (binc + 1) % 3;
  int bv = (binc + 2) % 3;
  std::array<Polygon, 2> poly;
  Vector3 cx = Vector3::Zero();
  Vector3 du = Vector3::Zero();
  Vector3 dv = Vector3::Zero();
  for (int r = 0; r < 3; ++r) {
    int c = r == 0 ? ax : (r == 1 ? ay : a);
    cx[r] = posoi[c] + tinc * sizeinc[binc] * rinc(c, binc);
    du[r] = sizeinc[bu] * rinc(c, bu);
    dv[r] = sizeinc[bv] * rinc(c, bv);
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

  Polygon accepted;
  int naccept = 0;
  double dupe2 =
      DUPEPS * (sizeref[ax] * sizeref[ax] + sizeref[ay] * sizeref[ay]);
  for (int k = 0; k < nvert; ++k) {
    const Vector3& vertex = (*cur)[k];
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
  Vector3 normal = nsign * matref.col(a);
  for (int k = 0; k < naccept; ++k) {
    const Vector3& v = accepted[k];
    Vector3 posc = Vector3::Zero();
    posc[ax] = v[0];
    posc[ay] = v[1];
    posc[a] = sgn * (sizeref[a] + 0.5 * v[2]);
    con[k].dist = v[2];
    con[k].pos = matref * posc + posref;
    con[k].normal = normal;
    con[k].tangent = Vector3::Zero();
  }
  return static_cast<std::uint32_t>(naccept);
}

// A contact's frame from its normal and tangent (mju_makeFrame).
auto make_frame(const Vector3& normal, const Vector3& tangent) -> Matrix3 {
  Vector3 x = normal;
  normalize(InOut(x));
  Vector3 y = tangent;
  if (y.dot(y) < 0.25) {
    y = Vector3::Zero();
    if (x[1] < 0.5 && x[1] > -0.5) {
      y[1] = 1;
    } else {
      y[2] = 1;
    }
  }
  Vector3 tmp = x * x.dot(y);
  y -= tmp;
  normalize(InOut(y));
  Matrix3 frame;
  frame.row(0) = x;
  frame.row(1) = y;
  frame.row(2) = x.cross(y);
  return frame;
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
  const Vector3& s = geom.size;
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
      frame.pos = body.xmat * geom.pos + body.xpos;
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
      frame.mat = (body.xquat * geom.quat).toRotationMatrix();
      break;
  }
  return frame;
}

auto collide(const Geom& first, const GeomFrame& f1, const Geom& second,
             const GeomFrame& f2, double margin,
             std::span<PreContact, MAX_PAIR_CONTACTS> out) -> std::uint32_t {
  using enum GeomType;
  Out1 con{out};
  const Vector3& size1 = first.size;
  const Vector3& size2 = second.size;
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
    Vector3 dif = p1.pos - p2.pos;
    double reach = r1 + r2 + bound;
    if (dif.dot(dif) > reach * reach) {
      return;
    }
  } else {
    auto above = [&](const GeomFrame& plane, const GeomFrame& other) {
      return (other.pos - plane.pos).dot(plane.mat.col(2));
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
      has_collider(g1->type, g2->type)
          ? collide(*g1, frames[first], *g2, frames[second], bound, found)
          : collide_convex(*g1, frames[first], radii[first], *g2,
                           frames[second], radii[second], bound, found);
  if (n == 0) {
    return;
  }

  // Parameters: the geom of higher priority's, else mixed (mj_contactParam;
  // solmix is 1 for both, so the mix is even).
  Contact contact;
  contact.geom = {first, second};
  contact.include_margin = margin;
  Vector3 friction = Vector3::Zero();
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

}  // namespace simon::articulated
