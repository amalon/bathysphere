/*
 * collision/Collision.h
 *
 * Copyright (C) 2015 James Hogan <james@albanarts.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
 * (in the file called COPYING).
 *
 *
 * Collision detection functions.
 *
 */

#ifndef _COLLISION_COLLISION_H_
#define _COLLISION_COLLISION_H_

#include <maths/Equations.h>
#include <maths/Line.h>
#include <maths/Plane.h>
#include <maths/Matrix.h>

#include <algorithm>
#include <limits>

namespace Collision
{
  template <unsigned int N, typename T>
  class OrthoRange
  {
    public:
      /*
       * Could represent in different ways, which would basically make it like a
       * maths::Line<N, T, Base>.
       */
      maths::Vector<N, T> min, max;
  };

  template <unsigned int N, typename T, typename LB>
  class Cylinder
  {
    public:
      typedef maths::Line<N, T, LB> CenterLine;

      CenterLine center_line;
      T rad;

      const CenterLine &centerLine(void) const
      {
        return center_line;
      }

      T radius() const
      {
        return rad;
      }
  };

  template <unsigned int N, typename T, typename LB>
  bool RayIntersectPlane(const maths::Line<N, T, LB> &ray,
                         const maths::Vector<N+1, T> &pl,
                         T &out)
  {
    /*
     * ray.vector() / pl.normal() is the rate of ascension from plane
     * pl(ray.S) is the initial distance from plane
     * -pl(ray.S) / (ray.vector / pl.normal) is the intersection ray parameter
     *
     * If plane is normalized, we can skip the divide in the dot quotient, and
     * it becomes a dot product.
     */
    T incline = ray.vector() * planeNormal(pl);
    T start_alt = plane(pl, ray.startPosition());

    if (unlikely(incline == 0)) {
      if (start_alt == 0) {
        out = 0;
        return true;
      }
      return false;
    }

    out = -start_alt / incline;
    return true;
  }

  /* Same as above, but for only a single axis */
  template <typename T>
  bool RayIntersectPlane1(T ray_vector, T ray_start,
                          T pl_offs, T &out)
  {
    T start_alt = pl_offs - ray_start;

    if (unlikely(ray_vector == 0)) {
      if (start_alt == 0) {
        out = 0;
        return true;
      }
      return false;
    }

    out = start_alt / ray_vector;
    return true;
  }

  /* Initialise ray limits for an infinite forward ray */
  template <typename T>
  void RayInfiniteForward(T &min_param, T &max_param)
  {
    min_param = 0;
    max_param = std::numeric_limits<T>::infinity();
  }

  /* Adjust ray limits forward to last min */
  template <typename T>
  void RayForwardTo(T &min_param, T &max_param, T max)
  {
    min_param = 0;
    max_param = max;
  }

  /* min_param >= 0 */
  template <typename T, typename LBRay, typename LBCyl>
  bool RayIntersectCylinder(const maths::Line<3, T, LBRay> &ray,
                            const Cylinder<3, T, LBCyl> &cyl,
                            T &min_param, T &max_param)
  {
    const T r2 = maths::sqr(cyl.radius());
    maths::Vector<4, T> end_pl;
    T t0, t1 = 0;
    bool started = false, ended = false;

    /* Find intersections with the planes that cap the cylinder */

    /* First the start of cylinder */
    /* FIXME assumes efficient normal */
    const maths::Vector<3, T> &cyl_start = cyl.centerLine().startPosition();
    end_pl = planeFromNormal(cyl.centerLine().normal(), cyl_start);
    if (RayIntersectPlane(ray, end_pl, t0)) {
      /* Then the end of cylinder */
      maths::Vector<3, T> cyl_end = cyl.centerLine().endPosition();
      end_pl = planeFromNormal(cyl.centerLine().normal(), cyl_end);
      /* Must succeed, same normal as start cap plane */
      RayIntersectPlane(ray, end_pl, t1);

      /* Sort cylinder ends */
      const maths::Vector<3, T> *cyl_lo, *cyl_hi;
      if (t1 >= t0) {
        cyl_lo = &cyl_start;
        cyl_hi = &cyl_end;
      } else {
        std::swap(t0, t1);
        cyl_lo = &cyl_end;
        cyl_hi = &cyl_start;
      }

      /* Possible start */
      if (t0 > max_param)
        return false; /* no intersection */
      if ((ray*t0 - *cyl_lo).sqr() <= r2) {
        if (t0 >= min_param) {
          min_param = t0;
          return true; /* definitely intersected */
        }
        started = true;
      } else if (t0 > min_param) {
        min_param = t0;
      }

      /* Possible end */
      if (t1 < min_param)
        return false; /* no intersection */
      if ((ray*t1 - *cyl_hi).sqr() <= r2) {
        if (t1 < max_param)
          max_param = t1;
        if (started)
          return true; /* no need for full cylinder test */
        ended = true;
      } else if (t1 < max_param) {
        max_param = t1;
      }
    } else {
      /* perpendicular to cylinder, test arbitrary ray point against planes */
      T c0 = cyl.centerLine().closestPoint(ray.startPosition());
      if (c0 < (T)0 || c0 > (T)1)
        return false; /* no intersection */
    }

    /* Now for the full cylinder test */

    /* Calculate the ray start and vector relative to the cylinder */
    T c0 = cyl.centerLine().closestPoint(ray.startPosition());
    T c1 = cyl.centerLine().closestPoint(ray.endPosition());
    maths::Vector<3, T> cyl0 = cyl.centerLine() * c0;
    maths::Vector<3, T> dcyl_dt = cyl.centerLine().vector() * (c1 - c0);
    maths::Vector<3, T> ray_start = ray.startPosition() - cyl0;
    maths::Vector<3, T> ray_vec = ray.vector() - dcyl_dt;

    /*
     * To find point of intersection, need to solve "r2 = |ray|^2"
     * r2 = |ray_start + ray_vec * t|^2
     *    = sum((ray_start[i] + ray_vec[i] * t)^2)
     *    = sum(ray_start[i]^2
     *          + 2t*ray_start[i]*ray_vec[i]
     *          + ray_vec[i]^2*t^2)
     *    =         sum(ray_start[i]^2)
     *      + 2t * sum(ray_start[i]*ray_vec[i])
     *      + t^2 * sum(ray_vec[i]^2)
     *    = ray_start^2 + 2t*(ray_start*ray_vec) + t^2*ray_vec^2
     * That's a quadratic equation (0 = Ax^2 + Bx + C) where:
     * A = ray_vec^2
     * B = 2*ray_start*ray_vec
     * C = ray_start^2 - r2
     */
    T a = ray_vec.sqr();
    T b = (ray_start*ray_vec) * 2;
    T c = ray_start.sqr() - r2;

    unsigned int solutions;
    solutions = maths::QuadraticRoots(a, b, c, t0, t1);
    if (solutions == 0)
      return false; /* No intersection */

    /* a >= 0, so solutions are guaranteed to be ordered */

    if (!started) {
      /* Possible start */
      if (t0 > max_param)
        return false; /* no intersection */
      if (t0 > min_param)
        min_param = t0;
    }

    if (!ended) {
      /* Possible end */
      if (t1 < min_param)
        return false; /* no intersection */
      if (t1 < max_param)
        max_param = t1;
    }

    return true;
  }

  template <unsigned int N, typename T, typename LB>
  bool RayIntersectRange(const maths::Line<N, T, LB> &ray,
                         const OrthoRange<N, T> &range,
                         T &min_param, T &max_param)
  {
    unsigned int i;
    const maths::Vector<N, T> &start = ray.startPosition();
    const maths::Vector<N, T> vec = ray.vector();
    T ray_vec, ray_start, pl[2], param = 0;

    /* Go through each dimention */
    for (i = 0; i < N; ++i) {
      ray_vec = vec[i];
      ray_start = start[i];

      /* determine positions of entrance and exit planes */
      if (start[i] < range.min[i]) {
        /* min plane is entrance, max plane is exit */
        if (ray_vec <= 0)
          return false; /* no intersection */
        pl[0] = range.min[i];
        pl[1] = range.max[i];
      } else if (start[i] > range.max[i]) {
        /* max plane is entrance, min plane is exit */
        if (ray_vec >= 0)
          return false; /* no intersection */
        pl[0] = range.max[i];
        pl[1] = range.min[i];
      } else if (ray_vec > 0) {
        /* max plane is exit */
        pl[1] = range.max[i];
        goto exit_plane;
      } else if (ray_vec < 0) {
        /* min plane is exit */
        pl[1] = range.min[i];
        goto exit_plane;
      } else {
        /* ray_vec == 0, this dimention makes no difference */
        continue;
      }

      /* Entrance plane */
      RayIntersectPlane1(ray_vec, ray_start, pl[0], param);
      if (param > max_param)
        return false; /* no intersection */
      if (param > min_param)
        min_param = param;

exit_plane:
      /* Exit plane */
      RayIntersectPlane1(ray_vec, ray_start, pl[1], param);
      if (param < min_param)
        return false; /* no intersection */
      if (param < max_param)
        max_param = param;
    }

    return true;
  }

  /// Basic triangle description.
  class Triangle
  {
    public:
      maths::Vector<4, float> facePlane;
      maths::Vector<4, float> edgePlanes[3];
      maths::Vector<3, float> vertices[3];

    public:
      inline bool PointInside(const maths::Vector<3, float> & pos) const
      {
        return maths::plane(edgePlanes[0], pos) <= 0.0f &&
               maths::plane(edgePlanes[1], pos) <= 0.0f &&
               maths::plane(edgePlanes[2], pos) <= 0.0f;
      }
  };

  /// Basic line description
  class Sphere
  {
    public:
      float radius;
  };

  /// Describes a collision.
  class Interaction
  {
    public:
      /// displacement to undo the collision
      maths::Vector<3, float> normal;

      /// loction of the collision
      maths::Vector<3, float> location;

      /// Whether the collision was with a backface
      bool backface;

  };

#define NO_COLLISION (-1.0f)

  /// Find the closest point between two lines.
  /**
   * @param a1,a2        @a line1 start and end point.
   * @param b1,b2        @a line2 start and end point.
   * @param otherResult [out] Pointer to float in which to put the length along @a line2 that the closest point is.
   * @return The length along @a line1 that the closest point is.
   */
  float ClosestPointBetweenLines(const maths::Vector<3, float> & a1, const maths::Vector<3, float> & a2,
                                 const maths::Vector<3, float> & b1, const maths::Vector<3, float> & b2,
                                 float * otherResult = NULL);

  /// Find the closest location on the line to a particular point.
  /**
   * @param lineStart    The start position vector of the line.
   * @param lineNormal   The end position vector of the line.
   * @param pos          The position vector of the point.
   * @return Fraction along the line that the closest point is.
   */
  inline float ClosestPointOnLine(const maths::Vector<3, float> & lineStart, const maths::Vector<3, float> & lineEnd,
                                  const maths::Vector<3, float> & pos)
  {
  // Find the Norm
    maths::Vector<3, float> lineNormal = lineEnd - lineStart;
  // Dot the norm with the point
    float mag1 = (pos - lineStart) * lineNormal;
    float mag2 = (pos - lineEnd  ) * lineNormal;
    return mag1 / (mag1 - mag2);
  }

  /// Find collisions between a point and a moving sphere.
  float DetectPointSphere(Interaction & result,
                          const maths::Vector<3, float> & point, const Sphere & sphere,
                          const maths::Vector<3, float> & spherePosition,
                          const maths::Vector<3, float> & sphereMovement);

  /// Find collisions between a line and a moving sphere.
  float DetectLineSphere(Interaction & result,
                         const maths::Vector<3, float> & point1, const maths::Vector<3, float> & point2, const Sphere & sphere,
                         const maths::Vector<3, float> & spherePosition,
                         const maths::Vector<3, float> & sphereMovement);

  /// Find collisions between a plane and a moving sphere.
  float DetectPlaneSphere(Interaction & result, const maths::Vector<4, float> & plane, const Sphere & sphere,
                          const maths::Vector<3, float> & spherePosition,
                          const maths::Vector<3, float> & sphereMovement);

  /// Find collisions between a triangle and a moving sphere.
  float DetectTriangleSphere(Interaction & result,
                             const Triangle & triangle, const Sphere & sphere,
                             const maths::Vector<3, float> & spherePosition,
                             const maths::Vector<3, float> & sphereMovement);
}

#endif // _COLLISION_COLLISION_H_
