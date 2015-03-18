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

#include <maths/Line.h>
#include <maths/Plane.h>

#include <limits>

namespace collision
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
}

#endif // _COLLISION_COLLISION_H_
