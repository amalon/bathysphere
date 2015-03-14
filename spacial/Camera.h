/*
 * spacial/Camera.h
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
 * Generic 3D camera class
 *
 */

#ifndef _SPACIAL_CAMERA_H_
#define _SPACIAL_CAMERA_H_

#include <maths/Plane.h>
#include <maths/Vector.h>
#include <maths/glMatrix.h>

namespace spacial
{
  /**
   * Generic Camera object.
   */
  class Camera
  {
  public:
    /* Lens settings */
    float near;
    float far;
    float fov; /* Radians */

    /* Viewport */
    maths::Vector<2, unsigned int> vp_size;
    float aspect;
    maths::Matrix<4, float> inv_clip;

    // Frustum planes equations
    enum {
      PLANE_RIGHT = 0,
      PLANE_LEFT,
      PLANE_TOP,
      PLANE_BOTTOM,
      PLANE_FAR,
      PLANE_NEAR,
    };
    maths::Vector<4, float> frustum_planes[6];

    void updateFrustum(void)
    {
      maths::gl::mat4f clip, proj, modelview;

      /* Get the projection and modelview matrices from OpenGL */
      glGetMatrix(GL_PROJECTION_MATRIX, proj);
      glGetMatrix(GL_MODELVIEW_MATRIX, modelview);

      /* Multiply to get the clip matrix */
      clip = proj * modelview;

      /* Calculate clip planes */
      clipPlanes(clip, frustum_planes);

      /* Calculate inverse clip matrix */
      inv_clip = clip.inverted();
    }

    maths::Vector<3, float> screen_to_modelview(const maths::Vector<3, float> &scr)
    {
      maths::Vector<4, float> mv = inv_clip * (scr, 1.0f);

      return (maths::Vector<3, float>)mv / mv[3];
    }

    maths::Vector<3, float> clip_to_vec(const maths::Vector<2, float> &clip)
    {
      return (screen_to_modelview((clip, 1)) -
              screen_to_modelview((clip, 0))).normalized();
    }
  };
}

#endif // _SPACIAL_CAMERA_H_
