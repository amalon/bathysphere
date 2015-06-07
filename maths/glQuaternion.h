/*
 * maths/glQuaternion.h
 *
 * Copyright (C) 2006-2015 James Hogan <james@albanarts.com>
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
 * Quaternion types for GL scalars and inline OpenGL function helpers for those
 * types. These are overloaded to take multiple data types.
 *
 */

#ifndef _MATHS_GL_QUATERNION_H_
#define _MATHS_GL_QUATERNION_H_

// Maths Library - Quaternion Classes
#include "maths/Quaternion.h"
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>

namespace maths {
  namespace gl {
    // Make use of maths library quaternion classes
    typedef maths::Quaternion<float> quaternionf;
  }
}

// Rotate the modelview matrix by a quaternion
inline void glRotate(const maths::gl::quaternionf & q)
{
  // Convert the quaternion data into simple axis & angle so that it fits in glRotatef
  maths::gl::quaternionf Q = q.ToAxisAngle();
  // Convert Q.w into degrees (from radians) before passing into OpenGL
  glRotatef(57.295779513f * Q[3], Q[0], Q[1], Q[2]);
}

#endif // _MATHS_GL_QUATERNION_H_
