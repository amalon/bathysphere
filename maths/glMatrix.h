/*
 * maths/glMatrix.h
 *
 * Copyright (C) 2006-2014 James Hogan <james@albanarts.com>
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
 * Matrix types for GL scalars and inline OpenGL function helpers for those
 * types. These are overloaded to take multiple data types.
 *
 */

#ifndef _MATHS_GL_MATRIX_H_
#define _MATHS_GL_MATRIX_H_

// Main vector header
#include "maths/Matrix.h"

#include <GL/gl.h>

namespace maths {
  namespace gl {
    // Make use of maths library vector classes
    typedef maths::Matrix<2,float>  mat2f;
    typedef maths::Matrix<3,float>  mat3f;
    typedef maths::Matrix<4,float>  mat4f;
    typedef maths::Matrix<2,double> mat2d;
    typedef maths::Matrix<3,double> mat3d;
    typedef maths::Matrix<4,double> mat4d;
  } // ::maths::gl
} // ::maths

inline void glMultMatrix(const maths::gl::mat3f & matrix)
{
  maths::gl::mat4f m(matrix);
  glMultMatrixf(&m.col[0][0]);
}
inline void glMultMatrix(const maths::gl::mat3d & matrix)
{
  maths::gl::mat4d m(matrix);
  glMultMatrixd(&m.col[0][0]);
}
inline void glMultMatrix(const maths::gl::mat4f & matrix)
{
  glMultMatrixf(&matrix.col[0][0]);
}
inline void glMultMatrix(const maths::gl::mat4d & matrix)
{
  glMultMatrixd(&matrix.col[0][0]);
}

inline void glGetMatrix(GLenum pname, maths::gl::mat4f &out_matrix)
{
  glGetFloatv(pname, &out_matrix[0][0]);
}
inline void glGetMatrix(GLenum pname, maths::gl::mat4d &out_matrix)
{
  glGetDoublev(pname, &out_matrix[0][0]);
}

#endif // _MATHS_GL_MATRIX_H_
