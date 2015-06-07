/*
 * maths/glVector.h
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
 * Vector types for GL scalars and inline OpenGL function helpers for those
 * types. These are overloaded to take multiple data types.
 *
 */

#ifndef _MATHS_GLVECTOR_H_
#define _MATHS_GLVECTOR_H_

#include "maths/Vector.h"
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>

// Vector types for OpenGL scalars
typedef maths::Vector<2, GLfloat>  GLvec2f;
typedef maths::Vector<3, GLfloat>  GLvec3f;
typedef maths::Vector<4, GLfloat>  GLvec4f;
typedef maths::Vector<2, GLdouble> GLvec2d;
typedef maths::Vector<3, GLdouble> GLvec3d;
typedef maths::Vector<4, GLdouble> GLvec4d;

// Send a two dimentional vertex to OpenGL
inline void glVertex2(const GLvec2f & v)    { glVertex2fv(v); }
inline void glVertex2(const GLvec2d & v)    { glVertex2dv(v); }

// Send a three dimentional vertex to OpenGL
inline void glVertex3(const GLvec3f & v)    { glVertex3fv(v); }
inline void glVertex3(const GLvec3d & v)    { glVertex3dv(v); }

// Send a four dimentional vertex to OpenGL
inline void glVertex4(const GLvec4f & v)    { glVertex4fv(v); }
inline void glVertex4(const GLvec4d & v)    { glVertex4dv(v); }

// Send a normal vector to OpenGL
inline void glNormal(const GLvec3f & v)     { glNormal3fv(v); }
inline void glNormal(const GLvec4f & v)     { glNormal3fv(v); }
inline void glNormal(const GLvec3d & v)     { glNormal3dv(v); }
inline void glNormal(const GLvec4d & v)     { glNormal3dv(v); }

// Send a two dimentional texture coordinate to OpenGL
inline void glTexCoord2(const GLvec2f & v)  { glTexCoord2fv(v); }
inline void glTexCoord2(const GLvec2d & v)  { glTexCoord2dv(v); }

// Send a three dimentional texture coordinate to OpenGL
inline void glTexCoord3(const GLvec3f & v)  { glTexCoord3fv(v); }
inline void glTexCoord3(const GLvec3d & v)  { glTexCoord3dv(v); }

// Send a two dimentional mulitexture coordinate to OpenGL
inline void glMultiTexCoord2(GLenum target, const GLvec2f & v)
{
  glMultiTexCoord2fv(target, v);
}
inline void glMultiTexCoord2(GLenum target, const GLvec2d & v)
{
  glMultiTexCoord2dv(target, v);
}

// Send a three dimentional mulitexture coordinate to OpenGL
inline void glMultiTexCoord3(GLenum target, const GLvec3f & v)
{
  glMultiTexCoord3fv(target, v);
}
inline void glMultiTexCoord3(GLenum target, const GLvec3d & v)
{
  glMultiTexCoord3dv(target, v);
}

// Translate the matrix by a vector
inline void glTranslate(const GLvec2f & v, GLfloat  z = 0.0f)
{
  glTranslatef(v[0], v[1], z);
}
inline void glTranslate(const GLvec3f & v)
{
  glTranslatef(v[0], v[1], v[2]);
}
inline void glTranslate(const GLvec4f & v)
{
  glTranslatef(v[0], v[1], v[2]);
}
inline void glTranslate(const GLvec2d & v, GLdouble z = 0.0 )
{
  glTranslated(v[0], v[1], z);
}
inline void glTranslate(const GLvec3d & v)
{
  glTranslated(v[0], v[1], v[2]);
}
inline void glTranslate(const GLvec4d & v)
{
  glTranslated(v[0], v[1], v[2]);
}

// Rotate the matrix rad radians about axis
inline void glRotate(const float rad,  const GLvec3f & axis)
{
  glRotatef(rad*57.29577951f, axis[0], axis[1], axis[2]);
}
inline void glRotate(const double rad, const GLvec3d & axis)
{
  glRotated(rad*57.29577951,  axis[0], axis[1], axis[2]);
}

// Scale the matrix by a vector
inline void glScale(const GLvec3f & v) { glScalef(v[0], v[1], v[2]); }
inline void glScale(const GLvec3d & v) { glScaled(v[0], v[1], v[2]); }

#endif // _MATHS_GLVECTOR_H_
