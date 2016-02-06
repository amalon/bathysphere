/*
 * maths/Matrix.cpp
 *
 * Copyright (C) 2007-2014 James Hogan <james@albanarts.com>
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
 * Square transformation matrix class.
 *
 * Originally from Computer Graphics and Visualisation Open Assessment.
 *
 */

#include <maths/Matrix.h>
#include "maths/Quaternion.h"

namespace maths {

  template <>
  Matrix<3,float>::Matrix(const Quaternion<float> & q)
  {
    typedef float T;
    T xy = q[0]*q[1];
    T xz = q[0]*q[2];
    T yy = q[1]*q[1];
    T yw = q[1]*q[3];
    T zz = q[2]*q[2];
    T zw = q[2]*q[3];
    setVector3(col[0].v, 1-2*(yy+zz),   2*(xy+zw),   2*(xz-yw));
    T xx = q[0]*q[0];
    T xw = q[0]*q[3];
    T yz = q[1]*q[2];
    setVector3(col[1].v,  2*(xy-zw), 1-2*(xx+zz),   2*(yz+xw));
    setVector3(col[2].v,  2*(xz+yw),   2*(yz-xw), 1-2*(xx+yy));
  }

  /// Get an arbitrary matrix where the z axis points in the direction of Z.
  Matrix<3,float> getTbnMatrix(Vector<3, float> vZ)
  {
    Vector<3,float> vY(0, 0, -1);
    Vector<3,float> vX(cross(vY, vZ));
    if (vX.zero()) {
      vY.set(0,1,0);
      vX = cross(vY, vZ);
    }
    vX.normalize();
    vY = cross(vZ, vX);
    return Matrix<3,float>(vX, vY, vZ);
  }

  // Invert the 3x3 matrix
  /// not written by me
  template <>
  Matrix<3,float> & Matrix<3,float>::invert()
  {
    Matrix a = *this;
    Matrix b(1.0f);

    unsigned int c, r;
    unsigned int cc;
    unsigned int rowMax; // Points to max abs value row in this column
    unsigned int row;
    float tmp;

    // Go through columns
    for (c=0; c<3; c++) {
      // Find the row with max value in this column
      rowMax = c;
      for (r=c+1; r<3; r++) {
        if (fabs(a[c][r]) > fabs(a[c][rowMax])) {
          rowMax = r;
        }
      }

      // If the max value here is 0, we can't invert.  Return identity.
      if (a[rowMax][c] == 0.0F)
        return(setIdentity());

      // Swap row "rowMax" with row "c"
      for (cc=0; cc<3; cc++)
      {
        tmp = a[cc][c];
        a[cc][c] = a[cc][rowMax];
        a[cc][rowMax] = tmp;
        tmp = b[cc][c];
        b[cc][c] = b[cc][rowMax];
        b[cc][rowMax] = tmp;
      }

      // Now everything we do is on row "c".
      // Set the max cell to 1 by dividing the entire row by that value
      tmp = a[c][c];
      for (cc=0; cc<3; cc++) {
        a[cc][c] /= tmp;
        b[cc][c] /= tmp;
      }

      // Now do the other rows, so that this column only has a 1 and 0's
      for (row = 0; row < 3; row++) {
        if (row != c) {
          tmp = a[c][row];
          for (cc=0; cc<3; cc++) {
            a[cc][row] -= a[cc][c] * tmp;
            b[cc][row] -= b[cc][c] * tmp;
          }
        }
      }

    }

    *this = b;
    return *this;
  }

  // Invert the Matrix<4,float>
  template <>
  Matrix<4,float> & Matrix<4,float>::invert()
  {
    Matrix<4,float> a(*this);
    Matrix<4,float> b(1.0f);

    unsigned int r, c;
    unsigned int cc;
    unsigned int rowMax; // Points to max abs value row in this column
    unsigned int row;
    float tmp;

    // Go through columns
    for (c=0; c<4; c++) {
      // Find the row with max value in this column
      rowMax = c;
      for (r=c+1; r<4; r++) {
        if (fabs(a[c][r]) > fabs(a[c][rowMax])) {
          rowMax = r;
        }
      }

      // If the max value here is 0, we can't invert.  Return identity.
      //if (a[rowMax][c] == 0.0F)
      //  return(setIdentity());

      // Swap row "rowMax" with row "c"
      for (cc=0; cc<4; cc++) {
        tmp = a[cc][c];
        a[cc][c] = a[cc][rowMax];
        a[cc][rowMax] = tmp;
        tmp = b[cc][c];
        b[cc][c] = b[cc][rowMax];
        b[cc][rowMax] = tmp;
      }

      // Now everything we do is on row "c".
      // Set the max cell to 1 by dividing the entire row by that value
      tmp = a[c][c];
      for (cc=0; cc<4; cc++) {
        a[cc][c] /= tmp;
        b[cc][c] /= tmp;
      }

      // Now do the other rows, so that this column only has a 1 and 0's
      for (row = 0; row < 4; row++) {
        if (row != c) {
          tmp = a[c][row];
          for (cc=0; cc<4; cc++) {
            a[cc][row] -= a[cc][c] * tmp;
            b[cc][row] -= b[cc][c] * tmp;
          }
        }
      }
    }

    *this = b;
    return *this;
  }


  // Return a 3D axis-rotation maths::Matrix<4,float>
  // Pass in 'x', 'y', or 'z' for the axis.
  Matrix<4,float> RotateRadMatrix44(char axis, float rad)
  {
    Matrix<4,float> ret;
    float sinA, cosA;
    maths::sincos(rad, &sinA, &cosA);

    switch (axis)
    {
      case 0:
      case 'x':
      case 'X':
        ret[0][0] =  1.0F; ret[1][0] =  0.0F; ret[2][0] =  0.0F;
        ret[0][1] =  0.0F; ret[1][1] =  cosA; ret[2][1] = -sinA;
        ret[0][2] =  0.0F; ret[1][2] =  sinA; ret[2][2] =  cosA;
        break;

      case 1:
      case 'y':
      case 'Y':
        ret[0][0] =  cosA; ret[1][0] =  0.0F; ret[2][0] =  sinA;
        ret[0][1] =  0.0F; ret[1][1] =  1.0F; ret[2][1] =  0.0F;
        ret[0][2] = -sinA; ret[1][2] =  0.0F; ret[2][2] =  cosA;
        break;

      case 2:
      case 'z':
      case 'Z':
        ret[0][0] =  cosA; ret[1][0] = -sinA; ret[2][0] =  0.0F;
        ret[0][1] =  sinA; ret[1][1] =  cosA; ret[2][1] =  0.0F;
        ret[0][2] =  0.0F; ret[1][2] =  0.0F; ret[2][2] =  1.0F;
        break;
    }

    ret[0][3] = 0.0F; ret[1][3] = 0.0F; ret[2][3] = 0.0F;
    ret[3][0] = 0.0F;
    ret[3][1] = 0.0F;
    ret[3][2] = 0.0F;
    ret[3][3] = 1.0F;

    return ret;
  }
}
