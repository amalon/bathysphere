/*
 * orientation.h
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
 * Orientation class.
 *
 */

#ifndef _FILE_orientation_h
#define _FILE_orientation_h


#include "maths/Vector.h"
#include "maths/Quaternion.h"
#include "maths/Matrix.h"
#include "glmaths.h"

namespace Physics {
  
  /// Represents an orientation of an object.
  /**
   */
  class Orientation
  {
    protected:
      /// Quaternion rotation.
      maths::Quaternion<float> Q;
      /// Rotation matrix.
      maths::Matrix<3, float>  M;
      
    public:
      /// Default constructor.
      Orientation();
      /// Virtual destructor.
      inline virtual ~Orientation() {}
      
      /// Reset to the identity transformation.
      inline void Identity();
      
      /// Apply an immediate rotation.
      /**
       * @param q Quaternion rotation.
       * @param m Matrix rotation.
       * @pre @a q == @a m
       */
      void ApplyRotation(const maths::Quaternion<float> & q, const maths::Matrix<3, float> & m);
      
      /// Apply an immediate rotation about an axis.
      void ApplyRotation(float angle, maths::Vector<3, float> axis);
      
      /// Set the orientation quaternion.
      void SetQuaternion(const maths::Quaternion<float> & q);
      /// Set the orientation matrix.
      void SetMatrix(const maths::Matrix<3, float> & m);
      
      /// Rotate so that the Z axis is in the direction of normal.
      void RotateToNormal(const maths::Vector<3, float> & normal);
      
      /// Get the orientation as a quaternion.
      inline const maths::Quaternion<float> & GetQuaternion() const;
      inline const maths::Matrix<3, float>  & GetMatrix() const;
      
      /// Apply the orientation to OpenGL.
      inline void ApplyGlTranformation();
      /// Apply the inverse orientation to OpenGL.
      inline void ApplyInverseGlTranformation();
      
    protected:
      /// Callback for when the orientation changes.
      inline virtual void VNewOrientation(const maths::Quaternion<float> & oldOrientation,
                                          const maths::Quaternion<float> & newOrientation) { }
      
  };
  
  // Inline function definitions
  
  /// Reset to the identity transformation.
  inline void Orientation::Identity()
  {
    Q.setIdentity();
    M.setIdentity();
  }
  
  inline const maths::Quaternion<float> & Orientation::GetQuaternion() const
  {
    return Q;
  }
  
  inline const maths::Matrix<3, float>  & Orientation::GetMatrix() const
  {
    return M;
  }
  
  inline void Orientation::ApplyGlTranformation()
  {
    // Turn into a 4 by 4 matrix and send that to OpenGL.
    maths::Matrix<4, float> matrix(M);
    glMultMatrix(matrix);
  }
  
  inline void Orientation::ApplyInverseGlTranformation()
  {
    // Turn into a 4 by 4 matrix and send that to OpenGL.
    maths::Matrix<4, float> matrix(M.transposed());
    glMultMatrix(matrix);
  }
  
}

#endif // _FILE_orientation_h
