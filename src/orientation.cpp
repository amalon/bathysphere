/*
 * orientation.cpp
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

#include "orientation.h"

Physics::Orientation::Orientation()
{
  Identity();
}

// Apply an immediate rotation
void Physics::Orientation::ApplyRotation(const maths::Quaternion<float> & q, const maths::Matrix<3, float> & m)
{
  maths::Quaternion<float> oldQ(Q);
  Q = q*Q;
  M = m*M;
  if (Q.sqr() < 0.99f || Q.sqr() > 1.01f) {
    Q.normalize();
  }
  //M = (maths::Matrix<3, float>)Q;
  VNewOrientation(oldQ,Q);
}

void Physics::Orientation::ApplyRotation(float angle, maths::Vector<3, float> axis)
{
  angle *= 0.5f;
  float sinAngleBy2 = sinf(angle);
  float cosAngleBy2 = cosf(angle);
  maths::Quaternion<float> q(axis[0]*sinAngleBy2,
                             axis[1]*sinAngleBy2,
                             axis[2]*sinAngleBy2,
                             cosAngleBy2);
  ApplyRotation(q, q.toMatrix3());
}

void Physics::Orientation::SetQuaternion(const maths::Quaternion<float> & q)
{
  maths::Quaternion<float> oldQ = Q;
  M = maths::Matrix<3, float>((Q = q).toMatrix3());
  VNewOrientation(oldQ,Q);
}

void Physics::Orientation::SetMatrix(const maths::Matrix<3, float> & m)
{
  maths::Quaternion<float> oldQ = Q;
  Q = maths::Quaternion<float>(M = m);
  VNewOrientation(oldQ,Q);
}

/// Rotate so that the Z axis is in the direction of normal.
void Physics::Orientation::RotateToNormal(const maths::Vector<3, float> & normal)
{
  
}
