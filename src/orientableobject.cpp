/*
 * orientableobject.cpp
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
 * An object with an orientation.
 *
 */

#include "orientableobject.h"

/// Get the global position of a coordinate local to the object.
maths::Vector<3, float> OrientableObject::GetLocalPosition(const maths::Vector<3, float> & localPosition) const
{
  return GetPosition() + _orientation.GetMatrix() * localPosition;
}

/// Render the object (translating as appropriate then calling virtual render function)
void OrientableObject::RenderObj(const Observer & observer)
{
  glPushMatrix();
  glTranslate(GetPosition());
  _orientation.ApplyGlTranformation();
  VRender(observer);
  glPopMatrix();
}
