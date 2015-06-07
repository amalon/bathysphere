/*
 * observer.cpp
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
 * Camera class.
 *
 */

#include "observer.h"
#include "object.h"
#include "rigidbody.h"

#include <GL/glu.h>

/// The current observer.
Observer * Observer::s_active = NULL;

/// Default constructor
Observer::Observer()
  : _nearClip(1.0f), _farClip(100.0f), _fov(90.0f*PI/180.0f),
    _position(0.0f),
    _objectLock(NULL),
    _objectLockPosition(0.0f),
    _occlusionPlanesInUse(0)
{
}

/// Destructor
Observer::~Observer()
{
  // If we're active, deactivate.
  if (this == s_active) {
    s_active = NULL;
  }
}

/// Update the observer.
void Observer::Advance(float dt)
{
  if (_objectLock) {
    // If the lock object is a rigid body, use its orientation to transform the
    // lock position out of the local coordinate space.
    RigidBody * rigidBody = dynamic_cast<RigidBody*> (_objectLock);
    if (rigidBody) {
      SetPosition(rigidBody->GetLocalPosition(_objectLockPosition));
    } else {
      SetPosition(_objectLock->GetPosition() + _objectLockPosition);
    }
  }
}

/// Set the OpenGL projection matrix.
void Observer::SetProjection()
{
  gluPerspective(_fov * 180.0f/PI, ((GLfloat) _viewport.w)/((GLfloat)_viewport.h), _nearClip, _farClip);
}

/// Set OpenGL to use this observer.
void Observer::SetModelView()
{
  orientation.ApplyInverseGlTranformation();
  glTranslate(-GetPosition());
}

/// Lock the observer to a position relative to an object.
void Observer::LockToObject(Object * object, const maths::Vector<3,float> & position)
{
  _objectLock = object;
  _objectLockPosition = position;
}
