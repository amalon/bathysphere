/*
 * rangeconstraint.cpp
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
 * A simple distance constraint between two objects.
 *
 */

#include "rangeconstraint.h"
#include "object.h"

#include <cassert>

/// Constructor
RangeConstraint::RangeConstraint(Object * first, Object * second, float maxDistance)
  : _maxDistance(0.0f),
    _direction(maxDistance)
{
  /// @pre @a first and @a second must point to valid Objects
  assert(first && second && "Object * first, Object * second must point to valid objects");
  _objects[0] = first;
  _objects[1] = second;
  _tempImpulses[0].set(0.0f);
  _tempImpulses[1].set(0.0f);
  
  for (int i = 0; i < 2; ++i) {
    _objects[i]->hardConstraints.push_back(this);
  }
}

/// Destructor
RangeConstraint::~RangeConstraint()
{
  // Remove references to this constraint from both objects.
  for (int i = 0; i < 2; ++i) {
    if (_objects[i]) {
      std::list<HardConstraint*>::iterator it;
      for (it = _objects[i]->hardConstraints.begin(); it != _objects[i]->hardConstraints.end(); ++it) {
        if (*it == this) {
          _objects[i]->hardConstraints.erase(it);
          break;
        }
      }
    }
  }
}

/// Get the distance from @a self to the other object involved in the constraint.
maths::Vector<3, float> RangeConstraint::GetDirection(const Object * self)
{
  if (_direction.zero()) {
    _direction = _objects[1]->GetPosition() - _objects[0]->GetPosition();
    if (!_direction.zero()) {
      _direction.normalize();
    }
  }
  int selfId = GetObjectIndex(self);
  /// @pre @a self must be a part of this constraint.
  assert(selfId != -1 && "Object * self must be a part of this constraint");
  if (selfId == 0) {
    return _direction;
  } else {
    return -_direction;
  }
}

/// Get the other object.
const Object * RangeConstraint::GetOther(const Object * self) const
{
  /// @pre @a self must be a part of this constraint.
  if (self == _objects[0]) {
    return _objects[1];
  } else {
    assert(self == _objects[1] && "Object * self must be a part of this constraint");
    return _objects[0];
  }
}

/// Indicate that an object has moved.
void RangeConstraint::ObjectUpdated(const Object * self)
{
  _direction.set(0.0f);
}

/// Get the impulse required to move an object at a particular velocity.
maths::Vector<3, float> RangeConstraint::ImpulseRequired(const Object * self, const maths::Vector<3, float> & velocity)
{
  int selfId = GetObjectIndex(self);
  const Object * other = _objects[1-selfId];
  // project velocity to be in the direction of the constraint
  maths::Vector<3, float> direction = GetDirection(self);
  float distanceSq = (other->GetPosition() - self->GetPosition()).sqr();
  float speed = velocity * direction;
  if (speed <= 0.0f || distanceSq < _maxDistance*_maxDistance) {
    // The constraint is slack, and cannot have any tension
    _tempImpulses[selfId].set(0.0f);
    return _tempImpulses[selfId];
  } else {
    // The constraint is taut, and will carry a tension
    // Find the impulse through the constraint to move the other object.
    maths::Vector<3, float> impulse = direction * ((direction * (velocity - other->GetVelocity())) * other->GetMass());
    _tempImpulses[selfId] = impulse;
    maths::Vector<3, float> projectedVelocity = direction * speed;
    std::list<HardConstraint*>::const_iterator it;
    for (it = other->hardConstraints.begin(); it != other->hardConstraints.end(); ++it) {
      if (*it != this) {
        // Find the impulse required in the direction of the constraint
        impulse += direction * (direction * (*it)->ImpulseRequired(other, projectedVelocity));
      }
    }
    return impulse;
  }
}

/// Apply an impulse through the constraint.
void RangeConstraint::ApplyImpulse(Object * self, float impulseFactor)
{
  int selfId = GetObjectIndex(self);
  Object * other = _objects[1-selfId];
  // project velocity to be in the direction of the constraint
  if (!_tempImpulses[selfId].zero()) {
    // The constraint is taut, and will carry a tension
    // the impulse required is _tempImpulses[selfId]
    self->ApplyImpulse(_tempImpulses[selfId] * impulseFactor);
    // the impulse provided is impulse
    std::list<HardConstraint*>::const_iterator it;
    for (it = other->hardConstraints.begin(); it != other->hardConstraints.end(); ++it) {
      if (*it != this) {
        // Apply the impulse
        (*it)->ApplyImpulse(other, impulseFactor);
      }
    }
  }
}

/// Attempt to get one of the objects moving at a particular velocity.
void RangeConstraint::AttemptVelocity(Object * self, const maths::Vector<3, float> & velocity)
{
  maths::Vector<3, float> direction = GetDirection(self);
  const Object * other = GetOther(self);
  // The impulse required in order to satisfy the constraint.
  float constraintImpulse = direction * ImpulseRequired(self, velocity);
  // The impulse required by the object to meet the desired velocity.
  float selfImpulse = direction * ((velocity - self->GetVelocity()) * other->GetMass());
  // We only want just enough impulse to reach the velocity.
  float impulse = std::min(constraintImpulse, selfImpulse);
  
  // Apply the impulse to the other objects.
  ApplyImpulse(self, impulse/constraintImpulse);
}

/// Get the index of a given object.
int RangeConstraint::GetObjectIndex(const Object * self) const
{
  if (self == _objects[0])
    return 0;
  else if (self == _objects[1])
    return 1;
  else
    return -1;
}
