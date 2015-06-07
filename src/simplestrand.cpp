/*
 * simplestrand.cpp
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
 * Strand with simplified physics.
 *
 */

#include "simplestrand.h"
#include "environment.h"

#include <cstdlib>

/// Constructor
SimpleStrand::SimpleStrand(float length, unsigned int numControlPoints,
                           float crossSectionalArea, float density,
                           float rigidity,
                           const maths::Vector<3, float> & position,
                           const maths::Vector<3, float> & direction)
  : Strand(length, numControlPoints, crossSectionalArea, density, rigidity,
           position, direction),
    _attachee(NULL),
    _attachPosition(0.0f),
    _slackTime(0.0f)
{
  
}

/// Destructor.
SimpleStrand::~SimpleStrand()
{
  if (_attachee) {
    delete _attachee;
  }
}

/// Set the attachee object.
void SimpleStrand::SetAttachee(Object * attachee, const maths::Vector<3, float> & position)
{
  _attachee = attachee;
  _attachPosition = position;
}

/// Advance the strand.
void SimpleStrand::VAdvance(float dt)
{
  // We're simple and can only handle the strand if the attachee exists.
  if (!_attachee) {
    return;
  }
  
  Object::s_defaultEnvironment->AdvanceObject(_attachee);
  Object::s_defaultEnvironment->ApplyConstraintsToObject(_attachee, dt);
  
  // Animate the nodes in a fairly simple way.
  maths::Vector<3, float> base = GetPosition();
  maths::Vector<3, float> end = _attachee->GetLocalPosition(_attachPosition);
  maths::Vector<3, float> direction = end - base;
  
  // Constrain the attachee
  float distSq = direction.sqr();
  float distance = 0.0f;
  if (distSq > _length*_length) {
    _slackTime = 0.0f;
    direction.normalize();
    end = base + direction * _length;
    // Cancel the velocity away from the base.
    _attachee->ApplyImpulseAt(direction * -(direction * _attachee->GetVelocity()), end);
    _attachee->ApplyForceAt(direction * -(direction * _attachee->GetForces()), end);
    direction *= _length;
    // Set new position a bit closer.
    _attachee->SetLocalPosition(end, _attachPosition);
    distance = _length;
  } else {
    _slackTime += dt;
  }
  if (!distance) {
    distance = sqrt(distSq);
  }
  
  float slack = (_length - distance);
  if (slack > distance / 5.0f) {
    slack = distance / 5.0f;
  }
  
  for (unsigned int i = 0; i < _numControlPoints-1; ++i) {
    float factor = (float)i / _numControlPoints;
    
    Object::s_defaultEnvironment->AdvanceObject(&_controlPoints[i]);
    Object::s_defaultEnvironment->ApplyConstraintsToObject(&_controlPoints[i], dt);
    _controlPoints[i].ApplyForce(maths::Vector<3, float>(-1.0f + 2.0f * rand()/RAND_MAX,
                                                         -1.0f + 2.0f * rand()/RAND_MAX,
                                                         0.0f) * (_controlPoints[i].GetMass() * 0.5f));
    
    maths::Vector<3, float> norm = _controlPoints[i].GetPosition() - (base + direction * factor);
    //norm -= direction * ((norm * direction)/(distance*distance));
    float localSlack = 4.0f * factor * (1.0f - factor);
    float toEnd = (_length * (_numControlPoints-i-1)/_numControlPoints);
    if (localSlack > toEnd) {
      localSlack = toEnd;
    }
    if (localSlack < 0.0f) {
      localSlack = 0.0f;
    }
    if (norm.sqr() > localSlack*localSlack) {
      norm.resize(slack);
      _controlPoints[i].SetPosition(base + direction * factor + norm);
    }
#if 0
    norm = _controlPoints[i].GetPosition() - _attachee->GetPosition();
    if (norm.sqr() < _attachee->GetRadius() * _attachee->GetRadius()) {
      float distFromAttachee = norm.mag();
      norm /= distFromAttachee;
      distFromAttachee = _attachee->GetRadius() - distFromAttachee;
      distFromAttachee *= 100.0f;
      _controlPoints[i].ApplyForce(norm * (distFromAttachee));
    }
#endif
  }
  _controlPoints[_numControlPoints-1].SetPosition(end);
}
