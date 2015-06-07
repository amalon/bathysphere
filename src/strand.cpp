/*
 * strand.cpp
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
 * Basic rope physics object.
 *
 */

#include "strand.h"
#include "environment.h"
#include "maths/SplineDefinitions.h"

#include <cstdlib>

/// Substrand destructor.
Strand::SubStrand::~SubStrand()
{
  if (strand) {
    delete strand;
  }
}

/// ControlPoint constructor.
Strand::ControlPoint::ControlPoint()
  : tension(0.0f)
{
}

/// ControlPoint destructor.
Strand::ControlPoint::~ControlPoint()
{
  std::list<SubStrand*>::iterator it = substrands.begin();
  for (; it != substrands.end(); ++it) {
    delete *it;
  }
}

/// Constructor.
Strand::Strand(float length, unsigned int numControlPoints,
               float crossSectionalArea, float density,
               float rigidity, const maths::Vector<3, float> & position, const maths::Vector<3, float> & direction)
  : _length(length),
    _numControlPoints(numControlPoints),
    _randomMovement(1.0f)
{
  SetPosition(position);
  _radius = length;
  
  // Split the strand into sections with a control point each.
  _controlPoints = new ControlPoint[_numControlPoints];
  
  // Fill in the control point information.
  float subLength = _length / _numControlPoints;
  for (unsigned int i = 0; i < _numControlPoints; ++i) {
    _controlPoints[i].SetVolume(subLength * crossSectionalArea);
    _controlPoints[i].SetDensity(density);
    _controlPoints[i].SetRadius(subLength);
    _controlPoints[i].SetPosition(position + direction*(i+1)*subLength);
  }
}

/// Destructor.
Strand::~Strand()
{
  if (_controlPoints) {
    // this will call the SubStrand destructor which will cleanup other
    // Strand objects attached to this strand.
    delete [] _controlPoints;
  }
}

/// Get the position of an arbitrary point on the curve of the strand.
maths::Vector<3, float> Strand::InterpolateStrand(float height) const
{
  // Get the position of a control point X
#define GET_CONTROL_POINT(X) ( \
  (X) < 0                                ? GetPosition() : \
  _controlPoints[((X) < (int)_numControlPoints) ? (X) : (_numControlPoints-1)].GetPosition() \
  ) \

  float topLength = 0.0f;
  for (int i = 0; i < (int)_numControlPoints; ++i) {
    topLength += _controlPoints[i].GetRadius();
    if (topLength >= height) {
      // find the top and bottom position.
      float bottomLength = topLength - _controlPoints[i].GetRadius();
      float u = (height-bottomLength) / _controlPoints[i].GetRadius();
      float u2 = u*u;
      return maths::bSpline.Spline(
          u, u2, u2*u,
          GET_CONTROL_POINT(i-2), GET_CONTROL_POINT(i-1),
          _controlPoints[i].GetPosition(), GET_CONTROL_POINT(i+1));
    }
  }
  return _controlPoints[_numControlPoints-1].GetPosition();
  
#undef GET_CONTROL_POINT
}

/// Add a substrand at a specific height on this strand.
bool Strand::AddSubStrand(Object * substrand, float position)
{
  substrand->SetPosition(InterpolateStrand(position));
  float topLength = 0.0f;
  for (unsigned int i = 0; i < _numControlPoints; ++i) {
    topLength += _controlPoints[i].GetRadius();
    if (topLength >= position) {
      SubStrand * sub = new SubStrand();
      sub->strand = substrand;
      sub->position = position;
      _controlPoints[i].substrands.push_back(sub);
      return true;
    }
  }
  SubStrand * sub = new SubStrand();
  sub->strand = substrand;
  sub->position = _length;
  _controlPoints[_numControlPoints-1].substrands.push_back(sub);
  return true;
}

/// Render this strand.
void Strand::RenderObj(const Observer & observer)
{
  // Don't bother transforming into local space.
  VRenderStrand(observer);
}

/// Set the length of the strand.
/**
 * @todo implement to change length of strand / only draw the end of it.
 */
void Strand::SetLength(float length)
{
  if (_length != length) {
    _length = length;
    length /= _numControlPoints;
    for (unsigned int i = 0; i < _numControlPoints; ++i) {
      _controlPoints[i].SetVolume(_controlPoints[i].GetVolume() * length / _controlPoints[i].GetRadius());
      _controlPoints[i].SetRadius(length);
    }
  }
}

/// Advance the control points in a recursive manner
maths::Vector<3,float> Strand::TailAdvance(float dt, maths::Vector<3,float> base, unsigned int controlPoint)
{
  // move control point
  Object::s_defaultEnvironment->AdvanceObject(&_controlPoints[controlPoint]);
  // get some info that may have changed
  float length = _controlPoints[controlPoint].GetRadius();
  maths::Vector<3, float> pos = _controlPoints[controlPoint].GetPosition();
  maths::Vector<3, float> rel = pos - base;
  // has it gone too far?
  float dist2 = rel.sqr();
  bool tight = dist2 >= length*length;
  if (tight) {
    // bring in to max magnitude length
    float dist = sqrt(dist2);
    rel /= dist;
    _controlPoints[controlPoint].SetPosition(pos - rel * (dist - length));
    // The following line tends to make the strand do crazy stuff, waving around and around in a circle.
    //_controlPoints[controlPoint].SetVelocity(_controlPoints[controlPoint].GetVelocity() - (rel * ((dist - length)*5.0f)));
  }
  // apply forces to this part of the strand
  Object::s_defaultEnvironment->ApplyConstraintsToObject(&_controlPoints[controlPoint], dt);
  // Apply a random force to shake things up
  if (_randomMovement) {
    _controlPoints[controlPoint].ApplyForce(maths::Vector<3, float>(-1.0f + 2.0f * rand()/RAND_MAX,
                                                                    -1.0f + 2.0f * rand()/RAND_MAX,
                                                                    -1.0f + 2.0f * rand()/RAND_MAX) * (_controlPoints[controlPoint].GetMass() * _randomMovement));
  }
  if (controlPoint < _numControlPoints-1) {
    // apply any outward force from further parts of the strand.
    _controlPoints[controlPoint].ApplyForce(TailAdvance(dt, _controlPoints[controlPoint].GetPosition(),
                                            controlPoint+1) * (_controlPoints[controlPoint].GetMass() / _controlPoints[controlPoint+1].GetMass()));
  }
  // Any substrands, advance them too
  std::list<SubStrand*>::iterator it = _controlPoints[controlPoint].substrands.begin();
  for (; it != _controlPoints[controlPoint].substrands.end(); ++it) {
    SubStrand* object = *it;
    // Advance it
    Object::s_defaultEnvironment->AdvanceObject(object->strand);
    // Constrain to be touching the curve.
    maths::Vector<3,float> strandPos = object->strand->GetPosition();
    object->strand->SetPosition(InterpolateStrand(object->position));
    //object->strand->SetVelocity(object->strand->GetVelocity() + (object->strand->GetPosition()-strandPos)/dt);
    Object::s_defaultEnvironment->ApplyConstraintsToObject(object->strand, dt);
    //_controlPoints[controlPoint].ApplyForce(object->strand->GetForces() * (_controlPoints[controlPoint].GetMass() / object->strand->GetMass()));
  }
  // transmit any forces on to the previous node
  _controlPoints[controlPoint].tension = maths::Vector<3, float>(0.0f);
  if (tight && controlPoint) {
    // transmit outward forces to base if under tension
    maths::Vector<3,float> forces = _controlPoints[controlPoint].GetForces();
    float tension = forces * rel;
    if (tension > 0.0f) {
      forces = rel * (-tension);
      _controlPoints[controlPoint].tension = forces;
      // this force is the tension in the strand keeping this part of the strand
      // from floating off
      _controlPoints[controlPoint-1].ApplyForce(-forces);
      // this allows the base to handle the force
      return forces;
    }
  }
  return maths::Vector<3, float>(0.0f);
}

void Strand::VAdvance(float dt)
{
  ApplyForce(TailAdvance(dt, GetPosition(), 0));
}
