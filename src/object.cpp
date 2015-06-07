/*
 * object.cpp
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
 * Object class representing anything affected by the emvironment.
 *
 */

#include "object.h"
#include "environment.h"
#include "glmaths.h"

#include <cassert>

Environment * Object::s_defaultEnvironment = NULL;

/// Set the default environment.
void Object::SetDefaultEnvironment(Environment * environment)
{
  s_defaultEnvironment = environment;
}

/// Get the default environment.
Environment * Object::GetDefaultEnvironment()
{
  return s_defaultEnvironment;
}

// Render the object (translating as appropriate then calling virtual render function)
void Object::RenderObj(const Observer & observer)
{
  if (_visible) {
    glPushMatrix();
    glTranslate(GetPosition());
    //orientation.applyGlTranformation();
    VRender(observer);
    glPopMatrix();
  }
}



// Constructors
Object::Object()
  : _position(0.0f),
    _velocity(0.0f),
    _density(0.0f),
    _volume(0.0f),
    _radius(0.0f),
    _lastUpdated(Object::s_defaultEnvironment->GetClock()),
    _collisions(false),
    _frozen(false),
    _visible(true),
    _bouyancy(false),
    _alpha(false),
    _delete(false),
    _forces(0.0f),
    _lastFrameId(0)
{
  //UpdateCubeWithNewPosition(GetPosition());
}

// Destructor
Object::~Object()
{
  // Remove references to this obj from the octree
  while (!_octreeCubes.empty()) {
    RemoveObjectFromCube(*_octreeCubes.begin(),this);
  }
}

/// Advance the object using the forces.
void Object::Advance(float clock)
{
  float dt = clock - _lastUpdated;
  _lastUpdated = clock;
  if (_frozen) {
    return;
  }
    // Get the mass.
  float mass = GetMass();
  
  dt *= 1.0f;
  
  // If no mass, we can't really do much with the forces.
  if (dt > 0.0f) {
    VAdvance(dt);
    
    int iterations = 0;
    while (dt > 0.0f && mass && ++iterations < 20) {
      // Find the acceleration.
      // a = f/m
      maths::Vector<3, float> acceleration = _forces / GetMass();
      
      // Find the new velocity.
      // v = u + at
      maths::Vector<3, float> deltaVelocity = acceleration*dt;
      
      // Find the new displacement.
      // x = ut + 0.5at^2
      maths::Vector<3, float> displacement = (_velocity+deltaVelocity)*dt;
      
      if (_collisions) {
        Collision::Interaction interaction;
        float result = VDetectCollision(interaction, _position, _position + displacement);
        
        float untilCollision;
        if (result == NO_COLLISION) {
          untilCollision = 1.0f;
        } else {
          untilCollision = result;
          deltaVelocity *= result;
          displacement *= result;
        }
      
        SetVelocity(_velocity + deltaVelocity);
        SetPosition(_position + displacement);
        
        dt -= untilCollision;
        
        if (result != NO_COLLISION && dt > 0.0f) {
          if (!(interaction.normal.sqr() > 0.999f && interaction.normal.sqr() < 1.001f)) {
            interaction.normal.normalize();
          }
          float downwardVel = -(_velocity * interaction.normal);
          if (downwardVel > 0.0f) {
            SetVelocity(_velocity + (interaction.normal * downwardVel)*1.05f);
          }
          float downwardForce = -(_forces * interaction.normal);
          if (downwardForce > 0.0f) {
            ApplyForceAt((interaction.normal * downwardForce)*1.05f, interaction.location);
          }
          
          if (untilCollision < 1e-8f) {
            dt = 0.0f;
          }
        }
        
      } else {
        SetVelocity(_velocity + deltaVelocity);
        SetPosition(_position + displacement);
        dt = 0.0f;
      }
#ifdef OBJECT_DEBUG_TRAILS
      debug_trail.push_back(GetPosition());
#endif
    }
#ifdef OBJECT_DEBUG_TRAILS
    int count = debug_trail.size();
    while (count-- > 100) {
      debug_trail.pop_front();
    }
#endif
    
    // Reset the forces.
    _forces[0] = _forces[1] = _forces[2] = 0.0f;
  }
}

// This object has moved, update the octree
void Object::VNewPosition(const maths::Vector<3, float> & vOldPos,const maths::Vector<3, float> & vNewPos)
{
  UpdateCubeWithNewPosition(vNewPos);
}

void Object::UpdateCubeWithNewPosition(const maths::Vector<3, float> & vNewPos)
{
  // OPTIMIZE
  // Fail without try if cube list is empty and no default octree
  if (_octreeCubes.empty() && !s_defaultEnvironment)
    return;
  // Check each cube to see if it still contains any of the object
  std::set<OctreeCube*>::iterator it = _octreeCubes.begin();
  float fRad = GetMaxRadius();
restartLoop:
  for (; it != _octreeCubes.end(); ++it) {
    OctreeCube * pTmpCube = *it;
    if (!pTmpCube->ContainsAnyOf(vNewPos,fRad)) {
        // Erase, no longer an intersecting cube
      pTmpCube->_objects.erase(this);
      if (IsAlpha()) {
        // remove from sorted object list.
        std::list<DistanceSortedObject>::iterator it2;
        for (it2 = pTmpCube->_distanceSortedObjects.begin(); it2 != pTmpCube->_distanceSortedObjects.end(); ++it2) {
          if (*it2 == this) {
            std::list<DistanceSortedObject>::iterator tmpid = it2--;
            pTmpCube->_distanceSortedObjects.erase(tmpid);
            break;
          }
        }
      }
      ++it;
      _octreeCubes.erase(pTmpCube);
      pTmpCube->RealityCheck(true);
      goto restartLoop;
    }
  }
  // Add to top level and let it filter down
  OctreeCube * pNearCube;
  if (!_octreeCubes.empty())
    pNearCube = *_octreeCubes.begin();
  else
    pNearCube = NULL;
  // Iterate up until fully contains object
  if (pNearCube) {
    while (pNearCube->_parentCube &&
            !pNearCube->ContainsAllOf(vNewPos,fRad))
    {
      pNearCube = pNearCube->_parentCube;
    }
    // Add this object to the cube to find any other cubes
    AddObjectToCube(pNearCube,this);
  } else {
    // Use s_DefaultOctree to add object
    s_defaultEnvironment->octree.AddObject(this);
  }
}
