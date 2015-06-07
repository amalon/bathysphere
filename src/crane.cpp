/*
 * crane.cpp
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
 * A general purpose crane object.
 *
 */

#include "crane.h"
#include "environment.h"
#include "primitives.h"
#include "glmaths.h"
#include "constants.h"

#include <cmath>

/// Constructor.
Crane::Crane()
  : _tether(NULL),
    _height(4.0f),
    _armLength(6.0f),
    _maxTetherLength(110.0f),
    _rotation(0.0f),
    _elevation(0.0f),
    _windLength(0.5f),
    _windSpeed(0.0f)
{
  _radius = _maxTetherLength + _height;
}

/// Destructor.
Crane::~Crane()
{
  ClearActions();
  if (_tether) {
    delete _tether;
  }
}

/// Advance the action.
float Crane::ActionWait::Advance(Crane * crane, float dt)
{
  if (dt < _timeout) {
    _timeout -= dt;
    return dt;
  } else {
    return _timeout;
  }
}

/// Advance the action.
float Crane::ActionRotate::Advance(Crane * crane, float dt)
{
  bool done;
  done  = crane->RotateTowards(_destinationRotation, dt);
  done  = crane->ElevateTowards(_destinationElevation, dt) && done;
  return done ? 0.0f : dt;
}

/// Advance the action.
float Crane::ActionWind::Advance(Crane * crane, float dt)
{
  bool done;
  // If we're there, then great.
  if (_destinationWind == crane->GetWindLength()) {
    done = true;
  } else {
    // Get how long the tether has been slack for
    float slackTime = crane->GetTether()->GetSlackTime();
    // If our detination wind length is shorter than the current wind length
    // then we need to wind in which will produce the slack, so hardwire the
    // value to 0 to get past the next couple of if statements.
    if (_destinationWind < crane->GetWindLength()) {
      slackTime = 0.0f;
    }
    if (slackTime > 15.0f) {
      // Its been slack for a while, so just stop now.
      done = true;
    } else if (slackTime > 1.0f) {
      // Its been slack for a short time, slow the winding so that if the object
      // is just experiencing drag, the winder can wind at the same speed.
      _currentSpeed -= crane->GetMaxWindingAcceleration() * dt;
      if (_currentSpeed < 0.0f) {
        _currentSpeed = 0.0f;
      }
    } else {
      // Theres plenty of tension in the rope, so accelerate the winder.
      _currentSpeed += crane->GetMaxWindingAcceleration() * dt;
      if (_currentSpeed > crane->GetMaxWindingSpeed()) {
        _currentSpeed = crane->GetMaxWindingSpeed();
      }
    }
    // Now advance the winder.
    done = crane->WindTowards(_destinationWind, _currentSpeed, dt);
  }
  return done ? 0.0f : dt;
}

/// Rotate towards a rotation.
bool Crane::RotateTowards(float rotation, float dt)
{
  float diff = fmod(PI + rotation - _rotation, 2*PI);
  diff -= PI;
  if (diff <= -PI) {
    diff += 2*PI;
  }
  float speed = GetMaxRotationSpeed();
  bool finished;
  if (diff > 0.0f) {
    finished = diff < dt*speed;
    if (finished) {
      _rotation += diff;
    } else {
      _rotation += dt*speed;
    }
  } else if (diff < 0.0f) {
    finished = -diff < dt*speed;
    if (finished) {
      _rotation += diff;
    } else {
      _rotation -= dt*speed;
    }
  } else {
    finished = true;
  }
  
  _rotation = fmod(_rotation, 2*PI);
  if (diff < 0.0f) {
    diff += 2*PI;
  }
  return finished;
}

/// Elevate towards an elevation.
bool Crane::ElevateTowards(float elevation, float dt)
{
  float diff = elevation - _elevation;
  float speed = GetMaxElevationSpeed();
  bool finished;
  if (diff > 0.0f) {
    finished = diff < dt*speed;
    if (finished) {
      _elevation = elevation;
    } else {
      _elevation += dt*speed;
    }
  } else if (diff < 0.0f) {
    finished = -diff < dt*speed;
    if (finished) {
      _elevation = elevation;
    } else {
      _elevation -= dt*speed;
    }
  } else {
    finished = true;
  }
  
  if (_elevation < 0.0f) {
    _elevation = 0.0f;
  } else if (_elevation > PI/3) {
    _elevation = PI/3;
  }
  return finished;
}

/// Wind towards a length.
bool Crane::WindTowards(float length, float speed, float dt)
{
  float diff = length - _windLength;
  speed = std::min(speed, GetMaxWindingSpeed());
  bool finished;
  if (diff > 0.0f) {
    finished = diff < dt*speed;
    if (finished) {
      _windLength = length;
    } else {
      _windLength += dt*speed;
    }
  } else if (diff < 0.0f) {
    finished = -diff < dt*speed;
    if (finished) {
      _windLength = length;
    } else {
      _windLength -= dt*speed;
    }
  } else {
    finished = true;
  }
  
  if (_windLength < 0.2f) {
    _windLength = 0.2f;
    finished = true;
  } else if (_windLength > _maxTetherLength) {
    _windLength = _maxTetherLength;
    finished = true;
  }
  return finished;
}

/// Clear the action queue.
void Crane::ClearActions()
{
  std::list<Action*>::iterator it;
  for (it = _actionQueue.begin(); it != _actionQueue.end(); ++it) {
    delete *it;
  }
  _actionQueue.clear();
}

/// Add an action to the queue.
void Crane::PushAction(Action * action)
{
  _actionQueue.push_back(action);
}
    
/// Set the new tether of the crane.
void Crane::SetTether(Cord * tether)
{
  if (_tether) {
    delete _tether;
  }
  _tether = tether;
  if (_tether) {
    _tether->SetPosition(GetPosition() + GetArmTip());
  }
}

/// The crane has moved.
void Crane::VNewPosition(const maths::Vector<3, float> & vOldPos,const maths::Vector<3, float> & vNewPos)
{
  Object::VNewPosition(vOldPos, vNewPos);
  // Update the position of the tether.
  if (_tether) {
    _tether->SetPosition(GetLocalPosition(GetArmTip()));
  }
}

/// Advance the crane.
void Crane::VAdvance(float dt)
{
  float timeLeft = dt;
  while (timeLeft > 0.0f && !_actionQueue.empty()) {
    timeLeft -= _actionQueue.front()->Advance(this, timeLeft);
    if (timeLeft > 0.0f) {
      delete _actionQueue.front();
      _actionQueue.pop_front();
    }
  }
  
  if (_tether) {
    Object::s_defaultEnvironment->AdvanceObject(_tether);
    _tether->SetLength(_windLength);
    Object::s_defaultEnvironment->ApplyConstraintsToObject(_tether, dt);
  }
}

/// Render the crane.
void Crane::VRender(const Observer & observer)
{
  // Metal
  {
    static float diffuse[4]  = {0.8f, 0.2f, 0.2f, 1.0f};
    static float specular[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
  }
  
  // Main crane + arm
  glPushMatrix();
  {
    // Main tower
    Drawing::Cylinder(0.5f, 1.0f, _height, 0.0f, 32, true, false);
    
    glTranslatef(0.0f, 0.0f, _height);
    
    // Rotate around
    glRotatef(_rotation*180.0f/PI, 0.0f, 0.0f, 1.0f);
    // Joint
    glPushMatrix();
    {
      glRotatef(180.0f, 1.0f, 0.0f, 1.0f);
      Drawing::Cylinder(0.5f, 0.5f, 0.8f, -0.8f, 32, true, true);
    }
    glPopMatrix();
    // Arm
    glPushMatrix();
    {
      glRotatef(90.0f - _elevation*180.0f/PI, 1.0f, 0.0f, 0.0f);
      Drawing::Cylinder(0.1f, 0.5f, _armLength, 0.0f, 6, true, false);
    }
    glPopMatrix();
    // Coils
    bool above = Object::s_defaultEnvironment->GetAboveWater();
    if (above) {
  #define CABLE_SECTIONS_PER_METRE 5
  #define CABLE_SECTIONS 6
      glPushMatrix();
      {
        glTranslatef(0.0f, 2.0f, -2.0f);
        
        float coilRadius = 0.5f;
        float cableRadius = 0.02f;
        float coilHalfLength = 1.0f;
        
        // The thing around it.
        glPushMatrix();
        {
          glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
          Drawing::Cylinder(coilRadius, coilRadius, coilHalfLength, -coilHalfLength, 32);
          Drawing::ThickDisk(1.0f, 0.05f, 32, -coilHalfLength-0.05f);
          Drawing::ThickDisk(1.0f, 0.05f, 32, +coilHalfLength+0.05f);
        }
        glPopMatrix();
        
        if (_tether) {
        
          // Coil of cable
          int cableSection;
          int turnsPerRow = (int)(coilHalfLength / cableRadius);
          float cableLength = (_maxTetherLength - _windLength - _armLength - _height)*10;
          float lengthPerTurn = PI*coilRadius*2.0f;
          float lengthPerRow = lengthPerTurn * turnsPerRow;
          
          // construct a lookup table of cross section of the cable.
          struct LookupRecord {
            float norm[2];
          } lookup[CABLE_SECTIONS+1];
      
          for (cableSection = 0; cableSection <= CABLE_SECTIONS; ++cableSection) {
            float tubeAngle = 2.0f*PI*cableSection/CABLE_SECTIONS;
            lookup[cableSection].norm[0] = cos(tubeAngle);
            lookup[cableSection].norm[1] = -sin(tubeAngle);
          }
          
          float startRadius = coilRadius + cableRadius;
          int sections = (int)(cableLength*CABLE_SECTIONS_PER_METRE);
          float sectionLength = cableLength / sections;
          
          float direction[2][3];
          float sincosang[2][2];
          int alternator = 0;
          float prevLengthAlongCable = cableLength + 3.0f;
          direction[alternator][0] = 0.0f;
          direction[alternator][1] = -2.0f;
          direction[alternator][2] = 2.0f;
          sincosang[alternator][0] = 0.0f;
          sincosang[alternator][1] = 1.0f;
          alternator = 1-alternator;
          
          float textureHeight = _tether->GetTextureHeight() * startRadius;
          _tether->SetupMaterial();
          glEnable(GL_TEXTURE_2D);
          for (int i = 0; i < std::min(sections, (int)(2.0f*turnsPerRow*lengthPerTurn/sectionLength)); ++i) {
            float lengthAlongCable = cableLength - sectionLength * i;
            float radius = startRadius + 2.0f*cableRadius * (lengthAlongCable/lengthPerRow);
            float angle = fmod(2.0f*PI*sectionLength*i/lengthPerTurn, 2.0f*PI);
            sincosang[alternator][0] = sin(angle);
            sincosang[alternator][1] = cos(angle);
            direction[alternator][0] = -1.0f + 2.0f*fabs(fmod(lengthAlongCable/lengthPerRow, 2.0f) - 1.0f);
            direction[alternator][0] *= coilHalfLength;
            direction[alternator][1] = radius * sincosang[alternator][0];
            direction[alternator][2] = radius * sincosang[alternator][1];
            glBegin(GL_TRIANGLE_STRIP);
            {
              for (int cableSection = 0; cableSection <= CABLE_SECTIONS; ++cableSection) {
                glTexCoord2f((float)cableSection / CABLE_SECTIONS, -lengthAlongCable * radius / textureHeight);
                glNormal3f(lookup[cableSection].norm[0],
                            sincosang[alternator][0] * lookup[cableSection].norm[1],
                            sincosang[alternator][1] * lookup[cableSection].norm[1]);
                glVertex3f(direction[alternator][0] + cableRadius * lookup[cableSection].norm[0],
                            direction[alternator][1] + cableRadius * sincosang[alternator][0] * lookup[cableSection].norm[1],
                            direction[alternator][2] + cableRadius * sincosang[alternator][1] * lookup[cableSection].norm[1]);
                glTexCoord2f((float)cableSection / CABLE_SECTIONS, -prevLengthAlongCable * radius / textureHeight);
                glNormal3f(lookup[cableSection].norm[0],
                            sincosang[1-alternator][0] * lookup[cableSection].norm[1],
                            sincosang[1-alternator][1] * lookup[cableSection].norm[1]);
                glVertex3f(direction[1-alternator][0] + cableRadius * lookup[cableSection].norm[0],
                            direction[1-alternator][1] + cableRadius * sincosang[1-alternator][0] * lookup[cableSection].norm[1],
                            direction[1-alternator][2] + cableRadius * sincosang[1-alternator][1] * lookup[cableSection].norm[1]);
                
              }
            }
            glEnd();
            alternator = 1-alternator;
            prevLengthAlongCable = lengthAlongCable;
          }
          glDisable(GL_TEXTURE_2D);
        }
      }
      glPopMatrix();
    }
  }
  glPopMatrix();
  
}
