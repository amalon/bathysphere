/*
 * boat.cpp
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
 * Boat object.
 *
 */

#include "boat.h"
#include "environment.h"
#include "primitives.h"
#include "glmaths.h"

#define CRANE_STATE_LOADING  0
#define CRANE_STATE_LIFTING  1
#define CRANE_STATE_TURNING  2
#define CRANE_STATE_LOWERING 3
#define CRANE_STATE_WINDING  4

#define CRANE_LIFT_ELEVATION (PI/8)
#define CRANE_REST_ELEVATION (0.0f)
#define CRANE_LOAD_ROTATION  (0.0f)
#define CRANE_WIND_ROTATION  (PI/2)
#define CRANE_MIN_LENGTH         (0.8f)
#define CRANE_MAX_LOADING_LENGTH (1.1f)
#define CRANE_MAX_LENGTH         (100.0f)

#define BOAT_HALF_WIDTH           10.0f
#define BOAT_FRONT_LENGTH         (BOAT_HALF_WIDTH*3.0f)
#define BOAT_MIDDLE_HALF_LENGTH   (BOAT_HALF_WIDTH*2.0f)
#define BOAT_REAR_LENGTH          (BOAT_HALF_WIDTH/2.0f)
#define BOAT_HEIGHT               10.0f
  
#define FRONT_BOTTOM_FACTOR 0.4f

/// Default constructor.
Boat::Boat()
{
  _radius = 10.0f;
  _density = 400.0f;
  _volume = BOAT_HALF_WIDTH*2.0f * BOAT_HEIGHT*0.8f * BOAT_MIDDLE_HALF_LENGTH*2.0f;
  _bouyancy = true;
  
  SetTensor(maths::InertiaTensorCuboid(GetMass(), BOAT_HALF_WIDTH*2.0f, BOAT_HEIGHT*0.8f, BOAT_MIDDLE_HALF_LENGTH*2.0f));
}

/// Destructor.
Boat::~Boat()
{
  
}

/// Constructor
Boat::DeckCrane::DeckCrane()
  : _state(CRANE_STATE_LOADING)
{
  _maxTetherLength = 150.0f;
  _windLength = CRANE_MAX_LOADING_LENGTH;
}

/// Stop the crane.
void Boat::DeckCrane::Stop()
{
  ClearActions();
}

/// Wind the crane in and move to a fixed position.
void Boat::DeckCrane::WindIn()
{
  float rotation = GetRotation();
  
  ClearActions();
  if (_state != CRANE_STATE_LOADING) {
    if (_state != CRANE_STATE_LIFTING) {
      if (_state != CRANE_STATE_TURNING) {
        if (_state != CRANE_STATE_LOWERING) {
          PushAction(new Crane::ActionWind(CRANE_MIN_LENGTH));
          PushAction(new Crane::ActionSet<unsigned int>(_state, CRANE_STATE_LOWERING));
        }
        PushAction(new Crane::ActionRotate(rotation, CRANE_LIFT_ELEVATION));
        PushAction(new Crane::ActionSet<unsigned int>(_state, CRANE_STATE_TURNING));
      }
      PushAction(new Crane::ActionRotate(CRANE_LOAD_ROTATION, CRANE_LIFT_ELEVATION));
      PushAction(new Crane::ActionSet<unsigned int>(_state, CRANE_STATE_LIFTING));
    }
    PushAction(new Crane::ActionRotate(CRANE_LOAD_ROTATION, CRANE_REST_ELEVATION));
    PushAction(new Crane::ActionSet<unsigned int>(_state, CRANE_STATE_LOADING));
  }
  PushAction(new Crane::ActionWind(CRANE_MAX_LOADING_LENGTH));
}

/// Wind the crane to a particular length.
void Boat::DeckCrane::WindTo(float length)
{
  float rotation = GetRotation();
  
  ClearActions();
  if (_state != CRANE_STATE_WINDING) {
    if (_state != CRANE_STATE_LOWERING) {
      if (_state != CRANE_STATE_TURNING) {
        if (_state != CRANE_STATE_LIFTING) {
          PushAction(new Crane::ActionWind(CRANE_MIN_LENGTH));
          PushAction(new Crane::ActionSet<unsigned int>(_state, CRANE_STATE_LIFTING));
        }
        PushAction(new Crane::ActionRotate(rotation, CRANE_LIFT_ELEVATION));
        PushAction(new Crane::ActionSet<unsigned int>(_state, CRANE_STATE_TURNING));
      }
      PushAction(new Crane::ActionRotate(CRANE_WIND_ROTATION, CRANE_LIFT_ELEVATION));
      PushAction(new Crane::ActionSet<unsigned int>(_state, CRANE_STATE_LOWERING));
    }
    PushAction(new Crane::ActionRotate(CRANE_WIND_ROTATION, CRANE_REST_ELEVATION));
    PushAction(new Crane::ActionSet<unsigned int>(_state, CRANE_STATE_WINDING));
  }
  PushAction(new Crane::ActionWind(length));
}

/// Function for doing special bouyancy calculations.
void Boat::CalculateBouyancy(const HeightField * heightField,
                             float densityBeneath, float densityAbove,
                             maths::Vector<3, float> gravitationalFieldStrength)
{
  // Split the volume of the boat into sections.
  // Assume volume equaly spaced along length
  // Assume half of volume down the middle, and quarter on each side
#define BOUYANCY_SECTIONS 10
  float sectionVolume = GetVolume() / BOUYANCY_SECTIONS;
  for (int i = 0; i < BOUYANCY_SECTIONS; ++i) {
    float y = BOAT_MIDDLE_HALF_LENGTH * (-1.0f + 2.0f * i / (BOUYANCY_SECTIONS-1));
    maths::Vector<3, float> midPos = GetLocalPosition( maths::Vector<3, float>(0.0f, y, -BOAT_HEIGHT*0.5f));
    maths::Vector<3, float> leftPos = GetLocalPosition( maths::Vector<3, float>(-0.8f*BOAT_HALF_WIDTH, y, 0.0f));
    maths::Vector<3, float> rightPos = GetLocalPosition( maths::Vector<3, float>( 0.8f*BOAT_HALF_WIDTH, y, 0.0f));
    float midDiff = (heightField->GetAltitude(maths::Vector<2, float>(midPos)) - midPos[2]) / (BOAT_HEIGHT);
    float leftDiff = (heightField->GetAltitude(maths::Vector<2, float>(leftPos)) - leftPos[2]) / (BOAT_HEIGHT*0.5f);
    float rightDiff = (heightField->GetAltitude(maths::Vector<2, float>(rightPos)) - rightPos[2]) / (BOAT_HEIGHT*0.5f);
    float midDiffBelow = std::max(midDiff, 0.0f);
    float leftDiffBelow = std::max(leftDiff, 0.0f);
    float rightDiffBelow = std::max(rightDiff, 0.0f);
    float midDiffAbove = std::max(1.0f-midDiff, 0.0f);
    float leftDiffAbove = std::max(1.0f-leftDiff, 0.0f);
    float rightDiffAbove = std::max(1.0f-rightDiff, 0.0f);
    ApplyForceAt(gravitationalFieldStrength * ((_density - densityBeneath * midDiffBelow   - densityAbove * midDiffAbove  ) * sectionVolume * 0.5f), midPos);
    ApplyForceAt(gravitationalFieldStrength * ((_density - densityBeneath * leftDiffBelow  - densityAbove * leftDiffAbove ) * sectionVolume * 0.25f), leftPos);
    ApplyForceAt(gravitationalFieldStrength * ((_density - densityBeneath * rightDiffBelow - densityAbove * rightDiffAbove) * sectionVolume * 0.25f), rightPos);
  }
}

/// The boat has moved.
void Boat::VNewPosition(const maths::Vector<3, float> & vOldPos,const maths::Vector<3, float> & vNewPos)
{
  Object::VNewPosition(vOldPos, vNewPos);
  cranes[0].SetPosition(GetLocalPosition(maths::Vector<3, float>( 7.0f, 4.0f, BOAT_HEIGHT*0.5f)));
  cranes[0].SetOrientation(_orientation);
  cranes[1].SetPosition(GetLocalPosition(maths::Vector<3, float>(-7.0f, 4.0f, BOAT_HEIGHT*0.5f)));
  cranes[1].SetOrientation(_orientation);
}

/// Advance the boat.
void Boat::VAdvance(float dt)
{
  for (int i = 0; i < 2; ++i) {
    Object::s_defaultEnvironment->AdvanceObject(&cranes[0]);
    Object::s_defaultEnvironment->ApplyConstraintsToObject(&cranes[0], dt);
  }
}

/// Render the boat.
void Boat::VRender(const Observer & observer)
{
  //Drawing::Torus(8.0f, 2.0f, 32, 32);
  
  // Main hull
  RenderHull();
}

/// Render the hull of the boat.
void Boat::RenderHull()
{
  glPushMatrix();
  {
    glTranslatef(0.0f, 0.0f, -0.5f * BOAT_HEIGHT);
    RenderSide();
    glScalef(-1.0f, 1.0f, 1.0f);
    glFrontFace(GL_CW);
    RenderSide();
    glFrontFace(GL_CCW);
  }
  glPopMatrix();
}

/// Render one side of the boat.
void Boat::RenderSide()
{
#define HEIGHT_SECTIONS 10
#define FRONT_SECTIONS 10
#define REAR_SECTIONS 10
  
  // Metal
  {
    static float diffuse[4]  = {0.2f, 0.2f, 0.2f, 1.0f};
    static float specular[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
  }
  
  float prevFactor = 1.0f;
  float prevXScale = 0.0f;
  bool top = false;
  // bottom to top
  for (int i = 1; !top; ++i) {
    if (i == HEIGHT_SECTIONS+1) {
      top = true;
      i = HEIGHT_SECTIONS;
      glNormal3f(0.0f, 0.0f, 1.0f);
    }
    float factor = 1.0f - (float)i/HEIGHT_SECTIONS;
    float xScale = 1.0f - factor*factor;
    if (top) {
      xScale = 0.0f;
    }
    
    glBegin(GL_TRIANGLE_STRIP);
    {
      int j;
      for (j = 0; j <= FRONT_SECTIONS; ++j) {
        float frontY = BOAT_FRONT_LENGTH - BOAT_FRONT_LENGTH*j/FRONT_SECTIONS;
        if (!top) {
          /// @todo Correct front normal.
          glNormal3f(1.0f, 0.0f, 0.0f);
        }
        glVertex3f(BOAT_HALF_WIDTH*prevXScale * sin(PI*j/(FRONT_SECTIONS*2)),
                   BOAT_MIDDLE_HALF_LENGTH + frontY - frontY * FRONT_BOTTOM_FACTOR * (1.0f-prevXScale),
                   BOAT_HEIGHT * (1.0f-prevFactor));
        glVertex3f(BOAT_HALF_WIDTH*xScale * sin(PI*j/(FRONT_SECTIONS*2)),
                   BOAT_MIDDLE_HALF_LENGTH + frontY - frontY * FRONT_BOTTOM_FACTOR * (1.0f-xScale),
                   BOAT_HEIGHT * i / HEIGHT_SECTIONS);
      }
      for (j = 0; j <= REAR_SECTIONS; ++j) {
        float ang = PI * j / (REAR_SECTIONS*2);
        float sinAng = sin(ang);
        float cosAng = cos(ang);
        if (!top) {
          /// @todo Correct rear normals to include downward curvature.
          glNormal3f(cosAng, -sinAng, 0.0f);
        }
        glVertex3f(BOAT_HALF_WIDTH*prevXScale * cosAng,
                   -BOAT_MIDDLE_HALF_LENGTH - BOAT_REAR_LENGTH * sinAng * prevXScale,
                   BOAT_HEIGHT * (1.0f-prevFactor));
        glVertex3f(BOAT_HALF_WIDTH*xScale * cosAng,
                   -BOAT_MIDDLE_HALF_LENGTH - BOAT_REAR_LENGTH * sinAng * xScale,
                   BOAT_HEIGHT * i / HEIGHT_SECTIONS);
      }
    }
    glEnd();
    prevFactor = factor;
    prevXScale = xScale;
  }
}

