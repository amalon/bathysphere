/*
 * bathysphere.cpp
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
 * Bathyphere object.
 *
 */

#include "bathysphere.h"
#include "bubble.h"
#include "environment.h"
#include "observer.h"
#include "primitives.h"
#include "glmaths.h"
#include "constants.h"
#include "debug.h"

#include <cstdlib>
#include <iostream>

/// Default constructor.
Bathysphere::Bathysphere()
  : _airVolume(0.0f),
    _hullDisplayList(0),
    _centerOfModel(0.0f, 0.0f, 0.5f)
{
  _radius = 1.8f;
  // density roughly twice that of water
  _density = 1500.0f;
  // approx 2.5 spheres of unit radius
  _volume = 2*PI;
  
  _collisions = true;
  _alpha = true;
  
  SetTensor(maths::InertiaTensorSphere(GetMass(), GetRadius()));
  
  // Generate an OpenGL display list for the bits of the bathysphere
  _hullDisplayList = glGenLists(1);
  
  if (_hullDisplayList) {
    glNewList(_hullDisplayList, GL_COMPILE);
    RenderHull();
    glEndList();
  }
  
  _orientation.ApplyRotation(PI/4, maths::Vector<3, float>(0.0f, 0.0f, 1.0f));
}

/// Destructor.
Bathysphere::~Bathysphere()
{
  // Delete the display lists.
  if (_hullDisplayList) {
    glDeleteLists(_hullDisplayList, 1);
  }
}

/// Square a value, for distance squared boundaries.
/**
 * @param X Some value.
 * @return @a X squared.
 */
#define BOUNDARY(X) ((X)*(X))

/// Boundary of distance squared benieth which an object is considered to be inside.
#define INSIDE_BOUNDARY     BOUNDARY(  1.0f )
/// Boundary of distance squared benieth which an object is considered to be inside.
#define CLOSE_BOUNDARY      BOUNDARY(  5.0f )
/// Boundary of distance squared benieth which an object is considered to be inside.
#define NEAR_BOUNDARY       BOUNDARY( 10.0f )

#define INDENTATION_FACTOR  0.9f
#define WINDENTATION_FACTOR  0.95f

void Bathysphere::VAdvance(float dt)
{
  // There's air being pumped in so its released when it builds up in the form
  // of a bubble.
  _airVolume += dt;
 // _airVolume += dt * 1e-3f*10;
  //if (_airVolume > 0.0f){//1e-3f) {
    // volume = 4/3*PI*r^3
    // r = (v*3/4/PI)^1/3
  //Bubble * bubble = new Bubble(cbrt(_airVolume*0.75f*PI));
  while (_airVolume > 1.0f) {
    Bubble * bubble = new Bubble(0.05f + 0.1f * rand() / RAND_MAX);
    bubble->SetVelocity(GetVelocity());
    bubble->SetPosition(GetLocalPosition(GetTetherHookPosition() + maths::Vector<3, float>(1.2f*cos(PI/4),0.0f,-1.0f + 1.2f*sin(PI/4))));
    Object::s_defaultEnvironment->AddObject(bubble);
    //std::cout << "bubbling" << std::endl;
    _airVolume = 0.0f;
  }

#if 0
    maths::Vector<3, float> pos = GetPosition();
    if (pos[2] < -150) {
      pos[2] = -150;
      SetPosition(pos);
    }
#endif
 
  _angularMomentum *= 1.0f-dt;
    
  //}
}

 float Bathysphere::VDetectRigidCollision(Collision::Interaction & result,
                                         const maths::Vector<3, float> & prepos, const maths::Vector<3, float> & postpos,
                                         const Physics::Orientation & preori, const Physics::Orientation & postori)
{
  Reef * reef = Object::s_defaultEnvironment->reef;
  // Check for colisions between bathysphere and land
  _collisionTriangles.clear();
  reef->GetIntersectingTriangles(_collisionTriangles, GetPosition(), GetRadius()+(postpos-prepos).mag());
  
  maths::Vector<3, float> relativePrePos = prepos - reef->GetPosition();
  maths::Vector<3, float> relativePostPos = postpos - reef->GetPosition();
  float factor NO_COLLISION;
  float fac;
  Collision::Interaction interaction;
#if 1
  // Lower sphere
  fac = DetectCollisionSphere(interaction,
                              _collisionTriangles,
                              relativePrePos, relativePostPos,
                                  preori, postori,
                                  _centerOfModel + maths::Vector<3, float>(0.0f, 0.0f, 0.0f), 1.0);
  if (fac != NO_COLLISION && (factor == NO_COLLISION || fac < factor)) {
    factor = fac;
    result = interaction;
  }
  // Upper sphere
  fac = DetectCollisionSphere(interaction,
                              _collisionTriangles,
                              relativePrePos, relativePostPos,
                                  preori, postori,
                                  _centerOfModel + maths::Vector<3, float>(0.0f, 0.0f, 0.75f), 1.0);
  if (fac != NO_COLLISION && (factor == NO_COLLISION || fac < factor)) {
    factor = fac;
    result = interaction;
  }
#endif
#if 1
    // The base
  
#define COLLISION_DETECTION_BASE_SPHERES1        96
#define COLLISION_DETECTION_RING_SPHERES_RADIUS1 ( 1.45f )
#define COLLISION_DETECTION_RING_SPHERES_Z1      ( -1.05f )
#define COLLISION_DETECTION_RING_SPHERES_RADII1  ( 0.05f )
  
#define COLLISION_DETECTION_BASE_SPHERES2        24
#define COLLISION_DETECTION_RING_SPHERES_RADIUS2 ( 1.0f )
#define COLLISION_DETECTION_RING_SPHERES_Z2      ( -0.8f )
#define COLLISION_DETECTION_RING_SPHERES_RADII2  ( 0.3f )
  
#define COLLISION_DETECTION_BASE_SPHERES3        16
#define COLLISION_DETECTION_RING_SPHERES_RADIUS3 ( 0.4f )
#define COLLISION_DETECTION_RING_SPHERES_Z3      ( -0.6f )
#define COLLISION_DETECTION_RING_SPHERES_RADII3  ( 0.5f )
  
  int i;
  for (i = 0; i < COLLISION_DETECTION_BASE_SPHERES1; ++i) {
    float ang = 0.1f + 2*PI*i/COLLISION_DETECTION_BASE_SPHERES1;
    float sinAng = sin(ang);
    float cosAng = cos(ang);
    fac = DetectCollisionSphere(interaction,
                                _collisionTriangles,
                                relativePrePos, relativePostPos,
                                preori, postori,
                                _centerOfModel + maths::Vector<3, float>(sinAng*COLLISION_DETECTION_RING_SPHERES_RADIUS1,
                                                        cosAng*COLLISION_DETECTION_RING_SPHERES_RADIUS1,
                                                            COLLISION_DETECTION_RING_SPHERES_Z1),
                                COLLISION_DETECTION_RING_SPHERES_RADII1);
    if (fac != NO_COLLISION && (factor == NO_COLLISION || fac < factor)) {
      factor = fac;
      result = interaction;
    }
  }
#ifdef COLLISION_DETECTION_RING_SPHERES_RADIUS2
  for (i = 0; i < COLLISION_DETECTION_BASE_SPHERES2; ++i) {
    float ang = 0.2f + 2*PI*i/COLLISION_DETECTION_BASE_SPHERES2;
    float sinAng = sin(ang);
    float cosAng = cos(ang);
    fac = DetectCollisionSphere(interaction,
                                _collisionTriangles,
                                relativePrePos, relativePostPos,
                                preori, postori,
                                _centerOfModel + maths::Vector<3, float>(sinAng*COLLISION_DETECTION_RING_SPHERES_RADIUS2,
                                    cosAng*COLLISION_DETECTION_RING_SPHERES_RADIUS2,
                                    COLLISION_DETECTION_RING_SPHERES_Z2),
                                    COLLISION_DETECTION_RING_SPHERES_RADII2);
    if (fac != NO_COLLISION && (factor == NO_COLLISION || fac < factor)) {
      factor = fac;
      result = interaction;
    }
  }
#endif
#ifdef COLLISION_DETECTION_RING_SPHERES_RADIUS3
  for (i = 0; i < COLLISION_DETECTION_BASE_SPHERES3; ++i) {
    float ang = 2*PI*i/COLLISION_DETECTION_BASE_SPHERES3;
    float sinAng = sin(ang);
    float cosAng = cos(ang);
    fac = DetectCollisionSphere(interaction,
                                _collisionTriangles,
                                relativePrePos, relativePostPos,
                                preori, postori,
                                _centerOfModel + maths::Vector<3, float>(sinAng*COLLISION_DETECTION_RING_SPHERES_RADIUS3,
                                    cosAng*COLLISION_DETECTION_RING_SPHERES_RADIUS3,
                                    COLLISION_DETECTION_RING_SPHERES_Z3),
                                    COLLISION_DETECTION_RING_SPHERES_RADII3);
    if (fac != NO_COLLISION && (factor == NO_COLLISION || fac < factor)) {
      factor = fac;
      result = interaction;
    }
  }
#endif
#endif
  
  if (factor != NO_COLLISION) {
    if (factor < 0.0f) {
      factor = 0.0f;
    }
    if (factor > 1.0f) {
      factor = 1.0f;
    }
    factor *= 0.99f;
    // Get location into scene space.
    result.location += reef->GetPosition();
  }
  
  return factor;
}

// Render the object (translating as appropriate then calling virtual render function)
void Bathysphere::VRender(const Observer & observer)
{
#if 0
  // For debuggin, when dont' want to see bathysphere.
  {
    static float diffuse[4]  = {1.0f, 1.0f, 1.0f, 1.0f};
    static float specular[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
  }
#ifdef 0
    // draw simple line axes
    glLineWidth(3);
    glBegin(GL_LINES);
    {
      glVertex3f(-1.0f, 0.0f, 0.0f);
      glVertex3f( 1.0f, 0.0f, 0.0f);
      glVertex3f(0.0f, -1.0f, 0.0f);
      glVertex3f(0.0f,  1.0f, 0.0f);
      glVertex3f(0.0f, 0.0f, -1.0f);
      glVertex3f(0.0f, 0.0f,  1.0f);
    }
    glEnd();
    glLineWidth(1);
  }
#endif
  
  float radius = GetRadius();
  glBegin(GL_LINE_LOOP);
  {
    for (int i = 0; i < 32; ++i) {
      float ang = 2*PI*i/32;
      glVertex3f(radius*sin(ang), radius*cos(ang), 0.0f);
    }
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  {
    for (int i = 0; i < 32; ++i) {
      float ang = 2*PI*i/32;
      glVertex3f(radius*sin(ang), 0.0f, radius*cos(ang));
    }
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  {
    for (int i = 0; i < 32; ++i) {
      float ang = 2*PI*i/32;
      glVertex3f(0.0f, radius*sin(ang), radius*cos(ang));
    }
  }
  glEnd();
  
#else
  
  // Call the display list for the main hull.
  if (_hullDisplayList) {
    glCallList(_hullDisplayList);
  }
  
#endif
  
  // The following in scene coordinates
  glPopMatrix();
  if (Debug::DebugObject::GetDebugRender()) {
    glPushMatrix();
    {
      //glDisable(GL_DEPTH_TEST);
      // The triangles which are close to the bathysphere
      Reef * reef = Object::s_defaultEnvironment->reef;
      glTranslate(reef->GetPosition());
      
      glDisable(GL_LIGHTING);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      
      glColor4f(0,1,0, 0.5f);
      glBegin(GL_TRIANGLES);
      for (std::list<Collision::Triangle>::iterator it = _collisionTriangles.begin(); it != _collisionTriangles.end(); ++it) {
        for (int i = 0; i < 3; ++i) {
          glVertex3((*it).vertices[i]);
        }
      }
      glEnd();
      
      glColor3f(1,0,0);
      glLineWidth(3);
      // Draw the triangles that are close by
      for (std::list<Collision::Triangle>::iterator it = _collisionTriangles.begin(); it != _collisionTriangles.end(); ++it) {
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 3; ++i) {
          glVertex3((*it).vertices[i]);
        }
        glEnd();
      }
      glLineWidth(1);
      
      glEnable(GL_LIGHTING);
      glDisable(GL_BLEND);
    }
    glPopMatrix();
    //glEnable(GL_DEPTH_TEST);
  }
  
#if 0
  // Draw center of top collision detection sphere
  maths::Vector<3, float> pos = GetPosition() + _orientation.GetMatrix() * maths::Vector<3, float>(0.0f, 0.0f, 0.75f);
  glPushMatrix();
  {
    glTranslate(pos);
    // draw simple line axes
    glLineWidth(3);
    glBegin(GL_LINES);
    {
      glVertex3f(-5.0f, 0.0f, 0.0f);
      glVertex3f( 5.0f, 0.0f, 0.0f);
      glVertex3f(0.0f, -5.0f, 0.0f);
      glVertex3f(0.0f,  5.0f, 0.0f);
      glVertex3f(0.0f, 0.0f, -5.0f);
      glVertex3f(0.0f, 0.0f,  5.0f);
    }
    glEnd();
    glLineWidth(1);
  }
  glPopMatrix();
#endif
  
#if 0
#define VELOCITY_SAMPLES 6
  static float samples[VELOCITY_SAMPLES][3] = {
    {-2.0f, 0.0f, 0.0f},
    { 2.0f, 0.0f, 0.0f},
    { 0.0f,-2.0f, 0.0f},
    { 0.0f, 2.0f, 0.0f},
    { 0.0f, 0.0f,-2.0f},
    { 0.0f, 0.0f, 2.0f},
  };
  // draw arrows around the bathysphere to show velocity (inc rotational)
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glLineWidth(3);
  glBegin(GL_LINES);
  {
    for (int sample = 0; sample < VELOCITY_SAMPLES; ++sample) {
      maths::Vector<3, float> pos(samples[sample]);
      pos += GetPosition();
      glColor3f(1,1,0);
      glVertex3(pos);
      glColor3f(1,0,0);
      glVertex3(pos + GetVelocityAt(pos));
    }
  }
  glEnd();
  glLineWidth(1);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
#endif
  
#ifdef OBJECT_DEBUG_TRAILS
  std::list<maths::Vector<3, float> >::const_iterator it;
  // Trail of objects position
  glDisable(GL_LIGHTING);
  
  glColor3f(0,1,0);
  glLineWidth(2.0f);
  glBegin(GL_LINE_STRIP);
  for (it = debug_trail.begin(); it != debug_trail.end(); ++it) {
    glVertex3(*it);
  }
  glEnd();
  glLineWidth(1.0f);
  
  glColor3f(1,0,0);
  glPointSize(5.0f);
  glBegin(GL_POINTS);
  for (it = debug_trail.begin(); it != debug_trail.end(); ++it) {
    glVertex3(*it);
  }
  glEnd();
  glPointSize(1.0f);
  
  glEnable(GL_LIGHTING);
#endif
  glPushMatrix();
}

/// Render the main hull.
void Bathysphere::RenderHull()
{
  glPushMatrix();
  {
    // The bathysphere is centered around the bottom of the windows section
    // in the middle
    // x,y,z is right, forward, upward respectively
    glTranslate(_centerOfModel);
    
    int segs = 72;
    
    // Metal
    {
      static float diffuse[4]  = {0.2f, 0.2f, 0.2f, 1.0f};
      static float specular[4] = {0.5f, 0.5f, 0.5f, 1.0f};
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
    }
    
    RenderBase(segs);
    // go to center of body
    glPushMatrix();
    {
#if 0
      // wobbles the capsule so its easier to see
      static float dt = 0.0f; dt += 0.1f;
      glRotatef(30.0f*sin(dt*0.1f), 1.0f, 0.0f, 0.0f);
      glRotatef(dt, 0.0f, 0.0f, 1.0f);
#endif
      glTranslatef(0.0f, 0.0f, 0.375f);
      // draw 4 window frames
      glPushMatrix();
      {
        RenderWindowFrame(segs, 1.0f, WINDENTATION_FACTOR, false);
        for (int i = 1; i < 4; ++i) {
          glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
          RenderWindowFrame(segs, 1.0f, WINDENTATION_FACTOR, false);
        }
      }
      glPopMatrix();
      
      glPushMatrix();
      {
        // drop the top and bottom
        RenderHemisphere(segs, 1.0f, false);
        glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
        RenderHemisphere(segs, 1.0f, false);
      }
      glPopMatrix();
      
      RenderInside(segs);
      
#if 1
      glPushMatrix();
      {
        // Set the window material
        {
          static float diffuse[4] = {0.2f, 0.2f, 0.2f, 0.2f};
          static float specular[4] = {0.8f, 0.8f, 0.8f, 1.0f};
          glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
          glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
          glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
        }
        
        // Enable blending
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // The inside windows need drawing first so that from the outside the
        // oposite windows can be seen through the close windows.
        
        // draw 4 windows from the inside
        glFrontFace(GL_CW);
        RenderWindow(segs, true);
        for (int i = 1; i < 4; ++i) {
          glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
          RenderWindow(segs, true);
        }
        
        // draw 4 windows from the outside
        glFrontFace(GL_CCW);
        RenderWindow(segs, false);
        for (int i = 1; i < 4; ++i) {
          glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
          RenderWindow(segs, false);
        }
        
        // Disable blending
        glDisable(GL_BLEND);
      }
      glPopMatrix();
#endif
    }
    glPopMatrix();
  }
  glPopMatrix();
}

void Bathysphere::RenderBase(int detail)
{
  // Do it properly as a cylinder
  Drawing::ThickDisk(1.45f, 0.05f, detail, -1.05f);
}

void Bathysphere::RenderHemisphere(int detail, float ringRadius, bool inside)
{
  // center of the hemisphere
  // a smaller value will result in more squashed ends
  // a larger value will result in more bulbus ends
  float zOffset = 0.375f;
  
  float radius = sqrt((0.375f-zOffset)*(0.375f-zOffset) + ringRadius*ringRadius);
  // angle between horizontal and the edge of the window section
  static const float startAngle = atan2(0.375f-zOffset, radius);
  int circularDetail = detail;
  detail = detail >> 1;
  
  float fInside = 1.0f;
  if (inside) {
    glFrontFace(GL_CW);
    fInside = -1.0f;
  }
  
  // Then the circular strips between the windows and the end
  float x = ringRadius;
  float z = 0.375f;
  float sin0 = sin(startAngle);
  float cos0 = cos(startAngle);
  for (int i = 1; i < detail; ++i) {
    float ang1 = startAngle + (0.5f*PI-startAngle)*i/detail;
    float sin1 = sin(ang1);
    float cos1 = cos(ang1);
    float nextX = radius*cos1;
    float nextZ = zOffset + radius*sin1;
    glBegin(GL_TRIANGLE_STRIP);
    for (int j = 0; j <= circularDetail; ++j) {
      float ang2 = 2.0f*PI*j/circularDetail;
      float sin2 = sin(ang2);
      float cos2 = cos(ang2);
      glNormal3f(sin2*cos0*fInside, cos2*cos0*fInside, sin0*fInside);
      glVertex3f(sin2*x, cos2*x, z);
      glNormal3f(sin2*cos1*fInside, cos2*cos1*fInside, sin1*fInside);
      glVertex3f(sin2*nextX, cos2*nextX, nextZ);
    }
    glEnd();
    x = nextX;
    z = nextZ;
    sin0 = sin1;
    cos0 = cos1;
  }
  
  // The fan at the bottom
  glBegin(GL_TRIANGLE_FAN);
  glNormal3f(0.0f, 0.0f, fInside);
  glVertex3f(0.0f, 0.0f, zOffset+radius);
  for (int j = 0; j <= circularDetail; ++j) {
    float ang2 = -2.0f*PI*j/circularDetail;
    float sin2 = sin(ang2);
    float cos2 = cos(ang2);
    glNormal3f(sin2*cos0*fInside, cos2*cos0*fInside, sin0*fInside);
    glVertex3f(sin2*x, cos2*x, z);
  }
  glEnd();
  
  if (inside) {
    glFrontFace(GL_CCW);
  }
}

void Bathysphere::RenderInside(int detail)
{
  // draw 4 window frames
  glPushMatrix();
  {
    RenderWindowFrame(detail, INDENTATION_FACTOR, WINDENTATION_FACTOR, true);
    for (int i = 1; i < 4; ++i) {
      glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
      RenderWindowFrame(detail, INDENTATION_FACTOR, WINDENTATION_FACTOR, true);
    }
  }
  glPopMatrix();
  
  // cieling and floor
  glPushMatrix();
  {
    RenderHemisphere(detail, INDENTATION_FACTOR, true);
    glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
    RenderHemisphere(detail, INDENTATION_FACTOR, true);
  }
  glPopMatrix();
}

void Bathysphere::RenderWindowFrame(int detail, float primaryIndentation, float secondaryIndentation, bool inside)
{
  int windowSep = detail/36;
  detail = (detail >> 2);
  
  // setup lookup table
  struct LookupRecord {
    float ang;
    float norm[2];
    float vec[2];
  } * lookup = new LookupRecord[detail+1];
  int i;
  float fInside = (inside ? -1.0f : 1.0f);
  for (i = 0; i <= detail; ++i) {
    lookup[i].ang = 0.5f*PI*i/detail;
    lookup[i].vec[0] = sin(lookup[i].ang);
    lookup[i].vec[1] = cos(lookup[i].ang);
    lookup[i].norm[0] = lookup[i].vec[0] * fInside;
    lookup[i].norm[1] = lookup[i].vec[1] * fInside;
  }
  
  if (inside) {
    glFrontFace(GL_CW);
  }
  
  // bottom outer rim
  glBegin(GL_TRIANGLE_STRIP);
  for (i = 0; i <= detail; ++i) {
    glNormal3f(lookup[i].norm[0], lookup[i].norm[1], 0.0f);
    glVertex3f(primaryIndentation*lookup[i].vec[0], primaryIndentation*lookup[i].vec[1], -0.375f);
    glVertex3f(primaryIndentation*lookup[i].vec[0], primaryIndentation*lookup[i].vec[1], -0.3f);
  }
  glEnd();
  
  // bottom window rim
  glBegin(GL_TRIANGLE_STRIP);
  for (i = windowSep; i <= detail-windowSep; ++i) {
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(primaryIndentation  *lookup[i].vec[0], primaryIndentation  *lookup[i].vec[1], -0.3f);
    glVertex3f(secondaryIndentation*lookup[i].vec[0], secondaryIndentation*lookup[i].vec[1], -0.3f);
  }
  glEnd();
  
  // side outer rim
  glBegin(GL_TRIANGLE_STRIP);
  {
    for (i = 0; i <= windowSep; ++i) {
      glNormal3f(lookup[i].norm[0], lookup[i].norm[1],  0.0f);
      glVertex3f(primaryIndentation*lookup[i].vec[0], primaryIndentation*lookup[i].vec[1], -0.3f);
      glVertex3f(primaryIndentation*lookup[i].vec[0], primaryIndentation*lookup[i].vec[1],  0.3f);
    }
  }
  glEnd();
  
  // side window rim
  glBegin(GL_TRIANGLE_STRIP);
  {
    glNormal3f(lookup[detail].vec[0], lookup[detail].vec[1],  0.0f);
    glVertex3f(primaryIndentation  *lookup[windowSep].vec[0], primaryIndentation  *lookup[windowSep].vec[1], -0.3f);
    glVertex3f(primaryIndentation  *lookup[windowSep].vec[0], primaryIndentation  *lookup[windowSep].vec[1],  0.3f);
    glVertex3f(secondaryIndentation*lookup[windowSep].vec[0], secondaryIndentation*lookup[windowSep].vec[1], -0.3f);
    glVertex3f(secondaryIndentation*lookup[windowSep].vec[0], secondaryIndentation*lookup[windowSep].vec[1],  0.3f);
  }
  glEnd();
  
  // other side window rim
  glBegin(GL_TRIANGLE_STRIP);
  {
    glNormal3f(lookup[0].vec[0], lookup[0].vec[1],  0.0f);
    glVertex3f(secondaryIndentation*lookup[detail-windowSep].vec[0], secondaryIndentation*lookup[detail-windowSep].vec[1], -0.3f);
    glVertex3f(secondaryIndentation*lookup[detail-windowSep].vec[0], secondaryIndentation*lookup[detail-windowSep].vec[1],  0.3f);
    glVertex3f(primaryIndentation  *lookup[detail-windowSep].vec[0], primaryIndentation  *lookup[detail-windowSep].vec[1], -0.3f);
    glVertex3f(primaryIndentation  *lookup[detail-windowSep].vec[0], primaryIndentation  *lookup[detail-windowSep].vec[1],  0.3f);
  }
  glEnd();
  
  // other side outer rim
  glBegin(GL_TRIANGLE_STRIP);
  {
    for (i = detail-windowSep; i <= detail; ++i) {
      glNormal3f(lookup[i].norm[0], lookup[i].norm[1],  0.0f);
      glVertex3f(primaryIndentation*lookup[i].vec[0], primaryIndentation*lookup[i].vec[1], -0.3f);
      glVertex3f(primaryIndentation*lookup[i].vec[0], primaryIndentation*lookup[i].vec[1],  0.3f);
    }
  }
  glEnd();
  
  // top window rim
  glBegin(GL_TRIANGLE_STRIP);
  for (i = windowSep; i <= detail-windowSep; ++i) {
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(secondaryIndentation*lookup[i].vec[0], secondaryIndentation*lookup[i].vec[1], 0.3f);
    glVertex3f(primaryIndentation  *lookup[i].vec[0], primaryIndentation  *lookup[i].vec[1], 0.3f);
  }
  glEnd();
  
  // top outer rim
  glBegin(GL_TRIANGLE_STRIP);
  for (i = 0; i <= detail; ++i) {
    glNormal3f(lookup[i].norm[0], lookup[i].norm[1], 0.0f);
    glVertex3f(primaryIndentation  *lookup[i].vec[0], primaryIndentation  *lookup[i].vec[1], 0.3f);
    glVertex3f(primaryIndentation  *lookup[i].vec[0], primaryIndentation  *lookup[i].vec[1], 0.375f);
  }
  glEnd();
  
  if (inside) {
    glFrontFace(GL_CCW);
  }
  
  delete [] lookup;
}

void Bathysphere::RenderWindow(int detail, bool inside)
{
  int windowSep = detail/36;
  detail = (detail >> 2);
  float indentation = WINDENTATION_FACTOR;
  float fInside = (inside ? -1.0f : 1.0f);
  
  glBegin(GL_TRIANGLE_STRIP);
  for (int i = windowSep; i <= detail-windowSep; ++i) {
    float ang = 0.5f*PI*i/detail;
    float normX = sin(ang);
    float normY = cos(ang);
    glNormal3f(normX*fInside, normY*fInside, 0.0f);
    glVertex3f(indentation*normX, indentation*normY, -0.3f);
    glVertex3f(indentation*normX, indentation*normY, 0.3f);
  }
  glEnd();
}
