/*
 * environment.cpp
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
 * Scene management class.
 *
 */

#include "environment.h"
#include "glmaths.h"
#include "naturalconstraint.h"
#include "kelp.h"
#include "dirt.h"
#include "bubble.h"
#include "texload.h"
#include "constants.h"

#include <cstdio>
#include <cstdlib>
#include <GL/glu.h>

#define CAUSTICS_NUM_TEXTURES 16
#define CAUSTICS_FILE_NAME DATA_DIRECTORY "caust%02d.bw"

#define LATITUDE (30.0f*PI/180)
#define TIME_OF_DAY (180.0f*PI/180)

Environment::Environment()
  : _frameNumber(0),
    _clock(0.0f),
    _lastBubbleClock(0.0f),
    _octreeRenderOptions(OctreeRenderDefault),
    _causticTextures(NULL),
    _numCausticTextures(0),
    _causticSpeed(1.0f), // frames per second
    _currentCausticFrame(0.0f), // frame
    _sunDirection(-sin(TIME_OF_DAY),
                  -cos(TIME_OF_DAY)*cos(LATITUDE),
                  -cos(TIME_OF_DAY)*cos(LATITUDE)),
    octree(200.0f)
{
  Object::SetDefaultEnvironment(this);
  
  // Set up caustics
  _numCausticTextures = CAUSTICS_NUM_TEXTURES;
  _causticTextures = new GLuint[_numCausticTextures];
  glGenTextures(_numCausticTextures, _causticTextures);
  char filename[20];
  int errors = 0;
  GLubyte *data;
  for (unsigned int i = 0; i < _numCausticTextures; ++i) {
    sprintf(filename, CAUSTICS_FILE_NAME, i*2);
    // load the file
    int width, height;
    data = read_alpha_texture(filename, &width, &height);
    if (data) {
      // bind the texture and send the data to opengl.
      glBindTexture(GL_TEXTURE_2D, _causticTextures[i]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      gluBuild2DMipmaps(GL_TEXTURE_2D, GL_ALPHA, width, height,
                        GL_ALPHA, GL_UNSIGNED_BYTE, data);
      free(data);
    } else {
      // problems loading
      printf("Error: couldn't load caustic texture %s\n", filename);
      ++errors;
    }
  }
  if (errors) {
    exit(1);
  }
  
  NaturalConstraint * naturalConstraint = new NaturalConstraint(this);
  // gravity
  maths::Vector<3, float> gravity(0.0f, 0.0f, -9.806f);
  naturalConstraint->SetGravitationalFieldStrength(gravity);
  // density and viscosity of water at sea level
  naturalConstraint->SetMediumDensity(1025.0f);
  naturalConstraint->SetMediumViscosity(8.9e-4f);
  // add the natural constraints
  AddSoftConstraint(naturalConstraint);
    
  // make some dirt
  for (int i = 0; i < 100; ++i) {
    float x = -25.0f + 50.0f * ((float)rand() / RAND_MAX);
    float y = -25.0f + 50.0f * ((float)rand() / RAND_MAX);
    float z = -100.0f + 100.0f * ((float)rand() / RAND_MAX) * ((float)rand() / RAND_MAX);
    Dirt * dirt = new Dirt();
    dirt->SetPosition(maths::Vector<3, float>(x,y,z));
    AddObject(dirt);
  }
  
  InitLights();
}

Environment::~Environment()
{
  std::list<Constraint *>::iterator itSoft = _standardSoftConstraints.begin();
  for (; itSoft != _standardSoftConstraints.end(); ++itSoft) {
    delete *itSoft;
  }
  
  if (_causticTextures) {
    glDeleteTextures(_numCausticTextures, _causticTextures);
    delete [] _causticTextures;
  }
  
  // Delete the objects
  std::set<Object*>::iterator it = _objects.begin();
  for (; it != _objects.end(); ++it) {
    delete *it;
  }
}

void Environment::GenerateKelp(int num)
{
  // Make some kelp
  for (int i = 0; i < num; ++i) {
    float x = -25.0f + 50.0f * ((float)rand() / RAND_MAX);
    float y = -25.0f + 50.0f * ((float)rand() / RAND_MAX);
    float z = reef->GetAltitude(maths::Vector<2, float>(x, y));
    float len = 10.0f + 10.0f * ((float)rand() / RAND_MAX);
    Kelp * kelp = new Kelp(len, 0.1f, maths::Vector<3, float>(x,y,z));
    kelp->SetBaseExtension(1.0f);
    AddObject(kelp);
  }
}
    
/// Adds an object to the environment.
void Environment::AddObject(Object * object)
{
  _objects.insert(object);
  octree.AddObject(object);
}

/// Removes an object from the environment.
void Environment::RemoveObject(Object * object)
{
  _objects.erase(object);
}
    
/// Adds a constraint to the global constraints and takes responsibility for deletion.
void Environment::AddSoftConstraint(Constraint * constraint)
{
  _standardSoftConstraints.push_back(constraint);
}

/// Apply the natural constraints to an object.
void Environment::ApplyConstraintsToObject(Object * object, float dt)
{
  object->VAdvance(dt);
  // Apply the main constraints
  std::list<Constraint *>::iterator cit;
  for (cit = _standardSoftConstraints.begin(); cit != _standardSoftConstraints.end(); ++cit) {
    (*cit)->EnforceConstraint(object, dt);
  }
}

/// Advance an object to the current clock.
void Environment::AdvanceObject(Object * object)
{
  // Resolve the forces
  object->Advance(_clock);
}

/// Set the octree render options.
void Environment::SetRenderOptions(OctreeRenderOptions options)
{
  _octreeRenderOptions = options;
}

/// Advance the environment by @a dt seconds.
void Environment::Advance(float dt)
{
  // Advance the caustic
  _currentCausticFrame = fmod(_currentCausticFrame + _causticSpeed * dt * (float)_numCausticTextures, (float)_numCausticTextures);
  
  for (int i = 0; i < 1; ++i) {
    float prevClock = _clock;
    
    // Advance the objects
    std::set<Object*>::iterator it = _objects.begin();
    for (; it != _objects.end(); ++it) {
      int its = 1;
      if ((*it)->GetDensity() < 100.0f) {
        its = 1;
      }
      float miniDt = dt/its;
      for (int j = 0; j < its; ++j) {
        _clock += miniDt;
        ApplyConstraintsToObject(*it, miniDt);
        AdvanceObject(*it);
      }
      _clock = prevClock;
    }
    _clock += dt;
  }
  std::set<Object*>::iterator it = _objects.begin();
  for (; it != _objects.end(); ++it) {
    if ((*it)->IsDeleted()) {
      std::set<Object*>::iterator it2 = it;
      --it;
      delete *it2;
      _objects.erase(it2);
    }
  }
  
  // Bubble every 1.0 seconds
  if (_clock - _lastBubbleClock > 1.0f) {
    int num = 1;
    for (int i = 0; i < num; ++i) {
      float x = -25.0f + 50.0f * rand() / RAND_MAX;
      float y = -25.0f + 50.0f * rand() / RAND_MAX;
      float z = -100.0f;
      float randomRadius = 0.02f + 0.08f * rand() / RAND_MAX;
      Bubble * bubble = new Bubble(randomRadius);
      bubble->SetPosition(maths::Vector<3, float>(x,y,z));
      Object::s_defaultEnvironment->AddObject(bubble);
    }
  }
}

void Environment::InitLights()
{
  GLfloat sun_amb_and_diff[] = {0.5, 1.0, 1.0, 1.0};
  GLfloat global_abm[] =  {0.0, 0.0, 0.0, 1.0};
  GLfloat specular[] =   {0.3, 0.3, 0.3, 1.0};

  glEnable(GL_LIGHTING);

  glLightfv(GL_LIGHT0, GL_AMBIENT, sun_amb_and_diff);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_amb_and_diff);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

  glLightfv(GL_LIGHT1, GL_AMBIENT, sun_amb_and_diff);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, sun_amb_and_diff);
  glLightfv(GL_LIGHT1, GL_SPECULAR, specular);

  glLightfv(GL_LIGHT2, GL_AMBIENT, sun_amb_and_diff);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, sun_amb_and_diff);
  glLightfv(GL_LIGHT2, GL_SPECULAR, specular);

  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_abm);

  glEnable(GL_LIGHT0);
  //glEnable(GL_LIGHT1);
  //glEnable(GL_LIGHT2);
}

void Environment::PositionLights()
{
  //GLfloat light1_position[]={-50.0, 30.0, -50.0, 1.0};
  //GLfloat light2_position[]={-50.0, 40.0,  50.0, 1.0};

  glLightfv(GL_LIGHT0, GL_POSITION, _sunDirection);
  //glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
  //glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
}


/// Render the environment from the observer.
void Environment::Render(Observer & observer)
{
  // Make sure certain opengl states are set correctly.
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  
  _currentlyAboveWater = observer.GetPosition()[2] > seaSurface->GetAltitude((maths::Vector<2, float>)observer.GetPosition());
  observer.ClearOcclusionPlanes();
  if (_currentlyAboveWater) {
    static float fogColour[] = {0.6f, 0.8f, 0.8f, 1.0f};
    static float skyColour[] = {0.4f, 0.5f, 1.0f, 1.0f};
    glClearColor (skyColour[0], skyColour[1], skyColour[2], skyColour[3]);
    glFogfv(GL_FOG_COLOR, fogColour);
    glFogf(GL_FOG_MODE, GL_EXP);
    glFogf(GL_FOG_START, 50.0f);
    glFogf(GL_FOG_END, 200.0f);
    glFogf(GL_FOG_DENSITY, 0.005f);
    // Add an occlusion plane at sea level so nothing below is drawn.
    observer.AddOcclusionPlane(maths::Vector<4, float>(0.0f, 0.0f, 1.0f,
                               seaSurface->GetPosition()[2] + seaSurface->GetMaxAmplitude()));
  } else {
    glEnable(GL_FOG);
    static float fogColour[] = {0.07f, 0.25f, 0.4f, 1.0f};
    glClearColor (fogColour[0], fogColour[1], fogColour[2], fogColour[3]);
    glFogfv(GL_FOG_COLOR, fogColour);
    glFogf(GL_FOG_MODE, GL_EXP);
    glFogf(GL_FOG_START, 50.0f);
    glFogf(GL_FOG_END, 200.0f);
    glFogf(GL_FOG_DENSITY, 0.02f);
    // Add an occlusion plane at sea level so nothing above is drawn.
    observer.AddOcclusionPlane(maths::Vector<4, float>(0.0f, 0.0f, -1.0f,
                               seaSurface->GetPosition()[2] + seaSurface->GetMaxAmplitude()));
  }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glEnable(GL_FOG);
  skydome.Render(observer);
  
  glEnable(GL_POINT_SMOOTH);
  
  PositionLights();
  
  octree.RenderFromCam(observer, ++_frameNumber, _octreeRenderOptions);
  
  if (false) {
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glBegin(GL_LINES);
    maths::Vector<3,float> pos;
    for (int i = 0; i < 10; ++i) {
      pos[0] = -25.0f+50.0f*i/10;
      for (int j = 0; j < 10; ++j) {
        pos[1] = -25.0f+50.0f*j/10;
        for (int k = 0; k < 10; ++k) {
          pos[2] = -100.0f*k/10;
          
          glColor3f(1,1,1);
          glVertex3(pos);
          glColor3f(1,0,0);
          glVertex3(pos + GetCurrent(pos));
        }
      }
    }
    glEnd();
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
  }
}


/// Enable caustic rendering.
/**
 * This function was originally for adding caustics to objects using alpha addition.
 * However for performance reasons it is no longer used for that, and now only
 * used for texturing the sea surface.
 */
void Environment::BeginCaustics()
{
  // Second pass for rendering caustics.
  // Don't update the depth buffer, and match the current depth buffer exactly
  //glDepthMask(GL_FALSE);
  //glDepthFunc(GL_EQUAL);

  // Multiply the source color (from the caustic luminance
  // texture) with the previous color from the normal pass.  The
  // caustics are modulated into the scene.
  //glBlendFunc(GL_ZERO, GL_SRC_COLOR);
  //glEnable(GL_BLEND);
  
  
  // Set current color to "white" and disable lighting
  // to emulate OpenGL 1.1's GL_REPLACE texture environment.
  //glColor3f(1.0, 1.0, 1.0);
  //glDisable(GL_LIGHTING);

  // Generate the S & T coordinates for the caustic textures
  // from the object coordinates.
  
  //static maths::Vector<4, float> sPlane;
  //static maths::Vector<4, float> tPlane;
  /*static bool initialised = false;
  if (!initialised) {
    maths::Vector<3, float> up(0.0f, 0.0f, 1.0f);
    maths::Vector<3, float> side;
    //maths::Cross(side, up, (maths::Vector<3,float>)_sunDirection
   // _sunDirection
  }*/
  /**
   * The following is adapted from
   * http://www.opengl.org/resources/code/samples/mjktips/caustics/
   */

  GLfloat sPlane[4] = { 0.1, 0.06, 0.0, 0.0 };
  GLfloat tPlane[4] = { 0.0, 0.03, 0.1, 0.0 };
  
  glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
  glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
  glTexGenfv(GL_S, GL_OBJECT_PLANE, sPlane);
  glTexGenfv(GL_T, GL_OBJECT_PLANE, tPlane);
  glEnable(GL_TEXTURE_GEN_S);
  glEnable(GL_TEXTURE_GEN_T);
  glEnable(GL_TEXTURE_2D);
  
  glBindTexture(GL_TEXTURE_2D, _causticTextures[(int)_currentCausticFrame]);
  
}

/// Disable caustic rendering.
/**
 * This function was originally for adding caustics to objects using alpha addition.
 * However for performance reasons it is no longer used for that, and now only
 * used for texturing the sea surface.
 */
void Environment::EndCaustics()
{
  glDisable(GL_TEXTURE_2D);
  
  //glEnable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_GEN_S);
  glDisable(GL_TEXTURE_GEN_T);

  // Restore fragment operations to normal.
  //glDisable(GL_BLEND);
  //glDepthMask(GL_TRUE);
  //glDepthFunc(GL_LESS);
}
