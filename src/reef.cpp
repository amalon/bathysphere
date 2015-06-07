/*
 * reef.cpp
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
 * Coral reef ground object.
 *
 */

#include "reef.h"
#include "environment.h"
#include "reefmesh.h"
#include "texload.h"
#include "glmaths.h"
#include "constants.h"
#include "searodcoral.h"

#include <cstdlib>
#include <cstdio>
#include <GL/glu.h>

/// Constructor
Reef::Reef(float size)
  : _halfSize(size/2),
    _textureSize(5.0f)
{
  _radius = size*Sqrt3;
  
  // Make the reef different each time
  _mesh = new ReefMesh(_halfSize, rand());
  
  // Load the coral texture
  const char * filename = DATA_DIRECTORY "coral.rgb";
  glGenTextures(1, &_texture);
  GLubyte *data;
  // load the file
  int width, height;
  data = read_rgb_texture(filename, &width, &height);
  if (data) {
    // bind the texture and send the data to opengl.
    glBindTexture(GL_TEXTURE_2D, _texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, width, height,
                      GL_RGB, GL_UNSIGNED_BYTE, data);
    free(data);
  } else {
    // problems loading
    printf("Error: couldn't load reef texture %s\n", filename);
    exit(1);
  }
}

/// Destructor.
Reef::~Reef()
{
  delete _mesh;
}

/// Build a library of coral models.
void Reef::BuildCoralLibrary()
{
  // Build a library of coral models.
  for (int i = 0; i < 20; ++i) {
    _coralModels.push_back(new SeaRodCoral(0));
  }
}

/// Create a bunch of coral instances.
void Reef::CreateCoralInstances(unsigned int perModel)
{
  // Add coral objects.
  // For each model add numCoral corals using it.
  std::list<Coral::CoralModel*>::iterator it;
  for (it = _coralModels.begin(); it != _coralModels.end(); ++it) {
    for (unsigned int i = 0; i < perModel; ++i) {
      Coral * newCoral = new Coral(*it);
      maths::Vector<2, float> randomPos(-25.0f + 50.0f * rand() / RAND_MAX,
                                         -25.0f + 50.0f * rand() / RAND_MAX);
      float z = GetAltitude(randomPos);
      newCoral->Orientation().ApplyRotation(2.0f*PI*rand()/RAND_MAX, maths::Vector<3, float>(0.0f, 0.0f, 1.0f));
      newCoral->SetPosition(maths::Vector<3, float>(randomPos[0], randomPos[1], z));
    }
  }
}

/// How deep to go into triangle tree when searching for intersections.
#define MESH_SEARCH_DEPTH 8

/// Get a list of triangles which intersect a sphere.
void Reef::GetIntersectingTriangles(std::list<Collision::Triangle> & results, const maths::Vector<3, float> & center, float radius)
{
#define INTERSECTING_TRIANGLES_RADIUS_FRACTION 0.4f
  maths::Vector<3, float> pos = center - GetPosition();
  _mesh->getIntersectingTriangles(results, pos, radius,
                                  MESH_SEARCH_DEPTH);
}

/// Get the altitude at a particular location.
float Reef::GetAltitude(const maths::Vector<2, float> & pos)
{
#define FUZZY_MAX_AMPLITUDE 30.0f
  /// @todo Make Reef::GetAltitude lots more efficient, didn't have time to do this properly.
  // Get the triangles near the point (There'll be lots of these, which is why it should be more efficient)
  std::list<Collision::Triangle> results;
  maths::Vector<3, float> center(pos - (maths::Vector<2, float>)GetPosition());
  _mesh->getIntersectingTriangles(results, center, FUZZY_MAX_AMPLITUDE, MESH_SEARCH_DEPTH);
  // do triangle - sphere path with 0 radius (line) collision detection.
  // also probably not the most efficent method, should do simple ray collision detection without checking for intersection with edges.
  Collision::Sphere sphere;
  sphere.radius = 0.0f;
  maths::Vector<3, float> start(center), motion(0.0f);
  start[2] += FUZZY_MAX_AMPLITUDE;
  motion[2] -= 2.0f*FUZZY_MAX_AMPLITUDE;
  std::list<Collision::Triangle>::iterator it;
  for (it = results.begin(); it != results.end(); ++it) {
    Collision::Interaction interaction;
  // This involves unecessary sqrt's :(
    float f = DetectTriangleSphere(interaction, *it, sphere, start, motion);
    if (f != NO_COLLISION) {
      return GetPosition()[2] + interaction.location[2];
    }
  }
  // erm, can't really tell, outside of range, just use 0
  return GetPosition()[2];
}

// Render the object (translating as appropriate then calling virtual render function)
void Reef::VRender(const Observer & observer)
{
  // Only draw when under water.
  bool above = Object::s_defaultEnvironment->GetAboveWater();
  if (above) {
    return;
  }
  {
    static float diffuse[4]  = {0.8f, 0.8f, 0.8f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    static float zero[4]  = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, zero);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
  }
  
  maths::Observer<float> obs(observer.GetPosition() - GetPosition(),
                             observer.GetDirection());
  
  /// @todo Adapt to observer as well as collider.
#if 1
  unsigned int ops = 100;
  _mesh->adaptToObserver(obs, &ops, MESH_SEARCH_DEPTH, false); 
#endif
  
  // Set up texture coordinate generation.
  GLfloat sPlane[4] = { 1.0f/_textureSize, 0.0f, 0.0f, 0.0f };
  GLfloat tPlane[4] = { 0.0f, 1.0f/_textureSize, 0.0f, 0.0f };
  
  glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
  glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
  glTexGenfv(GL_S, GL_OBJECT_PLANE, sPlane);
  glTexGenfv(GL_T, GL_OBJECT_PLANE, tPlane);
  glEnable(GL_TEXTURE_GEN_S);
  glEnable(GL_TEXTURE_GEN_T);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, _texture);
  
  // Draw the main mesh.
  _mesh->renderGL(&obs);
  
  // Disable texture coordinate generation etc.
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_GEN_S);
  glDisable(GL_TEXTURE_GEN_T);
}
