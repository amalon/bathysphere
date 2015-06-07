/*
 * reef.h
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

#ifndef _FILE_reef_h
#define _FILE_reef_h

#include "object.h"
#include "coral.h"
#include "collision/Collision.h"

#include <GL/gl.h>
#include <list>

class ReefMesh;

/// Coral reef.
class Reef : public Object
{
  protected:
    /// Half size of the reef.
    float _halfSize;
    
    /// Main mesh of the reef
    ReefMesh * _mesh;
    
    /// OpenGL texture id.
    GLuint _texture;
    /// Size of the texture in metres.
    float _textureSize;
    
    /// Library of coral models.
    std::list<Coral::CoralModel*> _coralModels;
    
  public:
    /// Default constructor.
    Reef(float size);
    /// Destructor.
    virtual ~Reef();
    
    /// Build a library of coral models.
    void BuildCoralLibrary();
    
    /// Create a bunch of coral instances.
    void CreateCoralInstances(unsigned int perModel);
    
    /// Get the altitude at a particular location.
    float GetAltitude(const maths::Vector<2, float> & pos);
    
    /// Get a list of triangles which intersect a sphere.
    void GetIntersectingTriangles(std::list<Collision::Triangle> & results, const maths::Vector<3, float> & center, float radius);
    
    // Render the object (translating as appropriate then calling virtual render function)
    virtual void VRender(const Observer & observer);
};

#endif // _FILE_reef_h
