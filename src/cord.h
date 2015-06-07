/*
 * cord.h
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
 * A tether object.
 *
 */

#ifndef _FILE_cord_h
#define _FILE_cord_h

#include "simplestrand.h"

#include <GL/gl.h>

/// Metal cable.
class Cord : public SimpleStrand
{
  protected:
    /// Radius of the cord
    float _cordRadius;
    
    /// OpenGL texture id.
    GLuint _texture;
    /// Height of the texture on the cord.
    float _textureHeight;
    
  public:
    /// Default constructor.
    Cord(float length, const maths::Vector<3, float> & position);
    
    /// Destructor.
    ~Cord();
    
    /// Set up the material and texture.
    void SetupMaterial();
    
    /// Get the physical height of the texture.
    inline float GetTextureHeight()
    {
      return _textureHeight;
    }
  
  protected:
    /// Draw the strand using the derived classes functions.
    virtual void VRenderStrand(const Observer & observer);
};

#endif // _FILE_cord_h
