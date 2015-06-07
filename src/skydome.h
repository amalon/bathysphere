/*
 * skydome.h
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
 * The sky and sun.
 *
 */

#ifndef _FILE_skydome_h
#define _FILE_skydome_h

#include "maths/Vector.h"
#include <GL/gl.h>

class Observer;

/// Skydome object.
class Skydome
{
  protected:
    /// Colour of the sky.
    maths::Vector<3, float> _skyColour;
    /// Colour at the horizon.
    maths::Vector<3, float> _horizonColour;
    
    /// GL texture id for sun texture.
    GLuint _sunTexture;
    
  public:
    /// Constructor
    Skydome();
    
    /// Get the OpenGL texture id for the sun texture.
    inline GLuint GetSunTexture() const
    {
      return _sunTexture;
    }
    
    /// Draw the sky dome
    void Render(const Observer & observer);
    
    /// Render the sun glare.
    void RenderSunGlare();
};

#endif // _FILE_skydome_h
