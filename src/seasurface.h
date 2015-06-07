/*
 * seasurface.h
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
 * The waters surface.
 *
 */

#ifndef _FILE_seasurface_h
#define _FILE_seasurface_h

#include "object.h"
#include "heightfield.h"
#include <GL/gl.h>

#define LIGHT_RAY_QUANTITY 64

/// The sea's surface.
class SeaSurface : public Object, public HeightField
{
  protected:
    /// Half size of the reef.
    float _halfSize;
    
    /// Random displacements of light rays.
    maths::Vector<3,float> _rayDisplacements[LIGHT_RAY_QUANTITY][LIGHT_RAY_QUANTITY];
    
    /// GL light of special sun reflection light.
    GLenum _sunReflectLight;
    
    /// Whether to draw the light rays shining through the surface.
    bool _drawSunRays;
    
  public:
    /// Default constructor.
    SeaSurface(float size);
    
    /// Get the altitude of the water surface in scene coordinate space.
    virtual float GetAltitude(const maths::Vector<2, float> & position) const;
    /// Get the maximum deviation from the position of the sea.
    inline float GetMaxAmplitude() { return 1.0f/2.1f; }
    
    // Render the object (translating as appropriate then calling virtual render function)
    virtual void VRender(const Observer & observer);
    /// Render the surface of the water.
    void RenderSurface(const Observer & observer);
    /// Render light rays shining through the surface of the water.
    void RenderSunRays(const Observer & observer);
    /// Render the sun glare.
    void RenderSunGlare(const Observer & observer);
    
    /// Set whether to draw sun rays.
    inline void SetDrawSunRays(bool drawSunRays)
    {
      _drawSunRays = drawSunRays;
    }
};

#endif // _FILE_seasurface_h
