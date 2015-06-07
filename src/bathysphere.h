/*
 * bathysphere.h
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

#ifndef _FILE_bathysphere_h
#define _FILE_bathysphere_h

#include "rigidbody.h"
#include "bubble.h"

/// @todo remove these, they're temporary
#include "collision/Collision.h"
#include <list>

/// Bathysphere world object.
class Bathysphere : public RigidBody
{
  protected:
    /// Volume of air about to be released.
    float _airVolume;
    
    /// ID of the OpenGL display list containing the main hull.
    GLuint _hullDisplayList;
    
    /// The position of the model center relative to the center of mass.
    maths::Vector<3, float> _centerOfModel;
    
    /// @todo Remove this, its temporary
    std::list<Collision::Triangle> _collisionTriangles;
    
  public:
    /// Default constructor.
    Bathysphere();
    /// Destructor.
    virtual ~Bathysphere();
    
    /// Get the local position of the tether hook.
    inline maths::Vector<3, float> GetTetherHookPosition() const
    {
      return _centerOfModel + maths::Vector<3, float>(0.0f, 0.0f, 1.75f);
    }
    
  protected:
    /// Defines the quality levels at which the bits of the bathysphere can be drawn at.
    enum Quality
    {
      /// For when the observer is far away.
      QualityLow    = 1,
      /// For when the observer is close enough to see detail.
      QualityMedium = 2,
      /// For when the observer is really close.
      QualityHigh   = 3
    };
    
    virtual void VAdvance(float dt);
    virtual float VDetectRigidCollision(Collision::Interaction & result,
                                        const maths::Vector<3, float> & prepos, const maths::Vector<3, float> & postpos,
                                        const Physics::Orientation & preori, const Physics::Orientation & postori);
    
    // Render the object (translating as appropriate then calling virtual render function)
    virtual void VRender(const Observer & observer);
    
    /// Render the main hull.
    /**
     * @note This does not worry about display lists.
     */
    void RenderHull();
    void RenderBase(int detail);
    void RenderHemisphere(int detail, float radius, bool inside);
    void RenderInside(int detail);
    void RenderWindowFrame(int detail, float primaryIndentation, float secondaryIndentation, bool inside);
    void RenderWindow(int detail, bool inside);
};

#endif // _FILE_bathysphere_h
