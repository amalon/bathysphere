/*
 * bubble.h
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
 * Bubble object.
 *
 */

#ifndef _FILE_bubble_h
#define _FILE_bubble_h

#include "object.h"

class Bubble : public Object
{
  public:
    /// Constructor
    Bubble(float radius);
    
  protected:
    /// Render the bubble.
    virtual void VRender(const Observer & observer);
    /// Advance the bubble.
    virtual void VAdvance(float dt);
    
    
    /// Find whether the bubble has hit the water yet.
    float VDetectCollision(Collision::Interaction & result,
                           const maths::Vector<3, float> & prepos, const maths::Vector<3, float> & postpos);
    
    /// Get the stability of the bubble as a number in [0,1].
    float GetStability();
    /// Get the verticle squash factor due to vertical velocity.
    float GetSquash();
    /// Get the angle of the fringe of an unstable bubble.
    float GetFringeAngle(float stability);
    /// Get information about the fringe of an unstable bubble.
    void GetFringeCircle(float stability, float squash, float & z, float & radius);
};

#endif // _FILE_bubble_h
