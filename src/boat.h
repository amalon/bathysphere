/*
 * boat.h
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

#ifndef _FILE_boat_h
#define _FILE_boat_h

#include "rigidbody.h"
#include "crane.h"

/// Boat class.
class Boat : public RigidBody
{
  public:
    /// Boat deck crane.
    class DeckCrane : public Crane
    {
      protected:
        /// The state of the crane.
        unsigned int _state;
        
      public:
        /// Constructor
        DeckCrane();
        
        /// Stop the crane.
        void Stop();
        /// Wind the crane in and move to a fixed position.
        void WindIn();
        /// Wind the crane to a particular length.
        void WindTo(float length);
    };
    
  public:
    /// Crane objects.
    DeckCrane cranes[2];
    
  public:
    /// Default constructor.
    Boat();
    
    /// Destructor.
    virtual ~Boat();
    
    /// Function for doing special bouyancy calculations.
    virtual void CalculateBouyancy(const HeightField * heightField,
                                   float densityBeneath, float densityAbove,
                                   maths::Vector<3, float> gravitationalFieldStrength);
    
  protected:
    /// The boat has moved.
    virtual void VNewPosition(const maths::Vector<3, float> & vOldPos,const maths::Vector<3, float> & vNewPos);
    /// Advance the boat.
    virtual void VAdvance(float dt);
    /// Render the boat.
    virtual void VRender(const Observer & observer);
    
    /// Render the hull of the boat.
    void RenderHull();
    /// Render one side of the boat.
    void RenderSide();
};

#endif // _FILE_boat_h
