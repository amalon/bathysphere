/*
 * simplestrand.h
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
 * Strand with simplified physics.
 *
 */

#ifndef _FILE_simplestrand_h
#define _FILE_simplestrand_h

#include "strand.h"

/// A simple tether without extensive physical simulation.
class SimpleStrand : public Strand
{
  protected:
    /// Object that the strand is attached to.
    Object * _attachee;
    /// Position to attach to the object.
    maths::Vector<3, float> _attachPosition;
    
    /// Time that the strand has been slack.
    float _slackTime;
    
  public:
    /// Constructor.
    SimpleStrand(float length, unsigned int numControlPoints,
                 float crossSectionalArea, float density,
                 float rigidity, const maths::Vector<3, float> & position, const maths::Vector<3, float> & direction);
    
    /// Destructor.
    virtual ~SimpleStrand();
    
    /// Set the attachee object.
    void SetAttachee(Object * attachee, const maths::Vector<3, float> & position);
    
    /// Get the time for which the strand has been slack.
    inline float GetSlackTime() const
    {
      return _slackTime;
    }
    
  protected:
    /// Advance the strand.
    virtual void VAdvance(float dt);
};

#endif // _FILE_simplestrand_h
