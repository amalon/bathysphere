/*
 * rangeconstraint.h
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
 * A simple distance constraint between two objects.
 *
 */

#ifndef _FILE_rangeconstraint_h
#define _FILE_rangeconstraint_h

#include "constraint.h"

class Object;

/// A simple distance constraint between two objects.
class RangeConstraint : public HardConstraint
{
  protected:
    // Constant data members.
    /// The two objects which the constriant relates to.
    Object * _objects[2];
    /// The maximum distance that the objects can be from one another.
    float _maxDistance;
    
    // Varying data members.
    /// Direction from objects[0] to objects[1].
    maths::Vector<3, float> _direction;
    
    maths::Vector<3, float> _tempImpulses[2];
    
  public:
    // Management
    /// Constructor
    RangeConstraint(Object * first, Object * second, float maxDistance = 0.0f);
    
    /// Destructor
    virtual ~RangeConstraint();
    
    // Retrieval
    /// Get the distance from @a self to the other object involved in the constraint.
    maths::Vector<3, float> GetDirection(const Object * self);
    
    /// Get the other object.
    const Object * GetOther(const Object * self) const;
    
    // Update
    /// Indicate that an object has moved.
    /**
     * @param self The object that has moved.
     */
    virtual void ObjectUpdated(const Object * self);
    
    // Resolution
    /// Get the impulse required to move an object at a particular velocity and satisfy the constraint.
    /**
     * @param self The object which desires the velocity.
     * @param velocity The velocity desired by @a self.
     * @return The impulse required to achieve the velocity.
     */
    virtual maths::Vector<3, float> ImpulseRequired(const Object * self, const maths::Vector<3, float> & velocity);
    
    /// Apply an impulse through the constraint.
    /**
     * @param self The object applying the impulse.
     * @param impulseFactor The ammount of the required impulse that is able to be exerted as a ratio [0,1].
     * 
     * Applies a ratio of the impulses calculated from ImpulseRequired.
     */
    virtual void ApplyImpulse(Object * self, float impulseFactor);
    
    /// Attempt to get one of the objects moving at a particular velocity.
    /**
     * @param self The object attempting the velocity.
     * @param velocity The desired velocity of @a self.
     */
    virtual void AttemptVelocity(Object * self, const maths::Vector<3, float> & velocity);
    
  protected:
    // Internal
    /// Get the index of a given object.
    int GetObjectIndex(const Object * self) const;
};

#endif // _FILE_rangeconstraint_h
