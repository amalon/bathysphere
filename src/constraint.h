/*
 * constraint.h
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
 * A constraint on an object, mostly unused.
 *
 */

#ifndef _FILE_constraint_h
#define _FILE_constraint_h

#include "maths/Vector.h"

class Object;
class Environment;

/// A soft constraint dealing with forces.
class Constraint
{
  public:
    virtual ~Constraint() {}
    virtual void EnforceConstraint(Object * object, float dt) = 0;
    /// Indicate that a force is being applied to the object
    virtual void Force(Object * object, maths::Vector<3, float> force) {}
};

/// A hard constraint dealing with impulses.
class HardConstraint
{
  public:
    // Management
    /// Default constructor
    inline HardConstraint() {}
    
    /// Destructor
    inline virtual ~HardConstraint() {}
    
    // Update
    /// Indicate that an object has moved.
    /**
     * @param self The object that has moved.
     */
    virtual void ObjectUpdated(const Object * self) {}
    
    // Resolution
    /// Get the impulse required to move an object at a particular velocity and satisfy the constraint.
    /**
     * @param self The object which desires the velocity.
     * @param velocity The velocity desired by @a self.
     * @return The impulse required to achieve the velocity.
     */
    virtual maths::Vector<3, float> ImpulseRequired(const Object * self, const maths::Vector<3, float> & velocity) = 0;
    
    /// Apply an impulse through the constraint.
    /**
     * @param self The object applying the impulse.
     * @param impulseFactor The ammount of the required impulse that is able to be exerted as a ratio [0,1].
     * 
     * Applies a ratio of the impulses calculated from ImpulseRequired.
     */
    virtual void ApplyImpulse(Object * self, float impulseFactor) = 0;
    
    /// Attempt to get one of the objects moving at a particular velocity.
    /**
     * @param self The object attempting the velocity.
     * @param velocity The desired velocity of @a self.
     */
    virtual void AttemptVelocity(Object * self, const maths::Vector<3, float> & velocity) = 0;
};

#endif // _FILE_constraint_h
