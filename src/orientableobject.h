/*
 * orientableobject.h
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
 * An object with an orientation.
 *
 */

#ifndef _FILE_orientableobject_h
#define _FILE_orientableobject_h

#include "object.h"
#include "orientation.h"

/// An object with an orientation.
class OrientableObject : public Object
{
  protected:
    /// The orientation of the object.
    Physics::Orientation _orientation;
    
  public:
    /// Destructor
    inline virtual ~OrientableObject() {}
    
    /// Get the global position of a coordinate local to the object.
    virtual maths::Vector<3, float> GetLocalPosition(const maths::Vector<3, float> & localPosition) const;
    
    /// Set the orientation.
    inline void SetOrientation(Physics::Orientation orientation)
    {
      _orientation = orientation;
    }
    
    /// Get the orientation reference.
    inline Physics::Orientation & Orientation()
    {
      return _orientation;
    }
    
    /// Get the orientation reference.
    inline const Physics::Orientation & Orientation() const
    {
      return _orientation;
    }
    
    /// Render the object (translating as appropriate then calling virtual render function)
    virtual void RenderObj(const Observer & observer);
};

#endif // _FILE_orientableobject_h
