/*
 * heightfield.h
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
 * Height field.
 *
 */

#ifndef _FILE_heightfield_h
#define _FILE_heightfield_h

/// Abstract base class for objects representing a 2d field of height values.
class HeightField
{
  public:
    /// Destructor
    virtual ~HeightField() {}
    
    /// Get the altitude of the water surface in scene coordinate space.
    virtual float GetAltitude(const maths::Vector<2, float> & position) const = 0;
};

#endif // _FILE_heightfield_h
