/*
 * seabed.h
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
 * The bottom of the seabed.
 *
 */

#ifndef _FILE_seabed_h
#define _FILE_seabed_h

#include "object.h"

/// Absolute bottom seabed.
class Seabed : public Object
{
  protected:
    /// Half size of the reef.
    float _halfSize;
    
  public:
    /// Default constructor.
    Seabed(float size);
    
    // Render the object (translating as appropriate then calling virtual render function)
    virtual void VRender(const Observer & observer);
};

#endif // _FILE_seabed_h
