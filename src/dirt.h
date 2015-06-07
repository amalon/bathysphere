/*
 * dirt.h
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
 * Dirt floating around in the water.
 *
 */

#ifndef _FILE_dirt_h
#define _FILE_dirt_h

#include "object.h"

class Dirt : public Object
{
  public:
    Dirt();
    
  protected:
    // Render the object
    virtual void VRender(const Observer & observer);
};

#endif // _FILE_dirt_h
