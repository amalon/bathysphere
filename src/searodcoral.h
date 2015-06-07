/*
 * searodcoral.h
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
 * Sea Rod Coral generater.
 *
 */

#ifndef _FILE_searodcoral_h
#define _FILE_searodcoral_h

#include "coral.h"

/// Sea rod coral
class SeaRodCoral : public Coral::CoralModel
{
  public:
    /// Constructor.
    SeaRodCoral(float size);
    
  protected:
    /// Construct and draw an arm of the coral
    void CoralConstruction(float radius,
                           maths::Vector<3, float> position,
                           Physics::Orientation orientation,
                           maths::Vector<3, float> angularChange,
                           int maxDepth, int maxLength);
};

#endif // _FILE_searodcoral_h
