/*
 * collisionresponse.h
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
 * Collision impact reaction force arrow object.
 *
 */

#ifndef _FILE_collisionresponse_h
#define _FILE_collisionresponse_h

#include "debug.h"
#include "orientation.h"

namespace Collision {
  
  /// Collision resopnse rendered, non dynamic, temporary object.
  class Response : public Debug::DebugObject
  {
    protected:
      /// Orientation.
      Physics::Orientation _orientation;
      
      /// Force applied as a result of the collision relative to orientation.
      maths::Vector<3, float> _force;
      
      
    public:
      /// Default constructor.
      Response(const maths::Vector<3, float> & pos, const maths::Vector<3, float> & normal, const maths::Vector<3, float> & force);
      
      /// Virtual destructor.
      virtual ~Response() {}
      
    protected:
      /// Draw a simple plane and normal vector.
      virtual void VDebugRender(const Observer & observer);
  };
  
}

#endif // _FILE_collisionresponse_h
