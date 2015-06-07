/*
 * debug.h
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
 * Debug object which times out.
 *
 */

#ifndef _FILE_debug_h
#define _FILE_debug_h

#include "object.h"

namespace Debug {

  /// Simple renderable debug object which times out.
  class DebugObject : public Object
  {
    protected:
      /// Timeout.
      float _timeout;
      
      /// Whether to draw debug objects
      static bool _enableDebugRender;
      
    public:
      /// Main constructor.
      DebugObject(float timeout);
      
      /// Set whether to draw debug objects.
      inline static void SetDebugRender(bool render)
      {
        _enableDebugRender = render;
      }
      /// Find whether debug rendering is enabled.
      inline static bool GetDebugRender()
      {
        return _enableDebugRender;
      }
      
    protected:
      /// Advance the timeout.
      void VAdvance(float dt);
      
      /// Draw the debug object.
      void VRender(const Observer & observer)
      {
        if (_enableDebugRender) {
          VDebugRender(observer);
        }
      }
      
      /// Debug drawing functions for derivitive classes to implement.
      virtual void VDebugRender(const Observer & observer) {}
  };
  
  /// Draws a simple set of arrows for the axes.
  class Axes : public DebugObject
  {
    public:
      /// Main constructor.
      Axes(float size, float timeout);
      
    protected:
      /// Draw the axes
      void VDebugRender(const Observer & observer);
  };

}

#endif // _FILE_debug_h
