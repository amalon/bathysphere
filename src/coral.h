/*
 * coral.h
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
 * A coral colony object.
 *
 */

#ifndef _FILE_coral_h
#define _FILE_coral_h

#include "orientableobject.h"

/// Sea rod coral
class Coral : public OrientableObject
{
  public:
    class CoralModel
    {
      protected:
        friend class Coral;
        
        /// The display list of the coral.
        GLuint _displayList;
        
        /// The number of references to this coral model.
        int _references;
        
      public:
        /// Constructor.
        CoralModel();
        
        /// Destructor.
        ~CoralModel();
        
        /// Render the display list.
        void Render(const Observer & observer);
        
        /// Remove reference.
        inline void RemoveReference()
        {
          if (--_references < 1) {
            delete this;
          }
        }
        
        /// Add Reference
        inline void AddReference()
        {
          ++_references;
        }
        
      protected:
        /// Start compiling the display list.
        /**
         * @return Whether the display list exists.
         */
        bool BeginCompile();
        
        /// Finish compiling the display list.
        /**
         * @return Whether the display list exists.
         */
        bool EndCompile();
    };
    
  protected:
    /// The coral model to use.
    CoralModel * _model;
    
    /// Diffuse colour.
    maths::Vector<4, float> _diffuse;
    
    /// The coral texture id.
    static GLuint s_texture;
    /// References to the coral texture.
    static unsigned int s_textureRefs;
    
  public:
    /// Constructor.
    Coral(CoralModel * model);
    
    /// Destructor.
    ~Coral();
    
  protected:
    
    /// Render the coral model.
    virtual void VRender(const Observer & observer);
};

#endif // _FILE_coral_h
