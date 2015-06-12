/*
 * maths/Observer.h
 *
 * Copyright (C) 2007-2015 James Hogan <james@albanarts.com>
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
 * Very basic abstract base class for observers.
 *
 * This is intended to simply store pure observer data.
 * The position therefore is relative to some origin which is outside of the
 * scope of the class.
 * An example of intended usage is for mesh.
 *
 */

#ifndef _MATHS_OBSERVER_H_
#define _MATHS_OBSERVER_H_

// Vectors used for position vectors.
#include "maths/Vector.h"

namespace maths
{
  
  /// Pure 3d observer data.
  /**
   * @param T Component type (greater accuracy may be desired for very large objects such as planets).
   */
  template <typename T>
  class Observer
  {
    protected:
      /// Position relative to some origin.
      maths::Vector<3,T> _position;
      /// Direction the observer is facing (zero for facing all directions).
      maths::Vector<3,T> _direction;
      
    public:
      /// Default constructor.
      inline Observer()
      {
        /// @note Position and Direction are left undefined.
      }
      
      /// Construct from position and direction vector.
      inline Observer(const maths::Vector<3,T> & position, const maths::Vector<3,T> & direction)
      : _position(position), _direction(direction)
      {
      }
      
      // Core mutators.
      
      /// Set the position of the observer.
      inline void setPosition(const maths::Vector<3,T> & position)
      {
        _position = position;
      }
      
      /// Set the direction the observer is facing.
      inline void setDirection(const maths::Vector<3,T> & direction)
      {
        _direction = direction;
      }
      
      // Core accessors.
      
      /// Get the position of the observer.
      inline const maths::Vector<3,T> & getPosition() const
      {
        return _position;
      }
      /// Get the position of the observer.
      inline maths::Vector<3,T> & getPosition()
      {
        return _position;
      }
      
      /// Get the direction the observer is facing.
      inline const maths::Vector<3,T> & getDirection() const
      {
        return _direction;
      }
      /// Get the direction the observer is facing.
      inline maths::Vector<3,T> & getDirection()
      {
        return _direction;
      }
      
      // Derived functions
      
      /// Find the squared distance to some position.
      inline T distanceSquared(const maths::Vector<3,T> & position) const
      {
        return (position - _position).sqr();
      }
      
      /// Find whether some face is front facing with respect to the observer.
      inline bool frontFacing(const maths::Vector<3,T> & position, const maths::Vector<3,T> & normal) const
      {
        maths::Vector<3,T> relativePosition = position - _position;
        return (relativePosition * _direction > (T)0.0 &&
                relativePosition * normal     < (T)0.0);
      }
  };
  
}

#endif // _MATHS_OBSERVER_H_
