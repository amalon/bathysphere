/*
 * crane.h
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
 * A general purpose crane object.
 *
 */

#ifndef _FILE_crane_h
#define _FILE_crane_h

#include "cord.h"
#include "orientableobject.h"
#include <list>

/// Crane class.
class Crane : public OrientableObject
{
  public:
    /// Represents an action that the crane can perform.
    class Action
    {
      public:
        /// Destructor.
        inline virtual ~Action() {}
        
        /// Advance the action.
        /**
         * @param dt Time passed since last advance.
         * @return The amount of time used up. If < @a dt the action is over.
         */
        virtual float Advance(Crane * crane, float dt) = 0;
    };
    
    /// Wait action.
    class ActionWait : public Action
    {
      protected:
        /// Time left to wait.
        float _timeout;
        
      public:
        /// Constructor
        inline ActionWait(float timeout)  : _timeout(timeout)
        {
        }
        
        /// Advance the action.
        virtual float Advance(Crane * crane, float dt);
    };
    
    /// Rotate action
    class ActionRotate : public Action
    {
      protected:
        /// Destination rotation.
        float _destinationRotation;
        /// Destination elevation.
        float _destinationElevation;
        
      public:
        /// Constructor
        inline ActionRotate(float rotation, float elevation) : _destinationRotation(rotation), _destinationElevation(elevation)
        {
        }
        
        /// Advance the action
        virtual float Advance(Crane * crane, float dt);
    };
    
    /// Winding action
    class ActionWind : public Action
    {
      protected:
        /// Destination rotation.
        float _destinationWind;
        
        /// Speed of wind.
        float _currentSpeed;
        
      public:
        /// Constructor
        inline ActionWind(float wind) : _destinationWind(wind), _currentSpeed(0.0f)
        {
        }
        
        /// Advance the action
        virtual float Advance(Crane * crane, float dt);
    };
    
    /// Set a value action.
    template <typename T>
    class ActionSet : public Action
    {
      protected:
        /// Destination.
        T & _destination;
        /// Value.
        T _value;
        
      public:
        /// Constructor
        inline ActionSet(T & destination, T value) : _destination(destination), _value(value)
        {
        }
        
        /// Advance the action
        virtual float Advance(Crane * crane, float dt)
        {
          _destination = _value;
          return 0.0f;
        }
    };
    
  protected:
    // Constants
    /// Tether object.
    Cord * _tether;
    /// The height of the base of the crane.
    float _height;
    /// The length of the arm of the crane.
    float _armLength;
    /// Max tether length.
    float _maxTetherLength;
    
    // Variables
    /// Angle of the crane about Z axis.
    float _rotation;
    /// Elevation of the crane.
    float _elevation;
    /// Winding length.
    float _windLength;
    /// Winding speed.
    float _windSpeed;
    
    /// Queue of actions
    std::list<Action *> _actionQueue;
    
  public:
    /// Constructor.
    Crane();
    
    /// Destructor.
    virtual ~Crane();
    
    /// Get the maximum rotation speed in rad/s.
    inline float GetMaxRotationSpeed() const
    {
      return 0.1f;
    }
    
    /// Get the maximum elevation speed in rad/s.
    inline float GetMaxElevationSpeed() const
    {
      return 0.1f;
    }
    
    /// Get the maximum winding speed in m/s.
    inline float GetMaxWindingSpeed() const
    {
      return 2.0f;
    }
    
    /// Get the maximum winding speed in m/s.
    inline float GetMaxWindingAcceleration() const
    {
      return 1.0f;
    }
    
    /// Get the rotation.
    float GetRotation() const
    {
      return _rotation;
    }
    /// Get the elevation.
    float GetElevation() const
    {
      return _elevation;
    }
    /// Get the wind length.
    float GetWindLength() const
    {
      return _windLength;
    }
    
    inline maths::Vector<3, float> GetArmTip() const
    {
      return maths::Vector<3, float>(_armLength*sin(_rotation)*cos(_elevation),
                                     -_armLength*cos(_rotation)*cos(_elevation),
                                     _height + _armLength*sin(_elevation));
    }
    
    /// Rotate towards a rotation.
    bool RotateTowards(float rotation, float dt);
    /// Elevate towards an elevation.
    bool ElevateTowards(float elevation, float dt);
    /// Wind towards a length.
    bool WindTowards(float length, float speed, float dt);
    
    /// Clear the action queue.
    void ClearActions();
    
    /// Add an action to the queue.
    void PushAction(Action * action);
    
    /// Set the new tether of the crane.
    void SetTether(Cord * tether);
    
    /// Get the tether of the crane.
    inline const Cord * GetTether() const
    {
      return _tether;
    }
    
  protected:
    /// The boat has moved.
    virtual void VNewPosition(const maths::Vector<3, float> & vOldPos,const maths::Vector<3, float> & vNewPos);
    /// Advance the crane.
    virtual void VAdvance(float dt);
    
    /// Render the crane.
    virtual void VRender(const Observer & observer);
};

#endif // _FILE_crane_h
