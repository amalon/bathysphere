/*
 * observer.h
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
 * Camera class.
 *
 */

#ifndef _FILE_observer_h
#define _FILE_observer_h

#include <maths/Vector.h>
#include <maths/Plane.h>
#include "orientation.h"
#include <limits>

class Object;

#define OBSERVER_MAX_OCCLUSION_CULLING_PLANES 1

/// Basic observer class.
class Observer
{
  protected:
    /// OpenGL viewport information.
    struct { int x, y, w, h; } _viewport;
    /// Near cutting distance of OpenGL frustum
    float _nearClip;
    /// Far cutting distance of OpenGL frustum (for finite frustums)
    float _farClip;
    /// Standard Field of View in Radians
    float _fov;
    
    /// Position of observer.
    maths::Vector<3,float> _position;
    
    /// Object to lock position to.
    Object * _objectLock;
    /// Position relative to object lock.
    maths::Vector<3,float> _objectLockPosition;
    
    /// Occlusion culling planes.
    maths::Vector<4, float> _occlusionPlanes[OBSERVER_MAX_OCCLUSION_CULLING_PLANES];
    /// The number of occlusion culling planes in use.
    unsigned int _occlusionPlanesInUse;
    
    /// The current observer.
    static Observer * s_active;
    
  public:
    /// Orientation of observer.
    Physics::Orientation orientation;
    
    /// Default constructor
    Observer();
    
    /// Destructor
    ~Observer();
    
    /// Update the observer.
    void Advance(float dt);
    
    /// Set the OpenGL projection matrix.
    void SetProjection();
    /// Set OpenGL to use this observer.
    void SetModelView();
    
    inline void SetFov(float fov);
    inline float GetFov() const;
    inline void SetNearClip(float nearClip);
    inline float GetNearClip() const;
    inline void SetFarClip(float farClip);
    inline float GetFarClip() const;
    inline void SetViewport(int x, int y, int w, int h);
    inline maths::Vector<3,float> & GetPosition();
    inline const maths::Vector<3,float> & GetPosition() const;
    inline maths::Vector<3,float> GetDirection() const;
    inline void SetPosition(const maths::Vector<3,float> & position);
    /// Lock the observer to a position relative to an object.
    void LockToObject(Object * object, const maths::Vector<3,float> & position);
    /// Change the position of the observer relative to the lock object.
    inline void SetLockPosition(const maths::Vector<3,float> & position);
    /// Change the position of the observer relative to the lock object.
    inline const maths::Vector<3,float> & GetLockPosition();
    
    // Occlusion culling
    /// Clear the occlusion planes.
    inline void ClearOcclusionPlanes();
    /// Add an occlusion plane (+ve => visible).
    /**
     * @param plane Normalized plane equation.
     * @return Whether successfully added.
     */
    inline bool AddOcclusionPlane(const maths::Vector<4, float> & plane);
    /// Check whether the object should be culled from the scene or not.
    /**
     * @param position Position of the object to check.
     * @param radius Radius of the object to check.
     * @return A code to indicate how visible the object is.
     *  - -1: The sphere is definitely out of view, all smaller spheres within will be out.
     *  - 0: The sphere is partially out of view, some smaller spheres within may be out.
     *  - 1: The sphere is entirely in the view, all smaller spheres will be too.
     */
    inline int OcclusionCheck(const maths::Vector<3, float> & position, float radius) const;
    
    /// Generate a distance factor based on distance and direction, where closer is lower.
    inline float DistanceSquared(const maths::Vector<3,float>& position) const;
    /// Find whether a face is visible
    inline bool FrontFacing(const maths::Vector<3,float>& position,
                            const maths::Vector<3,float>& normal) const;
    
    
    inline float GetMeanResolution () const
    {
      return 0.5f*(_viewport.w + _viewport.h);
    }
    
    /// Set this observer as the active one.
    inline void SetActive()
    {
      s_active = this;
    }
    
    /// The the current active observer.
    inline static Observer * GetActiveObserver()
    {
      return s_active;
    }
    
};

inline float Observer::GetFov() const
{
  return _fov;
}

inline void Observer::SetFov(float fov)
{
  _fov = fov;
}

inline void Observer::SetNearClip(float nearClip)
{
  _nearClip = nearClip;
}
inline float Observer::GetNearClip() const
{
  return _nearClip;
}
inline void Observer::SetFarClip(float farClip)
{
  _farClip = farClip;
}
inline float Observer::GetFarClip() const
{
  return _farClip;
}

inline void Observer::SetViewport(int x, int y, int w, int h)
{
  _viewport.x = x;
  _viewport.y = y;
  _viewport.w = w;
  _viewport.h = h;
}

inline const maths::Vector<3,float> & Observer::GetPosition() const
{
  return _position;
}

inline maths::Vector<3,float> Observer::GetDirection() const
{
  return -orientation.GetMatrix()[2];
}

inline maths::Vector<3,float> & Observer::GetPosition()
{
  return _position;
}

inline void Observer::SetPosition(const maths::Vector<3,float> & position)
{
  _position = position;
}

/// Change the position of the observer relative to the lock object.
inline void Observer::SetLockPosition(const maths::Vector<3,float> & position)
{
  _objectLockPosition = position;
}

/// Change the position of the observer relative to the lock object.
inline const maths::Vector<3,float> & Observer::GetLockPosition()
{
  return _objectLockPosition;
}

/// Generate a distance factor based on distance and direction, where closer is lower.
inline float Observer::DistanceSquared(const maths::Vector<3,float>& position) const
{
  maths::Vector<3,float> relativePosition = position;
  relativePosition -= _position;
  return relativePosition.sqr();
}

/// Find whether a face is visible
inline bool Observer::FrontFacing(const maths::Vector<3,float>& position,
                                  const maths::Vector<3,float>& normal) const
{
  maths::Vector<3,float> relativePosition = position;
  relativePosition -= _position;
  return ((relativePosition * GetDirection() > 0.0f)   // in front
      &&  (relativePosition * normal     < 0.0f)); // facing us
}

// Occlusion culling functions
/// Clear the occlusion planes.
inline void Observer::ClearOcclusionPlanes()
{
  _occlusionPlanesInUse = 0;
}
/// Add an occlusion plane (+ve => visible).
inline bool Observer::AddOcclusionPlane(const maths::Vector<4, float> & plane)
{
  if (_occlusionPlanesInUse < OBSERVER_MAX_OCCLUSION_CULLING_PLANES) {
    _occlusionPlanes[_occlusionPlanesInUse++] = plane;
    return true;
  } else {
    return false;
  }
}

/// Check whether the object should be culled from the scene or not.
inline int Observer::OcclusionCheck(const maths::Vector<3, float> & position, float radius) const
{
  int result = 1;
  for (unsigned int i = 0; i < _occlusionPlanesInUse; ++i) {
    // if the object is definitely completely below the plane, cull it
    float distanceFromPlane = maths::plane(_occlusionPlanes[i], position);
    if (distanceFromPlane < -radius) {
      return -1;
    } else if (distanceFromPlane < radius) {
      result = 0;
    }
  }
  return result;
}

#endif // _FILE_observer_h
