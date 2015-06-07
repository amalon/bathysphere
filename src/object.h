/*
 * object.h
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
 * Object class representing anything affected by the emvironment.
 *
 */

#ifndef _FILE_object_h
#define _FILE_object_h

#include "observer.h"
#include "maths/Vector.h"
#include "constraint.h"
#include "heightfield.h"
#include "collision/Collision.h"
#include "constants.h"
#include <list>
#include <set>

/// Draw a trail of the recent path of the object
//#define OBJECT_DEBUG_TRAILS

class Octree;
class OctreeCube;
class Environment;

/// Abstract base class representing any physical object.
class Object
{
  protected:
    /// Position of object in environment.
    maths::Vector<3, float> _position;
    maths::Vector<3, float> _velocity;
    float _density;
    float _volume;
    float _radius;
    
    /// Last time the object was updated.
    float _lastUpdated;
    
    /// Whether the object can collide.
    bool _collisions;
    /// Whether the object is moveable
    bool _frozen;
    /// Whether the object is visible.
    bool _visible;
    /// Whether to use special bouyancy calculations.
    bool _bouyancy;
    /// Whether the object has alpha blending.
    bool _alpha;
    /// Whether to delete the object.
    bool _delete;
    
    /// List of constraints.
    //std::list<Constraint*> _softConstraints;
    
    // Temporary
    maths::Vector<3, float> _forces;
    
  // Pointer to object
  //peDynamic * pObj;
  // Other octree information
  //unsigned char iCubeOverlaps;
    std::set<OctreeCube*> _octreeCubes;
    unsigned int _lastFrameId;
    static Environment * s_defaultEnvironment;
    
    // debug info
#ifdef OBJECT_DEBUG_TRAILS
    std::list<maths::Vector<3, float> > debug_trail;
#endif
    
  public:
    /// List of hard constraints
    std::list<HardConstraint*> hardConstraints;
    
  public:
    Object();
    virtual ~Object();
    
    /// Get the position of the object.
    inline const maths::Vector<3, float> & GetPosition() const
    {
      return _position;
    }
    
    /// Get the global position of a coordinate local to the object.
    inline virtual maths::Vector<3, float> GetLocalPosition(const maths::Vector<3, float> & localPosition) const
    {
      return _position + localPosition;
    }
    
    /// Get the velocity of the object.
    inline const maths::Vector<3, float> & GetVelocity() const
    {
      return _velocity;
    }
    
    /// Get the momentum of the object.
    inline const maths::Vector<3, float> GetMomentum() const
    {
      return _velocity * GetMass();
    }
    
    /// Get the volume of the object.
    inline float GetVolume() const
    {
      return _volume;
    }
    
    /// Function for doing special bouyancy calculations.
    virtual void CalculateBouyancy(const HeightField * heightField,
                                   float densityBeneath, float densityAbove,
                                   maths::Vector<3, float> gravitationalFieldStrength) {}
    
    /// Set the volume of the object.
    inline void SetVolume(float volume)
    {
      _volume = volume;
    }
    
    /// Get the density of the object.
    inline float GetDensity() const
    {
      return _density;
    }
    
    /// Set the density of the object.
    inline void SetDensity(float density)
    {
      _density = density;
    }
    
    inline float GetMass() const
    {
      return _density*_volume;
    }

    inline void SetPosition(const maths::Vector<3,float> & position)
    {
      maths::Vector<3, float> old = _position;
      _position = position;
      VNewPosition(old, position);
    }

    inline void SetLocalPosition(const maths::Vector<3,float> & position, const maths::Vector<3,float> & local)
    {
      maths::Vector<3, float> old = _position;
      _position = position - (GetLocalPosition(local)-GetPosition());
      VNewPosition(old, position);
    }

    inline void SetVelocity(const maths::Vector<3,float> & velocity)
    {
      _velocity = velocity;
    }
    
    /// Get the position of the object relative to a fixed location.
    inline maths::Vector<3, float> GetPosition(maths::Vector<3, float> relativeTo) const
    {
      return _position - relativeTo;
    }
    
    /// Get the current forces on the object.
    inline const maths::Vector<3, float> & GetForces() const
    {
      return _forces;
    }
    
    /// Get the position of the object.
    inline float GetMaxRadius() const
    {
      return _radius;
    }
    
    /// Get the radius of the object
    inline float GetRadius() const
    {
      return _radius;
    };
    
    /// Find whether the object needs deleting.
    inline bool IsDeleted() const
    {
      return _delete;
    }
    
    /// Find whether the object uses special bouyancy calculations.
    inline bool IsBouyant() const
    {
      return _bouyancy;
    }
    
    /// Find whether the object uses alpha blending.
    inline bool IsAlpha() const
    {
      return _alpha;
    }
    
    /// Flag the object for deletion.
    inline void Delete()
    {
      _delete = true;
    }
    
    /// Get cross sectional area x drag coefficient
    virtual float GetDrag(const maths::Vector<3, float> & direction) { return PI*_radius*_radius*0.5f; };
    
    /// Apply a force to the object.
    inline void ApplyForce(const maths::Vector<3, float> & force)
    {
      _forces += force;
    }
    
    /// Apply a force to the object.
    /**
     * @param force Force vector in scene coordinate system.
     * @param applied Application position vector in scene coordinate system.
     */
    inline virtual void ApplyForceAt(const maths::Vector<3, float> & force, const maths::Vector<3, float> & applied)
    {
      ApplyForce(force);
    }
    
    /// Apply an impulse to the object.
    /**
     * @param impulse Impulse vector.
     */
    inline void ApplyImpulse(const maths::Vector<3, float> & impulse)
    {
      _velocity += impulse / GetMass();
    }
    
    /// Apply an impulse to the object.
    /**
     * @param impulse Impulse vector in scene coordinate system.
     * @param applied Application position vector in scene coordinate system.
     */
    inline virtual void ApplyImpulseAt(const maths::Vector<3, float> & impulse, const maths::Vector<3, float> & applied)
    {
      ApplyImpulse(impulse);
    }
    
    /// Advance the object using the forces.
    virtual void Advance(float clock);
    
  // Accessors and mutators
    inline Object & SetLastFrameId(unsigned int iNewId)
    {
      _lastFrameId = iNewId;
      return *this;
    }
    inline unsigned int GetLastFrameId()
    {
      return _lastFrameId;
    }
    
    /// Set the default environment.
    static void SetDefaultEnvironment(Environment * environment);
    
    /// Get the default environment.
    static Environment * GetDefaultEnvironment();

    // Render the object (translating as appropriate then calling virtual render function)
    virtual void RenderObj(const Observer & observer);

  protected:
    virtual void VAdvance(float dt) {}
    virtual float VDetectCollision(Collision::Interaction & result,
                                   const maths::Vector<3, float> & prepos, const maths::Vector<3, float> & postpos)
    {
      return NO_COLLISION;
    }
    // Render the object
    virtual void VRender(const Observer & observer) { }
    virtual void VNewPosition(const maths::Vector<3, float> & vOldPos,const maths::Vector<3, float> & vNewPos);
    
    // Protected functions used by other om classes
    friend class OctreeCube;
    friend class Octree;
    friend class NaturalConstraint;

    inline unsigned char GetCubeRefs() const
    {
      return _octreeCubes.size();
    }
    
    friend class Environment;
    
  private:
    void UpdateCubeWithNewPosition(const maths::Vector<3, float> & vOldPos);
    // Main functions
    friend bool AddObjectToCube(OctreeCube * pCube, Object * pObj);   // Add an object and return whether successful
    friend bool RemoveObjectFromCube(OctreeCube * pCube, Object * pObj); // Remove an object and return whether successful
    //unsigned char AddCubeRef(OctreeCube * pCube);
    //unsigned char RemoveCubeRef(OctreeCube * pCube);
};

/// Wrapper class for having STL containers of distance sorted objects.
class DistanceSortedObject
{
  protected:
    /// The object.
    Object * _object;
    
  public:
    /// Constructor.
    inline DistanceSortedObject(Object * object) : _object(object)
    {
    }
    
    /// Dereference.
    inline Object * operator * ()
    {
      return _object;
    }
    /// Dereference.
    inline const Object * operator * () const
    {
      return _object;
    }
    /// Dereference.
    inline Object * operator -> ()
    {
      return _object;
    }
    /// Dereference.
    inline const Object * operator -> () const
    {
      return _object;
    }
    
    /// Equality.
    inline bool operator == (const Object * other) const
    {
      return _object == other;
    }
    /// Equality.
    inline bool operator == (const DistanceSortedObject & other) const
    {
      return _object == other._object;
    }
    /// Inequality.
    inline bool operator < (const DistanceSortedObject & other) const
    {
      return Observer::GetActiveObserver()->DistanceSquared(_object->GetPosition())
          >  Observer::GetActiveObserver()->DistanceSquared(other->GetPosition());
    }
};

#endif // _FILE_object_h
