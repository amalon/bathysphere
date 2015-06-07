/*
 * rigidbody.h
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
 * Object simulated with rigid body dynamics.
 *
 */

#ifndef _FILE_rigidbody_h
#define _FILE_rigidbody_h

#include "orientableobject.h"

namespace maths {
  /// Calculate an Inertia Tensor matrix for a uniform cuboid
  Matrix<3,float> InertiaTensorCuboid(float mass, float x, float y, float z);
  /// Calculate an Inertia Tensor matrix for a uniform cube
  Matrix<3,float> InertiaTensorCube(float mass, float x);
  /// Calculate an Inertia Tensor matrix for a uniform sphere
  Matrix<3,float> InertiaTensorSphere(float mass, float radius);
}

/// Object with more realistic rigid body properties.
class RigidBody : public OrientableObject
{
  protected:
    // Constant properties
    /// Inertia tensor.
    maths::Matrix<3, float> _bodyTensor;
    /// Inverted inertial tensor.
    maths::Matrix<3, float> _bodyTensorInverse; 

    // State quantities
    /// Angular momentum vector.
    maths::Vector<3, float> _angularMomentum;

    // Derived quantities
    /// Current inverted initia tensor depending on orientation.
    maths::Matrix<3, float> _currentTensorInverse;
    /// Angular velocity vector.
    maths::Vector<3, float> _angularVelocity;

    // Calculated quantities
    /// Torque on this object.
    maths::Vector<3, float> _torque;

  public:
    /// Default constructor
    RigidBody();
    
    
    /// Advance the object using the forces.
    virtual void Advance(float clock);
    virtual float VDetectRigidCollision(Collision::Interaction & result,
                                        const maths::Vector<3, float> & prepos, const maths::Vector<3, float> & postpos,
                                        const Physics::Orientation & preori, const Physics::Orientation & postori)
    {
      return NO_COLLISION;
    }
    
    
    /// Apply a force to the object.
    /**
     * @param force Force vector in scene coordinate system.
     * @param applied Application position vector in scene coordinate system.
     */
    virtual void ApplyForceAt(const maths::Vector<3, float> & force, const maths::Vector<3, float> & applied);
 
    /// Apply an impulse to the object.
    /**
     * @param impulse Impulse vector in scene coordinate system.
     * @param applied Application position vector in scene coordinate system.
     */
    virtual void ApplyImpulseAt(const maths::Vector<3, float> & impulse, const maths::Vector<3, float> & applied);
 
    /// Set the inertial tensor.
    void SetTensor(const maths::Matrix<3, float> & newTensor);
    // Get the inertial tensor
    const maths::Matrix<3, float> & GetTensor() const
    {
      return _bodyTensor;
    }
    const maths::Matrix<3, float> & GetTensorInv() const
    {
      return _bodyTensorInverse;
    }
    const maths::Matrix<3, float> & GetRotatedTensorInv() const
    {
      return _currentTensorInverse;
    }

    // Get angular velocities
    const maths::Vector<3, float> & GetAngularVelocity() const
    {
      return _angularVelocity;
    }
    const maths::Vector<3, float> & GetAngularMomentum() const
    {
      return _angularMomentum;
    }
    
    /// Get the velocity of the object at a particular location (taking angular velocity in to account).
    maths::Vector<3, float> GetVelocityAt(const maths::Vector<3, float> location) const
    {
      return GetVelocity() + maths::cross(_angularVelocity, location-GetPosition());
    }
    
    /// Get the velocity of the object at a particular location (taking angular velocity in to account).
    maths::Vector<3, float> GetMomentumAt(const maths::Vector<3, float> location) const
    {
      return GetMomentum() + maths::cross(_angularMomentum, location-GetPosition());
    }
    
    //maths::Vector<3, float> GetStopImpulseAt
    
  protected:
    
    /// Detect collisions with a sphere which is local to the rigid body.
    float DetectCollisionSphere(Collision::Interaction & result,
                                const std::list<Collision::Triangle> & triangles,
                                const maths::Vector<3, float> & prepos, const maths::Vector<3, float> & postpos,
                                const Physics::Orientation & preori, const Physics::Orientation & postori,
                                const maths::Vector<3, float> spherePosition, float sphereRadius);

};

#endif // _FILE_rigidbody_h
