/*
 * rigidbody.cpp
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

#include "rigidbody.h"
#include "collisionresponse.h"
#include "environment.h"

// http://en.wikipedia.org/wiki/List_of_moment_of_inertia_tensors

/// Calculate an Inertia Tensor matrix for a uniform cuboid
maths::Matrix<3,float> maths::InertiaTensorCuboid(float mass, float x, float y, float z)
{
  float Mo12 = mass/12;
  Vector<3,float> vS(x*x,y*y,z*z);
  return Matrix<3,float>( Vector<3,float>(Mo12 * (vS[1] + vS[2]), 0, 0),
                          Vector<3,float>(0, Mo12 * (vS[0] + vS[2]), 0),
                          Vector<3,float>(0, 0, Mo12 * (vS[0] + vS[1])));
}

/// Calculate an Inertia Tensor matrix for a uniform cube
maths::Matrix<3,float> maths::InertiaTensorCube(float mass, float x)
{
  float a = 2*x*x*mass/12;
  return Matrix<3,float>( Vector<3,float>(a, 0, 0),
                          Vector<3,float>(0, a, 0),
                          Vector<3,float>(0, 0, a));
}

/// Calculate an Inertia Tensor matrix for a uniform sphere
maths::Matrix<3,float> maths::InertiaTensorSphere(float mass, float radius)
{
  float a = 0.4*mass*radius*radius;
  return Matrix<3,float>( Vector<3,float>(a, 0, 0),
                          Vector<3,float>(0, a, 0),
                          Vector<3,float>(0, 0, a));
}

/// Default constructor
RigidBody::RigidBody()
  : _bodyTensor(1.0f),
    _bodyTensorInverse(1.0f),
    _angularMomentum(0.0f),
    _currentTensorInverse(1.0f),
    _angularVelocity(0.0f),
    _torque(0.0f)
{
}

/// Apply a force to the object.
/**
 * @param force Force vector in scene coordinate system.
 * @param applied Application position vector in scene coordinate system.
 */
void RigidBody::ApplyForceAt(const maths::Vector<3, float> & force, const maths::Vector<3, float> & applied)
{
  Object::ApplyForce(force);
  // Add the torque, this is in the axis perpendicular to the force and the position of application.
  _torque += maths::cross((applied - GetPosition()), force);
}

/// Apply an impulse to the object.
/**
 * @param impulse Impulse vector in scene coordinate system.
 * @param applied Application position vector in scene coordinate system.
 */
void RigidBody::ApplyImpulseAt(const maths::Vector<3, float> & impulse, const maths::Vector<3, float> & applied)
{
  Object::ApplyImpulse(impulse);
  // Add the angular momentum directly, this is in the axis perpendicular to the force and the position of application.
  _angularVelocity += maths::cross((applied - GetPosition()), impulse);
  _angularVelocity = _currentTensorInverse * _angularMomentum;
}

/// Advance the object using the forces.
void RigidBody::Advance(float clock)
{
  float dt = clock - _lastUpdated;
  _lastUpdated = clock;
  if (_frozen) {
    return;
  }
    // Get the mass.
  float mass = GetMass();
  
  dt *= 1.0f;
  
  // If no mass, we can't really do much with the forces.
  if (dt > 0.0f) {
    VAdvance(dt);
    
    int iterations = 0;
    while (dt > 0.0f && mass && ++iterations < 5) {
      // Find the acceleration.
      // a = f/m
      maths::Vector<3, float> acceleration = _forces / GetMass();
      
      // Find the new velocity.
      // v = u + at
      maths::Vector<3, float> deltaVelocity = acceleration*dt;
      
      // Find the new displacement.
      // x = ut + 0.5at^2
      maths::Vector<3, float> displacement = (_velocity+deltaVelocity)*dt;
      
      // Find the new momentum
      // Apply the torque for this timestep to the angular momentum
      maths::Vector<3, float> angularMomentum = _angularMomentum + _torque*dt;
#if 1
      const float angularVelocityLimit = 0.5f * GetMass();
      // Artificially limit the angular momentum
      if (angularMomentum.sqr() > angularVelocityLimit*angularVelocityLimit) {
        angularMomentum.resize(angularVelocityLimit);
      }
#endif
      // Calculate the current angular velocity
      maths::Vector<3, float> angularVelocity = _currentTensorInverse * angularMomentum;
      
      _angularMomentum = angularMomentum;
      _angularVelocity = angularVelocity;
      
      // do the rigid body stuff now
      bool orientationChanged = !_angularMomentum.zero();
      Physics::Orientation orientation = _orientation;
      if (orientationChanged) {
        /**
         * The algorithm for stepping rigid body orientations is based loosely on the one
         * presented at http://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf.
         * The following few lines are base on material from that document.
         */
        // Rotate the object by the new angular momentum
        // the conversion vector to quaternion here produces a pure quaternion
        maths::Quaternion<float> Q = _orientation.GetQuaternion();
        Q += maths::Quaternion<float>(_angularVelocity*(0.5f*dt)) * Q;

        // Convert the quaternion into a rotation matrix
        orientation.SetQuaternion(Q.normalize());
      }
      
      Collision::Interaction interaction;
      float result = VDetectRigidCollision(interaction,
                                           _position, _position + displacement,
                                           _orientation, orientation);
      
      float untilCollision;
      if (result == NO_COLLISION) {
        untilCollision = 1.0f;
      } else {
        untilCollision = result;
        deltaVelocity *= result;
        displacement *= result;
      }
      
      SetVelocity(_velocity + deltaVelocity);
      SetPosition(_position + displacement);
      
      if (orientationChanged) {
        if (result != NO_COLLISION) {
          // Rotate the object by the new angular momentum
          // the conversion vector to quaternion here produces a pure quaternion
          maths::Quaternion<float> Q = _orientation.GetQuaternion();
          Q += maths::Quaternion<float>(_angularVelocity*(0.5f*dt*untilCollision)) * Q;
          // Convert the quaternion into a rotation matrix
          _orientation.SetQuaternion(Q.normalize());
        } else {
          _orientation = orientation;
        }
      
        /**
         * The algorithm for stepping rigid body orientations is based loosely on the one
         * presented at http://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf.
         * The following few lines are base on material from that document.
         */
        // Calculate the current inertial tensor
        _currentTensorInverse  = orientation.GetMatrix() * _bodyTensorInverse * _orientation.GetMatrix().transposed();
        // Calculate the current angular velocity
        _angularVelocity = _currentTensorInverse * _angularMomentum;

        //VNewOrientation(oldQ,Q);
      }
      
      dt -= untilCollision;
      
      if (result != NO_COLLISION && dt > 0.0f) {
        if (!(interaction.normal.sqr() > 0.9999f && interaction.normal.sqr() < 1.0001f)) {
          interaction.normal.normalize();
        }
        // counter any velocity towards the collision point
        float downwardVel = -(GetVelocityAt(interaction.location) * interaction.normal);
        if (downwardVel > 0.0f) {
          SetVelocity(_velocity + (interaction.normal * downwardVel));
        }
        
        // counter any forces towards the collision point
        float normalForce = -(_forces * interaction.normal);
        if (normalForce > 0.0f) {
          maths::Vector<3, float> forces = _forces;
          ApplyForceAt((interaction.normal * normalForce), interaction.location);
          
#if 0
          /// This doesn't seem to be working yet, and i didn't have time to
          /// make it work properly.
          // apply a wee bit of friction
          // friction = mu*normalForce
          float mu = 0.5f;
          float friction = normalForce * mu;
          //maths::Vector<3, float> tangentialMomentum = GetMomentumAt(interaction.location);
          maths::Vector<3, float> tangentialMomentum = GetMomentum();
          tangentialMomentum -= tangentialMomentum * (tangentialMomentum * interaction.normal);
          float maxImpulse = tangentialMomentum.mag();
          float maxFriction = maxImpulse / dt;
          if (friction > maxFriction) {
            friction = maxFriction;
          }
          //ApplyForceAt(tangentialMomentum * (-friction/maxImpulse), interaction.normal);
          ApplyForce(tangentialMomentum * (-friction/maxImpulse));
          
#endif
          
          // Draw debug reaction if applicable.
          Collision::Response * reaction = new Collision::Response(interaction.location, interaction.normal, _forces - forces);
          Object::s_defaultEnvironment->AddObject(reaction);
        }
        
        /// This doesn't work as it uses the object center:
        /*float downwardDisplacement = ((interaction.location - GetPosition()) * interaction.normal);
        if (downwardDisplacement > 0.0f) {
          SetPosition(_position + interaction.normal * downwardDisplacement);
          SetVelocity(_velocity + interaction.normal * downwardDisplacement / dt);
        }*/
      }
#ifdef OBJECT_DEBUG_TRAILS
      debug_trail.push_back(GetPosition());
#endif
    }
#ifdef OBJECT_DEBUG_TRAILS
    int count = debug_trail.size();
    while (count-- > 100) {
      debug_trail.pop_front();
    }
#endif
    
    // Reset the forces.
    _forces[0] = _forces[1] = _forces[2] = 0.0f;
    _torque[0] = _torque[1] = _torque[2] = 0.0f;
  }
}

/// Set the inertial tensor.
void RigidBody::SetTensor(const maths::Matrix<3, float> & newTensor)
{
  // Set the inertial tensor matrix and calculate & store its inverse
  _bodyTensor = newTensor;
  _bodyTensorInverse = newTensor.inverted();
  const maths::Matrix<3, float> & R = _orientation.GetMatrix();
  _currentTensorInverse = R * _bodyTensorInverse * R.transposed();
}

/// Detect collisions with a sphere which is local to the rigid body.
float RigidBody::DetectCollisionSphere(Collision::Interaction & result,
                                       const std::list<Collision::Triangle> & triangles,
                                       const maths::Vector<3, float> & prepos, const maths::Vector<3, float> & postpos,
                                       const Physics::Orientation & preori, const Physics::Orientation & postori,
                                       const maths::Vector<3, float> spherePosition, float sphereRadius)
{
  Collision::Sphere sphere;
  sphere.radius = sphereRadius;
  
  // Get the position of the sphere in global coordinates at the start and end.
  maths::Vector<3, float> preSpherePos  = prepos + preori.GetMatrix() * spherePosition;
  maths::Vector<3, float> postSpherePos = postpos + postori.GetMatrix() * spherePosition;
  
  float factor = NO_COLLISION;
  for (std::list<Collision::Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
    Collision::Interaction interaction;
    float newFactor = Collision::DetectTriangleSphere(interaction, *it, sphere, preSpherePos, postSpherePos-preSpherePos);
    if (newFactor != NO_COLLISION && (factor == NO_COLLISION || newFactor < factor)) {
      factor = newFactor;
      result = interaction;
    }
  }
  
  if (factor != NO_COLLISION) {
    //result.position 
  }
  
  return factor;
}
