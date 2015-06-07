/*
 * environment.h
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
 * Scene management class.
 *
 */

#ifndef _FILE_environment_h
#define _FILE_environment_h

#include "object.h"
#include "octree.h"
#include "reef.h"
#include "seasurface.h"
#include "skydome.h"
#include <set>
#include <GL/gl.h>

class Environment
{
  protected:
    /// Standard soft constraints for simple objects.
    std::list<Constraint *> _standardSoftConstraints;
    /// Standard hard constraints for simple objects.
    //std::list<HardConstraint *> _standardHardConstraints;
    
    /// Frame counter.
    unsigned int _frameNumber;
    /// Time counter.
    float _clock;
    
    /// Last bubble creation time.
    float _lastBubbleClock;
    
    /// Set of all objects.
    std::set<Object*> _objects;
    /// Octree render options
    OctreeRenderOptions _octreeRenderOptions;
    
    /// Caustic textures.
    GLuint * _causticTextures;
    /// Number of caustic textures.
    unsigned int _numCausticTextures;
    /// Caustic transition speed in frames per second.
    float _causticSpeed;
    /// Current caustic frames.
    float _currentCausticFrame;
    
    /// Direction towards the sun
    maths::Vector<4, float> _sunDirection;
    
    /// Whether the camera is above the waters surface.
    bool _currentlyAboveWater;
    
  public:
    /// Object octree storing objects by position.
    Octree octree;
    
    /// The reef.
    Reef * reef;
    
    /// Surface of the sea.
    SeaSurface * seaSurface;
    
    /// The sky.
    Skydome skydome;
    
  public:
    /// Default constructor.
    Environment();
    /// Default destructor.
    ~Environment();
    
    /// Generate a set of kelp.
    void GenerateKelp(int num);
    
    /// Adds an object to the environment.
    void AddObject(Object * object);
    /// Removes an object from the environment.
    void RemoveObject(Object * object);
    
    /// Adds a constraint to the global constraints and takes responsibility for deletion.
    void AddSoftConstraint(Constraint * constraint);
    
    /// Get the current in the water.
    inline maths::Vector<3, float> GetCurrent(const maths::Vector<3, float> & position, float dt = 0.0f) const
    {
      return maths::Vector<3, float>(sin((_clock+dt)*2.1f + position[0]*0.2f + position[1]*0.2f + position[2]*0.2f),
                                     cos((_clock+dt)*2.1f + position[0]*0.2f + position[1]*0.2f + position[2]*0.2f),
                                     sin(1.0f+(_clock+dt)*2.1f + position[0]*0.2f + position[1]*0.2f + position[2]*0.2f));
    }
    /// Get the aproximate integrated current in the water.
    inline maths::Vector<3, float> GetIntegratedCurrent(const maths::Vector<3, float> & position, float dt = 0.0f) const
    {
      // integrate sin (a*t + c) dt [clock, clock+dt]
      // integrate sin (a*(t + c/a)) dt [clock, clock+dt]
      // integrate sin (a*t) dt [clock+c/a, clock+c/a+dt]
      return maths::Vector<3, float>( -cos((_clock+dt)*2.1f + position[0]*0.2f + position[1]*0.2f + position[2]*0.2f)/2.1f,
                                      sin((_clock+dt)*2.1f + position[0]*0.2f + position[1]*0.2f + position[2]*0.2f)/2.1f,
                                      -cos(1.0f+(_clock+dt)*2.1f + position[0]*0.2f + position[1]*0.2f + position[2]*0.2f)/2.1f);
    }
    
    /// Get the direction of the sun.
    inline maths::Vector<4, float> GetSunDirection() const
    {
      return _sunDirection;
    }
    
    /// Get the clock value.
    inline float GetClock() const
    {
      return _clock;
    }
    
    /// Find whether the camera is currently above water.
    inline bool GetAboveWater() const
    {
      return _currentlyAboveWater;
    }
    
    /// Set the octree render options.
    void SetRenderOptions(OctreeRenderOptions options);
    
    /// Advance the environment by @a dt seconds.
    void Advance(float dt);
    /// Apply the natural constraints to an object.
    void ApplyConstraintsToObject(Object * object, float dt);
    /// Advance an object to the current clock.
    void AdvanceObject(Object * object);
    
    /// Initialise the light sources.
    void InitLights();
    /// Position the lights in the scene.
    void PositionLights();
    /// Render the environment from the observer.
    void Render(Observer & observer);
    
    /// Enable caustic rendering.
    void BeginCaustics();
    /// Disable caustic rendering.
    void EndCaustics();
};

#endif // _FILE_environment_h
