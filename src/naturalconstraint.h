/*
 * naturalconstraint.h
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
 * Constraint for applying weight, buoyancy, drag, currents.
 *
 */

#ifndef _FILE_naturalconstraint_h
#define _FILE_naturalconstraint_h

#include "constraint.h"
#include "maths/Vector.h"

class NaturalConstraint : public Constraint
{
  protected:
    /// Gravitational field strength measured in m/s^2.
    maths::Vector<3, float> _gravitationalFieldStrength;
    
    /// Density of the medium.
    float _mediumDensity;
    
    /// Viscosity of the medium.
    float _mediumViscosity;
    
    /// The environment of the natural constraints.
    Environment * _environment;
    
  public:
    /// Default constructor.
    NaturalConstraint(Environment * environment);
    
    /// Set the density of the medium.
    inline void SetMediumDensity(float density)
    {
      _mediumDensity = density;
    }
    
    /// Set the viscosity of the medium.
    inline void SetMediumViscosity(float viscosity)
    {
      _mediumViscosity = viscosity;
    }
    
    /// Set the acceleration due to gravity.
    inline void SetGravitationalFieldStrength(maths::Vector<3, float> fieldStrength)
    {
      _gravitationalFieldStrength = fieldStrength;
    }
    
  protected:
    void EnforceConstraint(Object * object, float dt);
};

#endif // _FILE_naturalconstraint_h
