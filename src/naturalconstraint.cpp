/*
 * naturalconstraint.cpp
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

#include "naturalconstraint.h"
#include "object.h"
#include "environment.h"

/// Default constructor.
NaturalConstraint::NaturalConstraint(Environment * environment)
  : _gravitationalFieldStrength(0.0f),
    _mediumDensity(0.0f),
    _mediumViscosity(0.0f),
    _environment(environment)
{
}

void NaturalConstraint::EnforceConstraint(Object * object, float dt)
{
#define WATER_DENSITY 1025.0f
#define AIR_DENSITY 1.2f
#define AIR_VISCOSITY 18.27e-6f
  float waterSurface = Object::s_defaultEnvironment->seaSurface->GetAltitude((maths::Vector<2, float>)object->GetPosition());
  float mediumDensity;
  float mediumViscosity;
  float z = object->GetPosition()[2];
  float mediumFactor;
  bool bouyant = false;
  if (z > waterSurface + object->GetRadius()) {
    mediumFactor = 1.0f;
  } else if (z < waterSurface - object->GetRadius()) {
    mediumFactor = 0.0f;
  } else {
    if (object->IsBouyant()) {
      object->CalculateBouyancy(Object::s_defaultEnvironment->seaSurface, WATER_DENSITY, AIR_DENSITY, _gravitationalFieldStrength);
      bouyant = true;
    }
    mediumFactor = (z - waterSurface + object->GetRadius()) / (object->GetRadius()*2);
  }
  
  mediumDensity = WATER_DENSITY * (1.0f-mediumFactor) + AIR_DENSITY * mediumFactor; // kg/m^3
  mediumViscosity = _mediumViscosity * (1.0f-mediumFactor) + AIR_VISCOSITY * mediumFactor;
  
  
  // Weight and Buoyancy
  // The buoyant mass is the mass of the object minus the mass of the displaced fluid.
  float mass = object->GetMass();
  float buoyantMass = mass - object->GetVolume()*mediumDensity;
  
  // Apply the buoyant weight
  if (!bouyant) {
    object->ApplyForce(_gravitationalFieldStrength * buoyantMass);
  }
  
  // Drag
  // F = 1/2 density v^2 area dragcoef direction
  maths::Vector<3, float> current = _environment->GetCurrent(object->GetPosition());
  maths::Vector<3, float> relativeVelocity = object->GetVelocity();
  relativeVelocity -= current;
 // if (relativeVelocity.sqr() > 0.0f) {
    if (false) {
      float b = 6 * PI * object->GetRadius()*mediumViscosity;
      maths::Vector<3, float> force = _gravitationalFieldStrength;
      float f2 = exp(-b*dt/buoyantMass);
      for (int i = 0; i < 3; ++i) {
        //std::cout << "f1: " << buoyantMass << "*" << force[i] << "/" << b << " - " << relativeVelocity[i] << std::endl;
        float f1 = buoyantMass*force[i]/b - relativeVelocity[i];
        force[i] *= (1.0f-exp(f2)*sqrt(fabs(f1)));
      }
      //std::cout << "force ("<<force[0]<<","<<force[1]<<","<<force[2]<<")" << std::endl;
      force = (force * buoyantMass/b - object->GetVelocity())*(mass/dt);
      object->ApplyForce(force);
      
    } else if (true) {
      if (mass) {
        float drag = mediumViscosity*6*PI*object->GetRadius();
        maths::Vector<3, float> dv = object->GetForces()*(dt/mass);
        if (dv.sqr() > 10.0f*10.0f) {
          maths::Vector<3, float> v = object->GetVelocity();
          int its = (int)(dt * 1000.0f);
          if (!its)
            its = 1;
          float f1m = exp(-2e5f*dt*drag/its);
          float f = 1.0f - f1m;
          maths::Vector<3, float> ddv = dv / its;
          for (int i = 0; i < its; ++i) {
            v += ddv;
            v = v*f1m + current*f;
          }
          object->SetVelocity(v);
          object->ApplyForce(-object->GetForces());
        } else {
          //object->ApplyForce(relativeVelocity * drag);
          object->SetVelocity(current + relativeVelocity * (1.0f-dt*drag*10.0f));
        }
      }
    } else if (false) {
      float speed = relativeVelocity.mag();
      maths::Vector<3, float> direction = relativeVelocity / speed;
      float areaTimesDragCoefficient = object->GetDrag(direction);
      //std::cout << "dragcoef:" << object->GetDrag(direction) << std::endl;
      //std::cout << "rad^2:" << object->GetRadius()*object->GetRadius() << std::endl;
      float drag = -0.5f * mediumDensity * relativeVelocity.sqr() * areaTimesDragCoefficient;
      //std::cout << "vel:" <<relativeVelocity.sqr() << std::endl;
      //std::cout << "mediumDensity:" << mediumDensity << std::endl;
      //std::cout << "drag:" << drag << std::endl;
      
      if (drag > mass*speed/dt*0.1f) {
        drag = mass*speed/dt*0.1f;
      }
      
      // Apply the drag
      //std::cout << drag << std::endl;
      object->ApplyForce(direction * drag);
      //*/
    }
  //}
}
